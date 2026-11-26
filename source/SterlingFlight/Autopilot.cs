// Sparrow RTLS mission autopilot - one button: launch, MECO, stage separation, flip, boostback, grid fins, entry burn,
// aerodynamic descent, legs, landing burn and touchdown at the launch site, while the second stage flies itself to a
// circular orbit. Repeatable: the pad state is saved when the mission starts, "Repeat" reloads it and flies again.
//
// Hooks (all main thread):
//   Vehicle.PrepareWorker prefix  - per vehicle, every sim step: fly whichever role this vehicle has (it runs right
//                                   before KSA copies the vehicle's own throttle / engine-on inputs to the worker)
//   Program.OnDrawUiFrame postfix - mission window
//   InputEvents.ApplyInputEvents  - deferred save / load
// Steering goes through KSA's own flight computer (Auto attitude, EnuBody frame, Custom target), the same way the
// game's EVA jetpack points a vehicle. Engines: each vehicle's own manual inputs (EngineOn / EngineThrottle) and the
// octaweb engine sets (EngineSets.cs). Grid fins get a torque command from here (Autopilot.FinCommand).
// Guidance is modelled on the author's validated kOS RTLS and upper stage scripts.
using System.Collections.Concurrent;
using System.Reflection;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace SterlingFlight
{
    internal sealed class MissionConfig
    {
        // pilot inputs
        public float OrbitKm = 200f;          // circular target orbit
        public float InclinationDeg = 28.5f;  // target inclination (clamped to >= launch latitude)
        // derived at launch from the vehicle and the pad
        public float HeadingDeg = 90f;        // launch azimuth (from inclination, latitude and planet rotation)
        public float BoosterDryT = 23.2f;     // measured: dry mass of the booster parts
        public float UpperT;                  // measured: second stage + payload at launch (fuelled)
        public double LatitudeDeg;
        public double LongitudeDeg;
        public bool AtSupportedSite = true;   // automated flights are only tuned for KSC so far (see Autopilot.SiteCheck)
        public string SiteName = "";
        // mission design constants (Sparrow RTLS profile)
        public float OvershootKm = 1.5f;      // boostback aims this far beyond the pad (drag pulls the booster short)
        public float SepAltKm = 85f;          // stage separation altitude (real RTLS: 70-90 km, ~85 km typical)
        public float FinalSinkMs = 1.5f;      // touchdown sink rate
        public bool WatchBooster = true;      // camera/control follows the booster after separation
        // droneship landing (no boostback): the ship waits ShipKm downrange on the launch azimuth
        public bool Droneship;
        public float ShipKm = 480f;
        public float AsdsOvershootKm = 3f;    // MECO aims the impact this far past the ship; the entry burn pulls it back
    }

    internal static class Autopilot
    {
        public const string SaveName = "Sparrow_RTLS_Pad";
        public static readonly MissionConfig Cfg = new();
        public static bool Active;
        /// booster landed and the upper stage is in orbit: flight control is shut down, the panel offers RECOVER VEHICLE
        public static bool AwaitingRecovery;
        public static string StackPhase = "IDLE", UpperPhase = "-";
        public static readonly List<string> Events = new();
        public static Vehicle? Stack, Booster, Upper;
        static readonly HashSet<Vehicle> PreExisting = new();   // vehicles in the world at launch: never taken as our stages
        public static double PadRadius;                 // pad distance from planet centre (m)
        public static double3 PadCcf;                   // pad position, planet-fixed
        public static double T0 = double.NaN;           // liftoff (sim seconds)
        public static double PhaseStart;
        public static double UpperPhaseStart;
        public static string Telemetry = "", UpperTelemetry = "";
        // grid fin torque request per vehicle, body (assembly) axes, components -1..1
        public static readonly ConcurrentDictionary<Vehicle, double3> FinCommand = new();
        static readonly ConcurrentDictionary<Vehicle, double> LastTick = new();
        // deferred save/load (main loop, outside the UI)
        public static volatile bool WantSave, WantLoad;
        public static bool AutoStartAfterLoad;
        static double _loadedAt;

        public static double Now => Universe.GetElapsedTime().Seconds();

        public static void Event(string s)
        {
            var t = double.IsNaN(T0) ? "T-" : $"T+{Now - T0:0}s";
            var line = $"{t} {s}";
            lock (Events) { Events.Add(line); if (Events.Count > 40) Events.RemoveAt(0); }
            Log.Info("mission " + line);
        }

        // ------------------------------------------------------------------ start / stop
        public static void Start(Vehicle v)
        {
            if (!HasPart(v, Parts.Octaweb)) { Event("no Sparrow octaweb on the controlled vehicle"); return; }
            Measure(v);
            if (!Cfg.AtSupportedSite) { Event($"automated flights are KSC-only for now (pad at {Cfg.LatitudeDeg:0.00}, {Cfg.LongitudeDeg:0.00})"); return; }
            Stack = Booster = Upper = null;
            Stack = v;
            Active = true;
            AwaitingRecovery = false;
            // every other vehicle already in the world (old upper stages, boosters left standing, the drone ship) is not ours
            PreExisting.Clear();
            try { foreach (var a in Universe.CurrentSystem!.All.AsSpan()) if (a is Vehicle o && o != v) PreExisting.Add(o); }
            catch (Exception e) { Log.ErrorOnce("preexisting", "pre-existing vehicles: " + e.Message); }
            T0 = double.NaN;
            var p = v.GetPositionCci();
            PadRadius = V.Len(p);
            if (v.Orbit.Parent is Celestial cel) PadCcf = Double3Ex.Transform(p, cel.GetCci2Ccf());
            lock (Events) Events.Clear();
            Measure(v);
            Webcast.Begin(v);
            if (Cfg.Droneship) DroneShip.Request(PadCcf, Cfg.ShipKm, Cfg.HeadingDeg);
            SetStackPhase("COUNTDOWN");
            UpperPhase = "-";
            Event($"tanks: S1 {FillFraction(v, Parts.S1Tank) * 100:0} %, S2 {FillFraction(v, Parts.S2Tank) * 100:0} %");
            Event((Cfg.Droneship ? $"DRONESHIP landing, ship {Cfg.ShipKm:0} km downrange. " : "RTLS landing. ") + $"mission start: {v.Props.TotalMass / 1000:0.0} t (booster dry {Cfg.BoosterDryT:0.0} t, upper stage + payload {Cfg.UpperT:0.0} t), " +
                  $"orbit {Cfg.OrbitKm:0} km at {Math.Max(Cfg.InclinationDeg, Math.Abs(Cfg.LatitudeDeg)):0.0} deg, pad latitude {Cfg.LatitudeDeg:0.00}, azimuth {Cfg.HeadingDeg:0.0}");
        }

        static readonly HashSet<string> BoosterParts = new() { Parts.Octaweb, Parts.S1Tank, Parts.Interstage, "SparrowGridFinPart", "SparrowLandingLegPart" };

        /// everything the mission needs, read from the vehicle on the pad
        /// extra diagnostics (attitude-thruster lines every 2 s in descent, feed probes): set env STERLING_DEBUG=1
        public static readonly bool DebugLog = Environment.GetEnvironmentVariable("STERLING_DEBUG") == "1";

        static readonly MethodInfo MarkRmDirty = AccessTools.Method(typeof(PartTree), "MarkResourceManagersDirty");

        /// The interstage attitude thrusters burn first-stage propellant, but KSA only lets a thruster draw from tanks whose
        /// RoleAffinity includes Thruster (default Engine only). New builds get Engine+Thruster from the part XML; this also
        /// fixes craft/saves made before that (their tank data stores "Engine").
        public static void EnsureThrusterFeed(Vehicle v)
        {
            try
            {
                bool changed = false;
                foreach (var t in v.Parts.Modules.Get<Tank>())
                {
                    var id = t.FullPart?.Template?.Id;
                    if (id == null || !id.StartsWith(Parts.S1Tank, StringComparison.Ordinal)) continue;
                    if ((t.RoleAffinity & ConsumerRole.Thruster) != 0) continue;
                    t.RoleAffinity |= ConsumerRole.Thruster;
                    changed = true;
                }
                if (changed)
                {
                    MarkRmDirty?.Invoke(v.Parts, null);
                    Event($"{v.Id}: first-stage tank opened to the interstage attitude thrusters");
                }
            }
            catch (Exception ex) { Log.ErrorOnce("thruster-feed", "thruster feed: " + ex.Message); }
        }

        static readonly HashSet<string> _probed = new();

        /// one-shot diagnostic: what each rocket core's propellant feed can reach (attitude thrusters vs. a main engine)
        public static void ProbeFeeds(Vehicle v)
        {
            if (!_probed.Add(v.Id)) return;
            try
            {
                bool mainDone = false;
                var e = v.Parts.RocketNozzles.ModulesAndAllStates.GetEnumerator();
                while (e.MoveNext())
                {
                    var nz = e.Current.Module;
                    if (nz is TurbineExhaustNozzle) continue;
                    bool small = nz.FxExitRadius < SootSkin.MinMainNozzleRadius;
                    if (!small) { if (mainDone) continue; mainDone = true; }
                    var core = nz.Core;
                    string info;
                    if (core is PlumbedCore pc)
                    {
                        var rm = pc.ResourceManager;
                        string order = "no resource manager";
                        if (rm != null)
                        {
                            var co = rm.ConsumptionOrder;
                            int tanks = 0; var names = new List<string>();
                            if (co.IsValid)
                                for (int i = 0; i < co.LevelCount; i++)
                                    foreach (var t in co[i]) { tanks++; if (t != null) names.Add(t.FullPart.Template.Id + (t.PropellantUseEnabled ? "" : "(off)")); }
                            order = $"order valid {co.IsValid}, levels {(co.IsValid ? co.LevelCount : 0)}, tanks {tanks} [{string.Join(", ", names)}]";
                        }
                        info = $"plumbing {pc.PlumbingCapability}, feedTanks {pc.FeedTanks.Length}, {order}";
                    }
                    else info = "core " + core?.GetType().Name;
                    Event($"feed probe {(small ? "thruster" : "main engine")} {nz.Parent?.FullPart?.Template?.Id}/{nz.TemplateId}: {info}");
                }
            }
            catch (Exception ex) { Event("feed probe failed: " + ex.Message); }
        }

        public static void Measure(Vehicle v)
        {
            EnsureThrusterFeed(v);
            double dry = 0, upper = 0;
            foreach (var p in v.Parts.Parts)
            {
                if (p.IsSubPart) continue;
                double m = p.ComputeSubtreeInertMass();
                if (BoosterParts.Contains(p.Template?.Id ?? "")) dry += m; else upper += m;
            }
            if (dry > 1000) Cfg.BoosterDryT = (float)(dry / 1000);
            Cfg.UpperT = (float)((upper + PropellantKg(v, Parts.S2Tank)) / 1000);
            var n = new Nav(v);
            var axis = V.Norm(n.Body.GetAngularVelocityCci());
            double lat = Math.Asin(Math.Clamp(V.Dot(n.Up, axis), -1, 1));
            Cfg.LatitudeDeg = lat * 180 / Math.PI;
            SiteCheck(v);
            // launch azimuth for the inclination (northbound), corrected for the ground's eastward speed
            double inc = Math.Max(Cfg.InclinationDeg, Math.Abs(Cfg.LatitudeDeg) + 0.01) * Math.PI / 180;
            double sinAz = Math.Clamp(Math.Cos(inc) / Math.Max(Math.Cos(lat), 1e-6), -1, 1);
            double azI = Math.Asin(sinAz);
            double r = n.Body.MeanRadius + Cfg.OrbitKm * 1000;
            double vOrb = Math.Sqrt(n.Mu / r);
            double vEq = V.Len(n.Body.GetAngularVelocityCci()) * n.Rmag * Math.Cos(lat);
            double vx = vOrb * Math.Sin(azI) - vEq, vy = vOrb * Math.Cos(azI);
            Cfg.HeadingDeg = (float)((Math.Atan2(vx, vy) * 180 / Math.PI + 360) % 360);
        }

        // Automated launches (ascent, boostback / drone ship, landing) are only tuned and tested from KSC. Other sites
        // (Vandenberg etc.) need their own downrange geometry, so the panel refuses them until that work is done.
        const double KscLatDeg = 28.608299, KscLonDeg = -80.604127, KscRadiusKm = 30;   // CCSFS LC-39A (Content/Core/Astronomicals.xml)
        static string _siteLogged = "";
        static void SiteCheck(Vehicle v)
        {
            try
            {
                if (v.Orbit?.Parent is not Celestial cel) { Cfg.AtSupportedSite = false; Cfg.SiteName = "not on a planet"; return; }
                // explicit sim time: the cached GetCci2Ccf() frame lagged the vehicle position (0.59 deg of longitude)
                var ccf = Double3Ex.Transform(v.GetPositionCci(), cel.GetCci2Ccf(Universe.GetElapsedTime()));
                double r = V.Len(ccf);
                double lat = Math.Asin(Math.Clamp(ccf.Z / r, -1, 1)), lon = Math.Atan2(ccf.Y, ccf.X);
                Cfg.LongitudeDeg = lon * 180 / Math.PI;
                double la0 = KscLatDeg * Math.PI / 180, lo0 = KscLonDeg * Math.PI / 180;
                double c = Math.Sin(lat) * Math.Sin(la0) + Math.Cos(lat) * Math.Cos(la0) * Math.Cos(lon - lo0);
                double km = Math.Acos(Math.Clamp(c, -1, 1)) * cel.MeanRadius / 1000;
                bool earth = (cel.Id ?? "").Contains("Earth", StringComparison.OrdinalIgnoreCase);
                Cfg.AtSupportedSite = earth && km < KscRadiusKm;
                Cfg.SiteName = Cfg.AtSupportedSite ? "KSC" : earth ? $"{km:0} km from KSC" : cel.Id ?? "?";
                var line = $"{cel.Id} {lat * 180 / Math.PI:0.000}, {Cfg.LongitudeDeg:0.000} -> {Cfg.SiteName}";
                if (Cfg.SiteName != _siteLogged) { _siteLogged = Cfg.SiteName; Log.Info("launch site: " + line); }
            }
            catch (Exception ex) { Cfg.AtSupportedSite = true; Log.ErrorOnce("site-check", "site check: " + ex.Message); }
        }

        public static void Abort(string why)
        {
            Event("ABORT: " + why);
            foreach (var v in new[] { Stack, Booster, Upper })
                if (v != null) { SetEngine(v, false, 0f); ReleaseAttitude(v); FinCommand.TryRemove(v, out _); }
            Active = false;
            AwaitingRecovery = false;
            StackPhase = "ABORTED";
            Webcast.OnPhase("ABORTED");
        }

        static bool Alive(Vehicle? v) => v != null && !v.IsDisposed;

        /// booster safe on the pad / ship and the upper stage in orbit: stop every automated control and wait for RECOVER
        static void Complete()
        {
            foreach (var v in new[] { Stack, Booster, Upper })
            {
                if (!Alive(v)) continue;
                SetEngine(v!, false, 0f);
                ReleaseAttitude(v!);
                FinCommand.TryRemove(v!, out _);
            }
            if (Alive(Booster)) Booster!.FlightComputer.RCSMode = FlightComputerRCSMode.Disabled;
            Active = false;
            AwaitingRecovery = true;
            Event("MISSION COMPLETE: flight control shut down, ready to recover");
        }

        /// RECOVER VEHICLE: remove the landed booster (and the drone ship on a drone ship mission) the way the game's own
        /// recover does, hand the camera to the upper stage, and reset the mission so the panel is ready for the next flight
        public static void Recover()
        {
            var gone = new List<string>();
            try
            {
                if (Alive(Upper)) Universe.FollowInHoveredViewport(Upper!);
                // ship first, so the game never hands control to it while the booster is removed
                if (Cfg.Droneship && DroneShip.Alive)
                {
                    InputEvents.VehicleDestroyBuffer.Add(new InputEvents.VehicleDestroyData { Vehicle = DroneShip.Ship!, Recovered = true });
                    gone.Add(DroneShip.Ship!.Id);
                }
                if (Alive(Booster))
                {
                    InputEvents.VehicleDestroyBuffer.Add(new InputEvents.VehicleDestroyData { Vehicle = Booster!, Recovered = true });
                    gone.Add(Booster!.Id);
                }
            }
            catch (Exception e) { Log.Error("recover: " + e); }
            Log.Info("mission: recovered " + (gone.Count > 0 ? string.Join(" + ", gone) : "nothing (already gone)"));
            if (Cfg.Droneship) DroneShip.Ship = null;
            foreach (var v in new[] { Stack, Booster, Upper }) if (v != null) { FinCommand.TryRemove(v, out _); LastTick.TryRemove(v, out _); }
            Stack = Booster = Upper = null;
            PreExisting.Clear();
            Active = false;
            AwaitingRecovery = false;
            StackPhase = "IDLE"; UpperPhase = "-";
            Telemetry = UpperTelemetry = Brief = UpperBrief = "";
            T0 = double.NaN;
            lock (Events) Events.Clear();
            if (Webcast.Showing) Webcast.Leave();
        }

        /// T- countdown length: longer with the webcast so the UI fade and the intro animation play before ignition
        public static float CountdownSeconds => Webcast.Enabled ? Webcast.CountdownWithWebcast : 3f;

        public static string Brief = "", UpperBrief = "";
        public static void SetStackPhase(string p) { StackPhase = p; PhaseStart = Now; Event(p + (Brief.Length > 0 ? "  [" + Brief + "]" : "")); Webcast.OnPhase(p); }
        public static void SetUpperPhase(string p) { UpperPhase = p; UpperPhaseStart = Now; Event("S2 " + p + (UpperBrief.Length > 0 ? "  [" + UpperBrief + "]" : "")); Webcast.OnPhase(p, upper: true); }

        // ------------------------------------------------------------------ per-vehicle tick
        public static void Tick(Vehicle v)
        {
            if (DroneShip.IsShip(v)) { DroneShip.Notice(v); return; }
            if (AutoStartAfterLoad && !Active && Program.ControlledVehicle == v && Now - _loadedAt > 3.0 && HasPart(v, Parts.Octaweb))
            {
                AutoStartAfterLoad = false;
                Start(v);
            }
            if (!Active) return;
            if (StackPhase == "BOOSTER DONE" && (!Alive(Upper) || UpperPhase == "ORBIT")) { Complete(); return; }
            double now = Now;
            double dt = LastTick.TryGetValue(v, out var last) ? Math.Clamp(now - last, 0.0, 1.0) : 0.0;
            LastTick[v] = now;
            if (v.Orbit?.Parent == null) return;

            // roles after staging: the vehicle with the octaweb is the booster, the one with the vacuum engine the upper stage.
            // Which half keeps the original vehicle depends on the build order (root part), so accept either:
            //   stage 1 built first -> the stack stays the booster, the upper stage splits off as a new vehicle
            //   stage 2 built first -> the stack stays the upper stage, the booster splits off as a new vehicle
            // Only a vehicle that just split off OUR stack counts: not one that already existed at launch (2026-11-25: an
            // upper stage left in orbit by the previous flight was taken as this flight's upper stage at liftoff), and
            // only one within 2 km of the stack.
            if (Booster == null && Stack != null && v != Stack && StackPhase != "COUNTDOWN" && Upper == null
                && !PreExisting.Contains(v) && V.Len(V.Sub(v.GetPositionCci(), Stack.GetPositionCci())) < 2000)
            {
                bool hasOw = HasPart(v, Parts.Octaweb), hasMv = HasPart(v, Parts.VacEngine);
                if (hasMv && !hasOw) { Upper = v; Booster = Stack; Event("upper stage separated: " + v.Id); }
                else if (hasOw && !hasMv) { Booster = v; Upper = Stack; Event("booster separated: " + v.Id + " (upper stage keeps " + Stack.Id + ")"); }
            }
            if (Upper == null && Booster == null && v == Stack) { BoosterGuidance.Tick(v, dt); return; }
            if (v == Booster) BoosterGuidance.Tick(v, dt);
            else if (v == Upper) UpperGuidance.Tick(v, dt);
        }

        public static void OnLoaded() { _loadedAt = Now; }

        // ------------------------------------------------------------------ actuators
        static readonly FieldInfo Inputs = AccessTools.Field(typeof(Vehicle), "_manualControlInputs");

        public static void SetEngine(Vehicle v, bool on, float throttle)
        {
            var mi = (ManualControlInputs)Inputs.GetValue(v)!;
            mi.EngineOn = on;
            mi.EngineThrottle = Math.Clamp(throttle, 0f, 1f);
            Inputs.SetValue(v, mi);
        }

        /// point the vehicle's nose (assembly +X) along fwdCci; roll reference rollUpCci (its -Z faces it)
        public static void Point(Vehicle v, double3 fwdCci, double3 rollUpCci, float rateLimit = 0f)
        {
            var fc = v.FlightComputer;
            var f = V.Norm(fwdCci);
            if (V.Len(f) < 0.5) return;
            var u = V.Sub(rollUpCci, V.Mul(f, V.Dot(rollUpCci, f)));
            if (V.Len(u) < 1e-3)
            {   // nose straight along the roll reference: any perpendicular will do
                u = V.Cross(f, new double3(0, 0, 1));
                if (V.Len(u) < 1e-3) u = V.Cross(f, new double3(0, 1, 0));
            }
            u = V.Norm(u);
            var y = V.Norm(V.Cross(f, u));
            var z = V.Mul(u, -1.0);
            var body2Cci = doubleQuat.CreateFromRotationMatrix(new double4x4(f.X, f.Y, f.Z, 0, y.X, y.Y, y.Z, 0, z.X, z.Y, z.Z, 0, 0, 0, 0, 1));
            var ctrl2Cci = doubleQuat.Concatenate(fc.Ctrl2Asmb, body2Cci);
            var enu2Cci = VehicleReferenceFrameEx.GetEnuBody2Cci(v.GetPositionCci());
            if (!enu2Cci.HasValue) return;
            var frame2Desired = doubleQuat.Concatenate(ctrl2Cci, enu2Cci.Value.Inverse());
            fc.RCSMode = FlightComputerRCSMode.Enabled;
            if (fc.AngleDeadband != FlightComputerAttitudeProfile.Strict.GetDeadband())
                fc.SetAttitudeProfile(FlightComputerAttitudeProfile.Strict);
            fc.AttitudeMode = FlightComputerAttitudeMode.Auto;
            fc.AttitudeFrame = VehicleReferenceFrame.EnuBody;
            fc.AttitudeTrackTarget = FlightComputerAttitudeTrackTarget.Custom;
            fc.CustomAttitudeTarget = VehicleReferenceFrame.EnuBody.QuaternionToEulerAngles(frame2Desired);
            if (rateLimit > 0f) fc.RateLimit = rateLimit;
        }

        public static void ReleaseAttitude(Vehicle v)
        {
            var fc = v.FlightComputer;
            fc.AttitudeMode = FlightComputerAttitudeMode.Manual;
            fc.AttitudeTrackTarget = FlightComputerAttitudeTrackTarget.None;
            fc.CustomAttitudeTarget = double3.Zero;
        }

        /// grid fin torque request from the attitude error (vehicle nose vs. wanted nose), body axes
        public static void FinsToward(Vehicle v, double3 wantCci)
        {
            var b2c = v.GetBody2Cci();
            var noseCci = Double3Ex.Transform(new double3(1, 0, 0), b2c);
            var axisCci = V.Cross(noseCci, V.Norm(wantCci));             // rotate nose toward want
            var axisBody = Double3Ex.Transform(axisCci, b2c.Inverse());
            var s = V.Len(axisBody);
            var cmd = s > 1e-6 ? V.Mul(axisBody, Math.Min(1.0, s * 4.0) / s) : new double3(0, 0, 0);
            FinCommand[v] = cmd;
        }

        public static bool Deploy(Vehicle v, string moduleId, bool deploy)
        {
            bool any = false;
            var mods = v.Parts.Modules.Get<KeyframeAnimationModule>();
            for (int i = 0; i < mods.Length; i++)
            {
                if (mods[i].TemplateId != moduleId) continue;
                mods[i].TimeGoal = deploy ? mods[i].Shared.Duration : 0f;
                any = true;
            }
            return any;
        }

        public static bool FireDecouplers(Vehicle v, string partId)
        {
            bool any = false;
            var mods = v.Parts.Modules.Get<Decoupler>();
            for (int i = 0; i < mods.Length; i++)
            {
                if (mods[i].Parent?.FullPart?.Template?.Id != partId) continue;
                mods[i].SetIsActive(v, true);
                any = true;
            }
            return any;
        }

        /// octaweb engine set (9 / 3 / 1); lights the groups if needed
        public static bool EngineSet(Vehicle v, int n)
        {
            foreach (var p in v.Parts.Parts)
            {
                if (p.Template?.Id != Parts.Octaweb) continue;
                if (!EngineSets.Find(p, out var c, out var pair, out var outer)) continue;
                EngineSets.Mode[c!.InstanceId] = n;
                if (!c.IsActive) c.SetIsActive(v, true);
                EngineSets.Enforce(v, c, pair!, outer!);
                return true;
            }
            return false;
        }

        public static bool ActivateEngines(Vehicle v, string partId)
        {
            bool any = false;
            foreach (var p in v.Parts.Parts)
            {
                if (p.Template?.Id != partId) continue;
                var m = p.Modules.Get<EngineController>();
                for (int i = 0; i < m.Length; i++) { if (!m[i].IsActive) m[i].SetIsActive(v, true); any = true; }
            }
            return any;
        }

        /// fill fraction of the tanks on parts with this id (-1: no such tank)
        public static double FillFraction(Vehicle v, string tankPartId)
        {
            var tree = v.Parts;
            var states = tree.Moles.States;
            double sum = 0; int n = 0;
            var tanks = tree.Tanks.Modules;
            for (int i = 0; i < tanks.Length; i++)
                if (tanks[i].Parent?.FullPart?.Template?.Id == tankPartId) { sum += tanks[i].FilledFraction(states); n++; }
            return n == 0 ? -1 : sum / n;
        }

        public static bool HasPart(Vehicle v, string id)
        {
            foreach (var p in v.Parts.Parts) if (p.Template?.Id == id) return true;
            return false;
        }

        public static double PropellantKg(Vehicle v, string tankPartId)
        {
            var tree = v.Parts;
            var states = tree.Moles.States;
            double kg = 0;
            var tanks = tree.Tanks.Modules;
            for (int i = 0; i < tanks.Length; i++)
                if (tanks[i].Parent?.FullPart?.Template?.Id == tankPartId) kg += tanks[i].ComputeSubstanceMass(states);
            return kg;
        }
    }

    internal static class Parts
    {
        public const string Octaweb = "SparrowOctawebPart", S1Tank = "SparrowS1TankPart", Interstage = "SparrowInterstagePart",
            S2Tank = "SparrowS2TankPart", VacEngine = "AnvilVacEnginePart", Fairing = "SparrowFairingHalfPart",
            GridFinAnim = "SparrowGridFinDeploy", LegAnim = "SparrowLandingLegDeploy";
    }

    /// local navigation state of one vehicle (CCI unless noted)
    internal readonly struct Nav
    {
        public readonly double3 R, Vel, VSurf, Up, East, North, Nose;
        public readonly double Rmag, Alt, RadarAlt, Vz, VhSurf, Mu, G, Mass, Q;
        public readonly IParentBody Body;

        public Nav(Vehicle v)
        {
            Body = v.Orbit.Parent;
            R = v.GetPositionCci(); Vel = v.GetVelocityCci(); VSurf = v.GetSurfaceVelocityCci();
            Rmag = V.Len(R); Up = V.Mul(R, 1.0 / Rmag);
            var axis = V.Norm(Body.GetAngularVelocityCci());
            if (V.Len(axis) < 0.5) axis = new double3(0, 0, 1);
            East = V.Norm(V.Cross(axis, Up));
            North = V.Cross(Up, East);
            Mu = Body.Mu; G = Mu / (Rmag * Rmag);
            Alt = Rmag - Body.MeanRadius;
            RadarAlt = v.GetRadarAltitude();
            Vz = V.Dot(VSurf, Up);
            VhSurf = V.Len(V.Sub(VSurf, V.Mul(Up, Vz)));
            Nose = Double3Ex.Transform(new double3(1, 0, 0), v.GetBody2Cci());
            Mass = v.Props.TotalMass;
            Q = v.ComputeDynamicPressure();
        }

        public double3 Heading(double headingDeg, double pitchDeg)
        {
            double h = headingDeg * Math.PI / 180, p = pitchDeg * Math.PI / 180;
            var hor = V.Add(V.Mul(North, Math.Cos(h)), V.Mul(East, Math.Sin(h)));
            return V.Norm(V.Add(V.Mul(hor, Math.Cos(p)), V.Mul(Up, Math.Sin(p))));
        }

        public double3 Horizontal(double3 x) => V.Sub(x, V.Mul(Up, V.Dot(x, Up)));
        public double AngleTo(double3 dir) => Math.Acos(Math.Clamp(V.Dot(V.Norm(Nose), V.Norm(dir)), -1, 1)) * 180 / Math.PI;
    }

    // ====================================================================== booster (and the stack before staging)
    internal static class BoosterGuidance
    {
        const double AnvilSlN = 845e3, AnvilVacN = 914e3, Ve = 2950;   // per Anvil 1D, effective exhaust velocity
        static bool _maxQ, _finsOut, _legsOut, _entryDone, _bbLow;
        static double _pitchBias, _entryStartSpeed;
        static double _phaseFlag;
        static double3 _bbDir;

        static double3 _prevVel; static bool _havePrev; static double _beta, _betaDense;
        static double3 _aeroAcc; static bool _haveAeroAcc;   // smoothed non-gravity acceleration (engines off), CCI
        static double _lastAeroDiag;
        static double _lastAeroLog, _lastLandLog;
        static double3? _lastLandHv;

        /// mass / (Cd A) from the measured drag deceleration (inertial dv/dt minus gravity), engines off only
        static void MeasureBeta(Vehicle v, Nav n, double dt, bool enginesOff)
        {
            if (_havePrev && dt > 0.02 && enginesOff && n.Q > 300)
            {
                var acc = V.Mul(V.Sub(n.Vel, _prevVel), 1.0 / dt);
                var g = V.Mul(n.R, -n.Mu / (n.Rmag * n.Rmag * n.Rmag));
                var drag = V.Sub(acc, g);
                _aeroAcc = _haveAeroAcc ? V.Add(_aeroAcc, V.Mul(V.Sub(drag, _aeroAcc), Math.Min(1.0, dt / 0.5))) : drag;
                _haveAeroAcc = true;
                var air = V.Norm(n.VSurf);
                double ad = -V.Dot(drag, air);
                if (ad > 0.2)
                {
                    double b = n.Q / ad;
                    if (b > 50 && b < 50000) _beta = _beta <= 0 ? b : _beta + (b - _beta) * Math.Min(1.0, dt / 2.0);
                    if (b > 50 && b < 50000 && n.Q > 2000)
                    {
                        // 2026-11-25 drone ship misses: the first dense sample replaced the 1500 default outright (1500 -> 1176
                        // in one step), the predicted miss jumped by ~800 m and the big tilt it asked for at high q diverged.
                        // Start from the default, limit each sample to +-30 % of the estimate, and blend slowly (8 s).
                        if (_betaDense <= 0) _betaDense = 1500;
                        double bc = Math.Clamp(b, _betaDense * 0.7, _betaDense * 1.3);
                        _betaDense += (bc - _betaDense) * Math.Min(1.0, dt / 8.0);
                    }
                }
            }
            _prevVel = n.Vel; _havePrev = true;
        }

        public static double Beta(Nav n) => Autopilot.Cfg.Droneship
            ? (_betaDense > 0 ? _betaDense : 1500)                             // thin-air measurements are noise (flight 2: 6800 vs 1460)
            : (_beta > 0 ? _beta : n.Mass / 10.5);                              // fallback: Cd 1 on the 3.7 m body

        static double _lastRcsLog = double.NegativeInfinity;

        /// diagnostic: are the attitude thrusters (small nozzles: interstage cold gas, any RCS) seen, firing, making thrust,
        /// and what attitude authority does KSA's flight computer think it has (rad/s^2 per control axis)
        static void LogAttitudeThrusters(Vehicle v)
        {
            Autopilot.ProbeFeeds(v);
            int nz = 0, firing = 0; double thrust = 0, mdot = 0;
            var e = v.Parts.RocketNozzles.ModulesAndAllStates.GetEnumerator();
            while (e.MoveNext())
            {
                var c = e.Current;
                if (c.Module is TurbineExhaustNozzle || c.Module.FxExitRadius >= SootSkin.MinMainNozzleRadius) continue;
                nz++;
                if (c.State.DutyCycle > 0f) firing++;
                double md = c.State.Performance.MassFlowRate * c.State.DutyCycle;
                mdot += md;
                thrust += md * c.State.Performance.EffectiveExhaustVelocity;
            }
            var fc = v.FlightComputer;
            var a = fc.RcsTorqueAuthority; var r = v.BodyRates;
            var cs = fc.ActiveControlSystem;
            Autopilot.Event($"rcs: {nz} small nozzles, {firing} firing, {thrust / 1000:0.0} kN, {mdot:0.00} kg/s | mode {fc.RCSMode}, " +
                            $"controllers {fc.VehicleConfig.Thrusters.Count}, axes {cs.X}/{cs.Y}/{cs.Z}, authority " +
                            $"{a.X * 57.3:0.00}/{a.Y * 57.3:0.00}/{a.Z * 57.3:0.00} deg/s2, rates {r.X * 57.3:0.0}/{r.Y * 57.3:0.0}/{r.Z * 57.3:0.0} deg/s");
        }

        public static void Tick(Vehicle v, double dt)
        {
            Autopilot.EnsureThrusterFeed(v);
            var n = new Nav(v);
            var ph = Autopilot.StackPhase;
            MeasureBeta(v, n, dt, ph == "COAST" || ph == "AERO" || ph == "FLIP");
            double tPh = Autopilot.Now - Autopilot.PhaseStart;
            var pad = TargetNowCci(n);
            var bottom = BottomAltitude(v, n);
            Autopilot.Brief = $"alt {n.Alt / 1000:0.0} km, radar {bottom:0} m, {V.Len(n.VSurf):0} m/s, vz {n.Vz:0}";
            Autopilot.Telemetry = $"{ph}  alt {n.Alt / 1000:0.0} km  radar {bottom:0} m  vz {n.Vz:0} m/s  vh {n.VhSurf:0} m/s  " +
                                  $"q {n.Q / 1000:0.0} kPa  m {n.Mass / 1000:0.0} t";
            if (Autopilot.DebugLog && (ph == "FLIP" || ph == "BOOSTBACK" || ph == "COAST" || ph == "AERO") && Autopilot.Now - _lastRcsLog > 2.0)
            {
                _lastRcsLog = Autopilot.Now;
                LogAttitudeThrusters(v);
            }
            switch (ph)
            {
                case "COUNTDOWN":
                    _maxQ = _finsOut = _legsOut = _entryDone = _bbLow = false;
                    _pitchBias = 0; _beta = 0; _betaDense = 0; _havePrev = false; _steerSign = 1; _lastSteerMiss = -1;
                    _haveAeroAcc = false; _lastAeroDiag = 0;
                    Autopilot.Point(v, n.Up, n.Heading(Autopilot.Cfg.HeadingDeg + 180, 0), 6f);
                    if (tPh > Autopilot.CountdownSeconds)
                    {
                        Autopilot.EngineSet(v, 9);
                        Autopilot.SetEngine(v, true, 1f);
                        Autopilot.SetStackPhase("IGNITION");
                    }
                    break;

                case "IGNITION":
                    Autopilot.Point(v, n.Up, n.Heading(Autopilot.Cfg.HeadingDeg + 180, 0), 6f);
                    Autopilot.SetEngine(v, true, 1f);
                    if (tPh > 2.0) { Autopilot.T0 = Autopilot.Now; Autopilot.SetStackPhase("ASCENT"); }
                    break;

                case "ASCENT":
                {
                    // pitch program (kOS v3: vertical to 500 m, then 90 -> ~35 deg by MECO altitude), held within
                    // 5 deg of surface prograde once moving fast (keeps the angle of attack small through max-Q)
                    double h = n.Alt, vs = V.Len(n.VSurf);
                    double prog = Math.Clamp(90 - 62 * Math.Sqrt(Math.Clamp((h - 400) / 68000.0, 0, 1)) + _pitchBias, 20, 90);
                    double pitch;
                    if (vs < 60) pitch = 90;                                              // clear the tower
                    else if (vs < 150) pitch = Math.Min(prog, 90 - 4 * (vs - 60) / 90);   // pitch kick
                    else
                    {
                        var pro = V.Norm(n.VSurf);
                        double proPitch = Math.Asin(Math.Clamp(V.Dot(pro, n.Up), -1, 1)) * 180 / Math.PI;
                        pitch = Math.Clamp(prog, proPitch - 5, Math.Min(proPitch + 5, 88));  // never back to vertical
                    }
                    var dir = n.Heading(Autopilot.Cfg.HeadingDeg, pitch);
                    Autopilot.Point(v, dir, n.Up, 3f);
                    float thr = 1f;
                    if (n.Q > 30000) { thr = 0.72f; if (!_maxQ) { _maxQ = true; Autopilot.Event("max-Q throttle down"); Webcast.OnPhase("MAXQ"); } }
                    Autopilot.SetEngine(v, true, thr);
                    // RTLS MECO: stop when the booster holds just the propellant the return needs
                    double prop = Autopilot.PropellantKg(v, Parts.S1Tank);
                    double dry = Autopilot.Cfg.BoosterDryT * 1000;
                    double dv = 1.6 * n.VhSurf + 300 /*boostback*/ + 650 /*entry*/ + 550 /*landing*/;
                    double need = dry * (Math.Exp(dv / Ve) - 1.0) * 1.12;
                    if (Autopilot.Cfg.Droneship) need = dry * (Math.Exp((650.0 + 550.0 + 150.0) / Ve) - 1.0) * 1.12;   // entry + landing only
                    Autopilot.Telemetry += $"  S1 prop {prop / 1000:0.0} t (RTLS needs {need / 1000:0.0} t)";
                    // predicted altitude at MECO from the propellant left above the RTLS reserve; trim the pitch
                    // program so separation happens near SepAltKm (the real RTLS flies a lofted ascent)
                    double mdot = 9 * AnvilSlN * thr / Ve;
                    double tm = Math.Max(0, (prop - need) / mdot);
                    double aUp = 9 * AnvilSlN * thr / Math.Max(n.Mass, 1) * Math.Sin(pitch * Math.PI / 180) - n.G;
                    double predAlt = n.Alt + n.Vz * tm + 0.5 * aUp * tm * tm;
                    if (tPh > 60 && tm > 8 && !Autopilot.Cfg.Droneship)
                        _pitchBias = Math.Clamp(_pitchBias + (Autopilot.Cfg.SepAltKm * 1000 - predAlt) / 1000.0 * 0.02 * dt, -8, 8);
                    Autopilot.Telemetry += $"  MECO in {tm:0} s at ~{predAlt / 1000:0} km (pitch trim {_pitchBias:+0.0;-0.0})";
                    if (tPh > 5) Webcast.Predict("MECO", tm);
                    bool shipCut = false;
                    if (Autopilot.Cfg.Droneship && tPh > 60 && n.Vz > 0)
                    {
                        UpdateDownrange(n, pad);
                        Autopilot.Telemetry += $"  impact {_iipKm:0} km / ship {_shipKm:0} km (+{Autopilot.Cfg.AsdsOvershootKm:0})";
                        shipCut = _iipKm >= _shipKm + Autopilot.Cfg.AsdsOvershootKm;
                    }
                    if (prop <= need || prop < 1000 || shipCut)
                    {
                        if (Autopilot.Cfg.Droneship && !shipCut) Autopilot.Event($"drone ship out of reach: MECO on the landing reserve (impact {_iipKm:0} km, ship {_shipKm:0} km)");
                        if (shipCut) Autopilot.Event($"MECO on target: impact {_iipKm:0} km, ship {_shipKm:0} km");
                        Autopilot.SetEngine(v, false, 0f);
                        Autopilot.SetStackPhase("MECO");
                        Autopilot.Event($"S1 propellant {prop / 1000:0.0} t, RTLS reserve {need / 1000:0.0} t");
                    }
                    break;
                }

                case "MECO":
                    Autopilot.SetEngine(v, false, 0f);
                    Autopilot.Point(v, V.Norm(n.VSurf), n.Up, 3f);
                    if (tPh > 2.0)
                    {
                        if (!Autopilot.FireDecouplers(v, Parts.Interstage)) Autopilot.Event("no interstage decoupler found");
                        Autopilot.SetStackPhase("STAGE SEP");
                    }
                    break;

                case "STAGE SEP":
                    Autopilot.SetEngine(v, false, 0f);
                    if (Autopilot.Booster != null && tPh > 3.0)
                    {
                        Autopilot.Booster = v;
                        if (Autopilot.Cfg.WatchBooster && Program.ControlledVehicle != v) TrySetControlled(v);
                        Autopilot.Deploy(v, Parts.GridFinAnim, true);
                        _finsOut = true;
                        Autopilot.SetStackPhase(Autopilot.Cfg.Droneship ? "COAST" : "FLIP");
                    }
                    else if (tPh > 15.0) Autopilot.Abort("stage separation did not happen");
                    break;

                case "FLIP":
                {
                    PredictEntry(n);
                    // flip on the interstage attitude thrusters, main engines off (like the real cold-gas flip)
                    var want = BoostbackDir(n, pad);
                    v.FlightComputer.RCSMode = FlightComputerRCSMode.Enabled;
                    Autopilot.Point(v, want, n.Up, 10f);
                    Autopilot.SetEngine(v, false, 0f);
                    Autopilot.Telemetry += $"  flip {n.AngleTo(want):0} deg to go";
                    if (n.AngleTo(want) < 10 || tPh > 90)
                    {
                        if (tPh > 90) Autopilot.Event("flip slow: igniting anyway");
                        Autopilot.SetStackPhase("BOOSTBACK"); _bbLow = false;
                    }
                    break;
                }

                case "BOOSTBACK":
                {
                    PredictEntry(n);
                    var (impact, tti) = Impact.Predict(n, Autopilot.PadRadius);
                    var aim = AimPoint(n, pad, Autopilot.Cfg.OvershootKm * 1000);
                    var err = n.Horizontal(V.Sub(aim, impact));               // where the impact has to move
                    double miss = V.Len(err);
                    var home = V.Norm(n.Horizontal(V.Sub(pad, n.R)));
                    double along = V.Dot(V.Sub(impact, aim), home);           // > 0: impact already beyond the aim
                    _bbDir = miss > 1 ? V.Norm(V.Add(V.Norm(err), V.Mul(n.Up, 0.03))) : _bbDir;
                    Autopilot.Point(v, _bbDir, n.Up, 6f);
                    if (!_bbLow && miss < 5000) { _bbLow = true; Autopilot.EngineSet(v, 1); }
                    else if (!_bbLow) Autopilot.EngineSet(v, 3);
                    bool aligned = n.AngleTo(_bbDir) < 15;
                    Autopilot.SetEngine(v, aligned, _bbLow ? (float)Math.Clamp(miss / 5000.0, 0.0, 1.0) : 1f);
                    Autopilot.Telemetry += $"  impact miss {miss / 1000:0.00} km";
                    if (miss < 150 || (along > 0 && miss < 1500) || tPh > 180)
                    {
                        Autopilot.SetEngine(v, false, 0f);
                        Autopilot.Event($"boostback cut: miss {miss:0} m, impact {(tti):0} s out");
                        Autopilot.SetStackPhase("COAST");
                    }
                    break;
                }

                case "COAST":
                case "AERO":
                {
                    Autopilot.SetEngine(v, false, 0f);
                    if (!_finsOut) { Autopilot.Deploy(v, Parts.GridFinAnim, true); _finsOut = true; }
                    PredictEntry(n);
                    var want = AeroSteer(n, pad, 15);
                    Autopilot.Point(v, want, n.Up, 5f);
                    Autopilot.FinsToward(v, want);
                    // entry burn (3 engines) protects the booster through the dense air
                    if (!_entryDone && n.Vz < 0 && n.Alt < 60000 && n.Alt > 25000 && V.Len(n.VSurf) > 450)
                    {
                        _entryStartSpeed = V.Len(n.VSurf);
                        Autopilot.SetStackPhase("ENTRY BURN");
                    }
                    LegsAndBurnCheck(v, n, bottom);
                    break;
                }

                case "ENTRY BURN":
                {
                    v.FlightComputer.RCSMode = FlightComputerRCSMode.Enabled;
                    var retro = V.Mul(V.Norm(n.VSurf), -1.0);
                    bool onTarget = false;
                    if (Autopilot.Cfg.Droneship)
                    {
                        // burn until the impact point comes back onto the ship; lean a little to null cross-range
                        UpdateDownrange(n, pad);
                        var (imp, _) = Impact.PredictDrag(n, TargetRadius(n), Beta(n), true);
                        var c = n.Horizontal(V.Sub(pad, imp));
                        var hvd = V.Norm(n.Horizontal(n.VSurf));
                        var cross = V.Sub(c, V.Mul(hvd, V.Dot(c, hvd)));
                        double cm = V.Len(cross);
                        if (cm > 1) retro = V.Norm(V.Add(retro, V.Mul(cross, Math.Min(cm / 2000.0, 0.25) / cm)));
                        Autopilot.Telemetry += $"  impact {_iipKm - _shipKm:+0.0;-0.0} km along, {cm / 1000:0.00} km across";
                        // also need the cross-range nulled (flight 4 cut at 1.57 km across; the aero phase could not recover it)
                        // ...but not at any price: 2026-11-25 19:19 the burn kept going for the cross-range while the impact
                        // ran 1.7 km short of the ship; the aero phase fixed that but arrived with 48 m/s sideways and the
                        // landing came down 132 m short. Once the impact is 0.5 km short, cut and leave the rest to the fins.
                        double alongKm = _iipKm - _shipKm;
                        onTarget = tPh > 2 && alongKm <= 0.2 && (cm < 300 || alongKm <= -0.5);
                        if (onTarget && cm >= 300) Autopilot.Event($"entry burn cut before cross-range nulled ({cm:0} m across) to stop the impact running short");
                    }
                    Autopilot.Point(v, retro, n.Up, 5f);
                    Autopilot.FinsToward(v, retro);
                    Autopilot.EngineSet(v, 3);
                    Autopilot.SetEngine(v, n.AngleTo(retro) < 20, 1f);
                    double sp = V.Len(n.VSurf);
                    double minFrac = Autopilot.Cfg.Droneship ? 0.35 : 0.65;
                    if (onTarget) Autopilot.Event($"entry burn cut on target: impact {_iipKm - _shipKm:+0.0;-0.0} km from the ship");
                    if (onTarget || sp < Math.Max(_entryStartSpeed * minFrac, Autopilot.Cfg.Droneship ? 350 : 450) || n.Alt < 25000 || tPh > (Autopilot.Cfg.Droneship ? 60 : 40))
                    {
                        Autopilot.SetEngine(v, false, 0f);
                        _entryDone = true;
                        Autopilot.SetStackPhase("AERO");
                    }
                    break;
                }

                case "LANDING BURN":
                    LandingBurn(v, n, pad, bottom, dt);
                    break;

                case "LANDED":
                    Autopilot.SetEngine(v, false, 0f);
                    Autopilot.Point(v, n.Up, n.Heading(0, 0), 3f);
                    if (tPh > 6)
                    {
                        Autopilot.ReleaseAttitude(v);
                        Autopilot.FinCommand.TryRemove(v, out _);
                        var off = V.Len(n.Horizontal(V.Sub(n.R, pad)));
                        Autopilot.Event($"booster safe: {off:0.0} m from the pad, {Autopilot.PropellantKg(v, Parts.S1Tank) / 1000:0.0} t propellant left");
                        Autopilot.SetStackPhase("BOOSTER DONE");
                    }
                    break;
            }
        }

        /// webcast: seconds until the booster falls through the entry-burn altitude (60 km)
        static void PredictEntry(Nav n)
        {
            if (_entryDone || n.Alt <= 60000) return;
            double g = Math.Max(n.G, 1), h = n.Alt - 60000;
            Webcast.Predict("ENTRY", (n.Vz + Math.Sqrt(n.Vz * n.Vz + 2 * g * h)) / g);
        }

        static void LegsAndBurnCheck(Vehicle v, Nav n, double bottom)
        {
            // legs deploy in the landing burn (LegDeployDelay after ignition), not before
            // hoverslam start: simulate the fall with one engine at 95 % (after ~1 s spool-up), gravity and the
            // booster's measured drag in the thickening air; light when that burn would just stop it 20 m up
            double stop = BurnStopDrop(n, 0.95);
            Autopilot.Telemetry += $"  burn at {stop + 20:0} m";
            bool allowed = (_entryDone || n.Alt < 20000) && n.Alt < 15000;
            if ((_entryDone || n.Alt < 20000) && n.Vz < -5)
            {
                // webcast: ignition when the stopping distance catches up with the height left (and below 15 km)
                double vd = -n.Vz;
                double eta = Math.Max(Math.Max(bottom - 20 - stop, 0) / vd, Math.Max(n.Alt - 15000, 0) / vd);
                Webcast.Predict("LANDBURN", eta);
                Webcast.Predict("LANDING", eta + 2 * (stop + 20) / vd);
            }
            if (allowed && n.Vz < -5 && stop >= bottom - 20) Autopilot.SetStackPhase("LANDING BURN");
        }

        static double OneEngineAccel(Nav n) => AnvilSlN / Math.Max(n.Mass, 1000);

        /// height lost before a one-engine burn at thrustFrac (lit after 1 s) brings the descent to a stop, with drag
        static double BurnStopDrop(Nav n, double thrustFrac)
        {
            double v = Math.Max(0, -n.Vz), vtot = Math.Max(V.Len(n.VSurf), 1);
            double beta = Beta(n), rho0 = 2 * n.Q / (vtot * vtot), aT = OneEngineAccel(n) * thrustFrac;
            double drop = 0, dt = 0.05;
            for (int i = 0; i < 6000 && v > 0; i++)
            {
                double rho = rho0 * Math.Exp(drop / 7200.0);                 // denser as it falls
                double a = (i * dt < 1.0 ? 0 : aT) + 0.5 * rho * v * v / beta - n.G;
                v -= a * dt;
                if (v > 0) drop += v * dt;
            }
            return drop;
        }

        static void LandingBurn(Vehicle v, Nav n, double3 pad, double bottom, double dt)
        {
            Autopilot.EngineSet(v, 1);
            // user: legs out 1.5 s after the landing burn lights (sim time); safety: by 600 m above the pads regardless
            if (!_legsOut && (Autopilot.Now - Autopilot.PhaseStart > LegDeployDelay || bottom - LegClearance < 600)) { Autopilot.Deploy(v, Parts.LegAnim, true); _legsOut = true; Autopilot.Event("legs deploy"); }
            double aMax = OneEngineAccel(n);
            double hover = n.G / aMax;                                       // throttle that holds weight
            double vDown = -n.Vz;
            double thr;
            // 'bottom' is the engine bells; the leg pads reach ~4.1 m below them (flight 6: bells 4.4 m up, pads 0.3 m)
            double padH = bottom - LegClearance;
            if (padH > 4)
            {
                // suicide burn: constant-deceleration profile that only reaches FinalSink 1.5 m above the pads' contact
                double h = Math.Max(padH - 1.5, 0.5);
                // minus the braking the air already provides (q / ballistic coefficient)
                // only half the current air braking is credited: it fades as the booster slows (flights 3-4 ran out of
                // throttle near the ground, thr 1.75-3.2 below 60 m)
                double need = (vDown * vDown - Autopilot.Cfg.FinalSinkMs * Autopilot.Cfg.FinalSinkMs) / (2 * h) + n.G - 0.5 * n.Q / Beta(n);
                thr = need / aMax;
            }
            else
            {
                // last few metres: hold the touchdown sink rate, no hover-and-translate (user: one solid trajectory
                // to the deck centre, slowing right before touchdown)
                double sink = Math.Clamp(Autopilot.Cfg.FinalSinkMs + padH * 0.15, Autopilot.Cfg.FinalSinkMs, 2.5);
                thr = hover + 0.08 * (-sink - n.Vz);
            }
            Autopilot.SetEngine(v, true, (float)Math.Clamp(thr, 0.0, 1.0));

            // steering: retrograde (+ target correction) high up, upright with horizontal-velocity kill low down
            var hv = n.Horizontal(n.VSurf);
            var toPad = n.Horizontal(V.Sub(pad, n.R));
            // horizontal acceleration that puts the booster over the pad with zero sideways speed at touchdown
            double tgo = Math.Max(2.0 * Math.Max(bottom, 1) / Math.Max(vDown, 1.0), 1.5);
            Webcast.Predict("LANDING", bottom > 25 ? tgo + 10 : bottom / Math.Max(vDown, 0.5));
            // ZEM/ZEV gains fade as time-to-go grows near the ground (flight 6: 20 m off, 4.6 m/s sideways), so
            // blend into a fixed position/velocity hold (wn ~0.45 rad/s, about critically damped) below ~200 m
            var aZem = V.Sub(V.Mul(toPad, 6.0 / (tgo * tgo)), V.Mul(hv, 4.0 / tgo));
            // position/velocity hold, approach speed capped (flight 3: chased a target 150 m away and touched down at 7.9 m/s sideways)
            var vDes = V.Mul(toPad, 0.22);
            double vDesLen = V.Len(vDes);
            if (vDesLen > MaxHoldApproach) vDes = V.Mul(vDes, MaxHoldApproach / vDesLen);
            var aHold = V.Mul(V.Sub(vDes, hv), 0.9);
            double w = Math.Clamp((bottom - 60) / 150.0, 0, 1);
            var aH = V.Add(V.Mul(aZem, w), V.Mul(aHold, 1 - w));
            double maxTiltDeg = bottom > 200 ? 12.0 : 5.0 + 7.0 * bottom / 200.0;
            // thick air: body + fin lift from the tilt outweighs its thrust component and pushes the OTHER way (flight 1
            // tilted back ~10 deg to brake and flew 52 m past; flight 3 stayed upright and drag stopped it 134 m short).
            // So above ~7 kPa steer like the aero phase (reversed sign, max 6 deg); thrust steering again below 4 kPa.
            double qk = n.Q / 1000.0;
            double steerSign = 1.0 - 2.0 * Math.Clamp((qk - 4.0) / 3.0, 0, 1);
            if (qk > 4) maxTiltDeg = Math.Min(maxTiltDeg, 6.0);
            aH = V.Mul(aH, steerSign);
            double maxTilt = maxTiltDeg * Math.PI / 180;
            double aT = Math.Max(aMax * Math.Max(thr, 0.3), 1.0);
            double aHmax = aT * Math.Tan(maxTilt);
            double aHlen = V.Len(aH);
            if (aHlen > aHmax) aH = V.Mul(aH, aHmax / aHlen);
            var want = V.Norm(V.Add(n.Up, V.Mul(aH, 1.0 / aT)));
            if (Autopilot.Now - _lastLandLog > 1.0)
            {
                // achieved sideways accel along the commanded direction since the last line (aero can cancel the tilt)
                double cmdA = V.Len(aH), gotA = 0;
                if (_lastLandHv.HasValue && cmdA > 1e-3) gotA = V.Dot(V.Sub(hv, _lastLandHv.Value), V.Mul(aH, 1 / cmdA)) / (Autopilot.Now - _lastLandLog);
                _lastLandHv = hv;
                _lastLandLog = Autopilot.Now;
                Log.Info($"land: h {bottom:0.0} m, sink {vDown:0.0} m/s, pad {V.Len(toPad):0.0} m, side {V.Len(hv):0.00} m/s, thr {thr:0.00}, side accel cmd {cmdA:0.00} got {gotA:0.00} m/s2, q {n.Q / 1000:0.0} kPa, steer sign {steerSign:+0.00;-0.00}, tilt want {Math.Acos(Math.Clamp(V.Dot(want, V.Norm(n.Up)), -1, 1)) * 180 / Math.PI:0.0} got {n.AngleTo(n.Up):0.0} deg, off-axis {n.AngleTo(want):0.0} deg");
            }
            Autopilot.Point(v, want, n.Heading(0, 0), 8f);
            Autopilot.FinsToward(v, want);
            Autopilot.Telemetry += $"  thr {thr:0.00}  pad {V.Len(toPad):0} m  side {V.Len(hv):0.0} m/s";

            bool legContact = false;
            foreach (var l in LegRegistry.Legs.Values)
                if (l.Module?.Parent?.FullPart?.Tree == v.Parts && l.Engaged) legContact = true;
            // also: resting on something (pads within 1 m, no vertical motion for 0.5 s) even if no leg reports contact
            // (flight 6 sat on the deck at 4.4 m with the engine still running)
            if (padH < 1.0 && Math.Abs(n.Vz) < 0.3) { if (_restSince < 0) _restSince = Autopilot.Now; } else _restSince = -1;
            bool resting = _restSince >= 0 && Autopilot.Now - _restSince > 0.5;
            if ((legContact && n.Vz > -2.5) || resting || (bottom < 1.0 && Math.Abs(n.Vz) < 1.0))
            {
                Autopilot.SetEngine(v, false, 0f);
                Autopilot.Event($"TOUCHDOWN {(-n.Vz):0.0} m/s down, {V.Len(hv):0.0} m/s sideways, {V.Len(toPad):0.0} m from the pad");
                Autopilot.SetStackPhase("LANDED");
            }
        }

        /// landing target: the pad, or the drone ship's deck centre
        static double3 TargetNowCci(Nav n) => Autopilot.Cfg.Droneship ? DroneShip.DeckCci(n) : PadNowCci(n);
        static double TargetRadius(Nav n) => Autopilot.Cfg.Droneship ? V.Len(DroneShip.DeckCci(n)) : Autopilot.PadRadius;

        static double _iipKm, _shipKm, _lastIip;
        static double _steerSign = 1, _lastSteerMiss = -1, _lastSteerCheck;
        /// downrange (great circle from the pad) of the drag-aware impact point and of the target
        static void UpdateDownrange(Nav n, double3 target, bool force = false)
        {
            if (!force && Autopilot.Now - _lastIip < 0.5) return;
            _lastIip = Autopilot.Now;
            var padNow = PadNowCci(n);
            var (imp, _) = Impact.PredictDrag(n, V.Len(target), Beta(n), Autopilot.Cfg.Droneship);
            double R = n.Body.MeanRadius;
            double Ang(double3 a, double3 b) => Math.Acos(Math.Clamp(V.Dot(V.Norm(a), V.Norm(b)), -1, 1));
            _iipKm = Ang(padNow, imp) * R / 1000; _shipKm = Ang(padNow, target) * R / 1000;
        }

        static double3 PadNowCci(Nav n)
        {
            if (n.Body is Celestial cel) return Double3Ex.Transform(Autopilot.PadCcf, cel.GetCcf2Cci());
            return Autopilot.PadCcf;
        }

        /// pad shifted along the return direction (beyond the pad, seen from the booster)
        static double3 AimPoint(Nav n, double3 pad, double beyond)
        {
            var back = V.Norm(n.Horizontal(V.Sub(pad, n.R)));
            var padUp = V.Norm(pad);
            back = V.Norm(V.Sub(back, V.Mul(padUp, V.Dot(back, padUp))));
            return V.Mul(V.Norm(V.Add(pad, V.Mul(back, beyond))), V.Len(pad));
        }

        static double3 BoostbackDir(Nav n, double3 pad)
        {
            var (impact, _) = Impact.Predict(n, Autopilot.PadRadius);
            var err = n.Horizontal(V.Sub(AimPoint(n, pad, Autopilot.Cfg.OvershootKm * 1000), impact));
            if (V.Len(err) < 1) err = n.Horizontal(V.Sub(pad, n.R));
            return V.Norm(V.Add(V.Norm(err), V.Mul(n.Up, 0.03)));
        }

        /// engines-first descent: retrograde tilted to move the impact point onto the pad (kOS coast logic:
        /// the body tilts opposite to the needed correction; the grid fins hold it there)
        const double BurnCarrySeconds = 3.6;   // aim past the target by sideways speed x this (landing-burn drag loss)
        const double LegDeployDelay = 1.5;     // s after landing-burn ignition
        const double LegClearance = 4.1;       // m from the engine bells down to the deployed leg pads
        static double _restSince = -1;
        const double MaxHoldApproach = 4.0;    // m/s; fastest sideways approach the low-altitude hold asks for

        static double3 AeroSteer(Nav n, double3 pad, double maxTiltDeg)
        {
            var retro = V.Mul(V.Norm(n.VSurf), -1.0);
            if (V.Len(n.VSurf) < 10) return n.Up;
            bool inAir = n.Q > 500;
            var (impact, tti) = inAir ? Impact.PredictDrag(n, TargetRadius(n), Beta(n), Autopilot.Cfg.Droneship) : Impact.Predict(n, TargetRadius(n));
            var aim = (inAir || Autopilot.Cfg.Droneship) ? pad : AimPoint(n, pad, Autopilot.Cfg.OvershootKm * 1000);
            var hdir = V.Norm(n.Horizontal(n.VSurf));
            // flight 5 (aim 0): lit 77 m short at 13.7 m/s sideways; burn drag killed that speed in ~5 s and it stalled
            // 45 m short, then hovered across. The burn travels ~hv^2/(2*3.5) instead of the ballistic ~hv*7 s, so aim
            // past the target by sideways speed x BurnCarrySeconds, eased in from 5 to 3 km.
            double shortM = 0;
            if (inAir && n.Alt < 5000)
            {
                // cap was 100 m: 2026-11-25 19:20 the booster came in at ~48 m/s sideways, wanted ~170 m, got 100 and
                // landed 132 m short. 250 m covers ~70 m/s.
                double carry = Math.Clamp(V.Len(n.Horizontal(n.VSurf)) * BurnCarrySeconds, 0, 250);
                shortM = -carry * Math.Clamp((5000 - n.Alt) / 2000.0, 0, 1);
                var padUp = V.Norm(aim);
                var back = V.Norm(V.Sub(hdir, V.Mul(padUp, V.Dot(hdir, padUp))));
                aim = V.Mul(V.Norm(V.Sub(aim, V.Mul(back, shortM))), V.Len(aim));
            }
            var corr = n.Horizontal(V.Sub(aim, impact));
            double m = V.Len(corr);
            double along = -V.Dot(corr, hdir), across = V.Len(V.Sub(corr, V.Mul(hdir, V.Dot(corr, hdir))));
            if (Autopilot.Cfg.Droneship && inAir && m > 150)
            {
                // the tilt -> lift sign is not guaranteed in every regime (flights 1-2: the miss grew below 20 km):
                // if the miss keeps growing while we steer, steer the other way
                if (_lastSteerMiss < 0 || Autopilot.Now - _lastSteerCheck > 4)
                {
                    // auto-reversal disabled: in flights 2 and 4 every reversal made the miss grow (it fired when the tilt
                    // was saturated, not when the sign was wrong); flights 1 and 3 never reversed and converged to ~6 m
                    if (_lastSteerMiss > 0 && m > _lastSteerMiss * 1.10 + 50)
                        Autopilot.Event($"aero miss growing ({_lastSteerMiss:0} -> {m:0} m), keeping steering sign");
                    _lastSteerMiss = m; _lastSteerCheck = Autopilot.Now;
                }
                corr = V.Mul(corr, _steerSign);
            }
            Autopilot.Telemetry += $"  predicted miss {m:0} m (beta {Beta(n):0} kg/m2)";
            if (Autopilot.Now - _lastAeroLog > 5)
            {
                _lastAeroLog = Autopilot.Now;
                Autopilot.Event($"aero: alt {n.Alt / 1000:0.0} km, {V.Len(n.VSurf):0} m/s, predicted miss {m:0} m ({along:+0;-0} along, {across:0} across), aim {-shortM:0} m beyond, impact in {tti:0} s, beta {Beta(n):0}");
            }
            if (m < 5 || n.Q < 500) { AeroDiag(n, retro, retro, corr, maxTiltDeg); return retro; }
            // engines-first body: tilting the nose AWAY from where the impact must move makes body + fin lift push it there
            double gain = n.Alt < 10000 ? 12.0 : 40.0;                       // metres of miss per degree of tilt
            // 2026-11-25 drone ship misses: below ~20 km a big miss asked for up to 15 deg at high dynamic pressure, the
            // booster could not hold it (18-31 deg off the wanted attitude at landing-burn start) and the miss grew by
            // ~1 km per 4 s. Allow less tilt the denser the air: full maxTilt up to 10 kPa, down to 4 deg at 60 kPa.
            double qCap = maxTiltDeg - (maxTiltDeg - 4.0) * Math.Clamp((n.Q - 10000) / 50000.0, 0, 1);
            double tiltDeg = Math.Min(m / gain, qCap);
            var want = V.Norm(V.Sub(retro, V.Mul(V.Norm(corr), Math.Tan(tiltDeg * Math.PI / 180))));
            AeroDiag(n, retro, want, corr, qCap);
            return want;
        }

        /// once a second below 45 km: wanted vs achieved tilt, and whether the measured aerodynamic side force pushes the
        /// impact point toward the target (lift toward > 0) - the data the 2026-11-25 drone ship misses did not have
        static void AeroDiag(Nav n, double3 retro, double3 want, double3 corr, double cap)
        {
            if (n.Alt > 45000 || Autopilot.Now - _lastAeroDiag < 1.0) return;
            _lastAeroDiag = Autopilot.Now;
            double Deg(double3 a, double3 b) => Math.Acos(Math.Clamp(V.Dot(V.Norm(a), V.Norm(b)), -1, 1)) * 180 / Math.PI;
            var air = V.Norm(n.VSurf);
            var cdir = V.Len(corr) > 1e-6 ? V.Norm(corr) : new double3(0, 0, 0);
            var noseOff = V.Sub(n.Nose, V.Mul(retro, V.Dot(n.Nose, retro)));          // achieved tilt, as a vector
            double tiltDir = V.Len(noseOff) > 1e-4 && V.Len(cdir) > 0 ? -V.Dot(V.Norm(noseOff), cdir) : 0;   // +1 = tilted away from corr, as wanted
            string lift = "n/a";
            if (_haveAeroAcc)
            {
                var side = V.Sub(_aeroAcc, V.Mul(air, V.Dot(_aeroAcc, air)));
                lift = $"{V.Len(side):0.0} m/s2, toward target {V.Dot(side, cdir):+0.0;-0.0}";
            }
            Autopilot.Event($"aero diag: alt {n.Alt / 1000:0.0} km, q {n.Q / 1000:0.0} kPa, tilt want {Deg(retro, want):0.0} (cap {cap:0.0}) got {Deg(retro, n.Nose):0.0} deg, " +
                            $"off-axis {n.AngleTo(want):0.0} deg, tilt dir {tiltDir:+0.00;-0.00}, side accel {lift}");
        }

        /// height of the engine bells above the ground (radar altitude of the CoM minus CoM-to-bell distance)
        static double BottomAltitude(Vehicle v, Nav n)
        {
            double comX = v.FlightComputer.CenterOfMassAsmb.X;
            double low = double.PositiveInfinity;
            foreach (var p in v.Parts.Parts)
                if (p.Template?.Id == Parts.Octaweb) low = Math.Min(low, p.PositionVehicleAsmb.X - 1.35);
            double h = Autopilot.Cfg.Droneship ? n.Rmag - V.Len(DroneShip.DeckCci(n)) : n.RadarAlt;
            if (double.IsInfinity(low)) return h;
            double cosTilt = Math.Max(0.2, Math.Abs(V.Dot(V.Norm(n.Nose), n.Up)));
            return h - (comX - low) * cosTilt;
        }

        static void TrySetControlled(Vehicle v)
        {
            try
            {
                var prop = typeof(Program).GetProperty("ControlledVehicle", BindingFlags.Public | BindingFlags.Static);
                if (prop?.CanWrite == true) prop.SetValue(null, v);
            }
            catch (Exception e) { Log.ErrorOnce("setcontrolled", e.Message); }
        }
    }

    // ====================================================================== second stage
    internal static class UpperGuidance
    {
        static bool _fairing;

        public static void Tick(Vehicle v, double dt)
        {
            var n = new Nav(v);
            double tPh = Autopilot.Now - Autopilot.UpperPhaseStart;
            var vIn = n.Vel;
            double vz = V.Dot(vIn, n.Up);
            var vh = V.Sub(vIn, V.Mul(n.Up, vz));
            double vhMag = V.Len(vh);
            var hdir = vhMag > 1 ? V.Mul(vh, 1.0 / vhMag) : n.Heading(Autopilot.Cfg.HeadingDeg, 0);
            double target = n.Body.MeanRadius + Autopilot.Cfg.OrbitKm * 1000;
            double vCirc = Math.Sqrt(n.Mu / n.Rmag);
            var o = v.Orbit;
            Autopilot.UpperBrief = $"alt {n.Alt / 1000:0.0} km, vh {vhMag:0} m/s, Ap {o.ApoapsisAltitude / 1000:0} / Pe {o.PeriapsisAltitude / 1000:0} km";
            Autopilot.UpperTelemetry = $"{Autopilot.UpperPhase}  alt {n.Alt / 1000:0.0} km  vh {vhMag:0} / {vCirc:0} m/s  vz {vz:0}  " +
                                       $"Ap {o.ApoapsisAltitude / 1000:0} km  Pe {o.PeriapsisAltitude / 1000:0} km  m {n.Mass / 1000:0.0} t";
            switch (Autopilot.UpperPhase)
            {
                case "-":
                    Autopilot.SetUpperPhase("COAST");
                    break;
                case "COAST":
                    Autopilot.SetEngine(v, false, 0f);
                    Autopilot.Point(v, V.Norm(vIn), n.Up, 3f);
                    if (tPh > 3.0)
                    {
                        Autopilot.ActivateEngines(v, Parts.VacEngine);
                        _fairing = false;
                        Autopilot.SetUpperPhase("SES-1");
                    }
                    break;
                case "SES-1":
                {
                    // closed loop: hold a climb rate that brings the stage to the target radius, put the rest of
                    // the thrust into horizontal speed; cut off at circular speed
                    double aT = Math.Max(0.1, ThrustAccel(v, n));
                    double vCircT = Math.Sqrt(n.Mu / target);
                    double ve = 348 * 9.80665;                                       // vacuum engine Isp
                    double dvGo = Math.Max(vCircT - vhMag, 0) + 0.5 * Math.Abs(vz);
                    double T = Math.Max(n.Mass / (aT * n.Mass / ve) * (1 - Math.Exp(-dvGo / ve)), 25.0);
                    // vertical accel profile a0 + a1 t that ends at the target radius with vz = 0 after T
                    double D = target - n.Rmag - vz * T;
                    double a1 = -12 * (vz * T / 2 + D) / (T * T * T);
                    double a0 = (-vz - a1 * T * T / 2) / T;
                    double az = a0 + n.G - vhMag * vhMag / n.Rmag;
                    double s = Math.Clamp(az / aT, -0.7, 0.85);
                    Autopilot.UpperTelemetry += $"  tgo {T:0} s";
                    if (tPh > 5) Webcast.Predict("SECO", n.Mass / (aT * n.Mass / ve) * (1 - Math.Exp(-dvGo / ve)));
                    if (!_fairing && n.Alt < 110000 && vz > 10) Webcast.Predict("FAIRING", (110000 - n.Alt) / vz);
                    var dir = V.Norm(V.Add(V.Mul(hdir, Math.Sqrt(1 - s * s)), V.Mul(n.Up, s)));
                    Autopilot.Point(v, dir, n.Up, 3f);
                    double dvLeft = vCirc - vhMag;
                    double peGap = Autopilot.Cfg.OrbitKm * 1000 - 8000 - o.PeriapsisAltitude;
                    float thr = peGap < 30000 ? (float)Math.Clamp(peGap / 30000.0, 0.4, 1.0) : 1f;
                    Autopilot.SetEngine(v, n.AngleTo(dir) < 30, thr);
                    if (!_fairing && n.Alt > 110000) { _fairing = true; Autopilot.FireDecouplers(v, Parts.Fairing); Autopilot.Event("fairing separation"); Webcast.OnPhase("FAIRING", upper: true); }
                    double tgtAlt = Autopilot.Cfg.OrbitKm * 1000;
                    if (o.PeriapsisAltitude > tgtAlt - 5000 || (dvLeft < 0.5 && Math.Abs(vz) < 15) ||
                        (o.ApoapsisAltitude > 3 * tgtAlt && o.PeriapsisAltitude > 120000))
                    {
                        Autopilot.SetEngine(v, false, 0f);
                        Autopilot.SetUpperPhase("SECO");
                    }
                    else if (tPh > 8 && v.Props.TotalPropellantMass < 30) { Autopilot.SetEngine(v, false, 0f); Autopilot.SetUpperPhase("SECO (out of propellant)"); }
                    break;
                }
                case "SECO":
                case "SECO (out of propellant)":
                    Autopilot.SetEngine(v, false, 0f);
                    Autopilot.Point(v, V.Norm(vIn), n.Up, 3f);
                    if (tPh > 3)
                    {
                        Autopilot.Event($"S2 orbit: Ap {o.ApoapsisAltitude / 1000:0.0} km, Pe {o.PeriapsisAltitude / 1000:0.0} km, " +
                                        $"{v.Props.TotalPropellantMass / 1000:0.0} t propellant left");
                        Autopilot.SetUpperPhase("ORBIT");
                    }
                    break;
                case "ORBIT":
                    Autopilot.Point(v, V.Norm(vIn), n.Up, 3f);
                    break;
            }
        }

        static double ThrustAccel(Vehicle v, Nav n)
        {
            var t = v.FlightComputer.ActiveEnginePerformanceMax.Thrust;
            if (t <= 1f) t = 981e3f;
            return t / Math.Max(n.Mass, 1000);
        }
    }

    // ====================================================================== impact prediction (vacuum, rotating planet)
    internal static class Impact
    {
        /// ballistic impact point (CCI, rotated into the planet's current orientation) and time to impact
        public static (double3 Point, double Time) Predict(Nav n, double groundRadius)
        {
            var r = n.R; var vel = n.Vel;
            double mu = n.Mu, t = 0, step = 1.0;
            for (int i = 0; i < 3000; i++)
            {
                var a1 = Grav(r, mu);
                var rm = V.Add(r, V.Add(V.Mul(vel, step * 0.5), V.Mul(a1, step * step / 8)));
                var a2 = Grav(rm, mu);
                var rn = V.Add(r, V.Add(V.Mul(vel, step), V.Mul(a2, step * step * 0.5)));
                var vn = V.Add(vel, V.Mul(a2, step));
                if (V.Len(rn) <= groundRadius)
                {
                    double r0 = V.Len(r), r1 = V.Len(rn);
                    double f = Math.Clamp((r0 - groundRadius) / Math.Max(r0 - r1, 1e-6), 0, 1);
                    r = V.Add(r, V.Mul(V.Sub(rn, r), f)); t += step * f;
                    break;
                }
                r = rn; vel = vn; t += step;
                if (t > 1500) break;
            }
            // the planet turns under the falling booster: express the impact in today's orientation
            var w = n.Body.GetAngularVelocityCci();
            double wm = V.Len(w);
            if (wm > 0)
            {
                var q = doubleQuat.CreateFromAxisAngle(V.Mul(w, 1.0 / wm), -wm * t);
                r = Double3Ex.Transform(r, q);
            }
            return (r, t);
        }

        /// impact with drag: exponential atmosphere anchored on the current density (from the game's q), drag on the
        /// air-relative velocity (planet rotation), measured ballistic coefficient
        public static (double3 Point, double Time) PredictDrag(Nav n, double groundRadius, double beta, bool realAtmosphere = false)
        {
            var atm = realAtmosphere ? n.Body.GetPhysicalAtmosphere() : null;
            double Rm = n.Body.MeanRadius;
            var w = n.Body.GetAngularVelocityCci();
            double v0 = Math.Max(V.Len(n.VSurf), 1);
            double rho0 = 2 * n.Q / (v0 * v0), h0 = n.Rmag;
            const double H = 7200;
            var r = n.R; var vel = n.Vel;
            double mu = n.Mu, t = 0, step = 0.25;
            double3 Acc(double3 rr, double3 vv)
            {
                double rm = V.Len(rr);
                var a = V.Mul(rr, -mu / (rm * rm * rm));
                var vAir = V.Sub(vv, V.Cross(w, rr));
                double va = V.Len(vAir);
                double rho;
                if (atm != null) { double alt = rm - Rm; rho = alt < atm.Height ? atm.ComputeConditionsAtAltitude(Math.Max(alt, 0)).Density : 0; }
                else rho = rho0 * Math.Exp(-(rm - h0) / H);
                return va > 0 ? V.Add(a, V.Mul(vAir, -0.5 * rho * va / beta)) : a;
            }
            for (int i = 0; i < 6000; i++)
            {
                var a1 = Acc(r, vel);
                var vm = V.Add(vel, V.Mul(a1, step * 0.5));
                var rm = V.Add(r, V.Mul(vel, step * 0.5));
                var a2 = Acc(rm, vm);
                var rn = V.Add(r, V.Mul(vm, step));
                var vn = V.Add(vel, V.Mul(a2, step));
                if (V.Len(rn) <= groundRadius)
                {
                    double r0 = V.Len(r), r1 = V.Len(rn);
                    double f = Math.Clamp((r0 - groundRadius) / Math.Max(r0 - r1, 1e-6), 0, 1);
                    r = V.Add(r, V.Mul(V.Sub(rn, r), f)); t += step * f;
                    break;
                }
                r = rn; vel = vn; t += step;
                if (t > 900) break;
            }
            double wm = V.Len(w);
            if (wm > 0) r = Double3Ex.Transform(r, doubleQuat.CreateFromAxisAngle(V.Mul(w, 1.0 / wm), -wm * t));
            return (r, t);
        }

        static double3 Grav(double3 r, double mu)
        {
            double m = V.Len(r);
            return V.Mul(r, -mu / (m * m * m));
        }
    }

    // ====================================================================== hooks
    [HarmonyPatch(typeof(Vehicle), nameof(Vehicle.PrepareWorker))]
    internal static class AutopilotTick
    {
        [HarmonyPrefix]
        static void Prefix(Vehicle __instance)
        {
            try { Autopilot.Tick(__instance); }
            catch (Exception e) { Log.ErrorOnce("autopilot", e.ToString()); }
        }
    }

    [HarmonyPatch(typeof(InputEvents), nameof(InputEvents.ApplyInputEvents))]
    internal static class AutopilotSaveLoad
    {
        [HarmonyPostfix]
        static void Postfix()
        {
            try
            {
                DroneShip.ApplyPending();
                if (Autopilot.WantSave)
                {
                    Autopilot.WantSave = false;          // LAUNCH pressed (no automatic pad save any more: the player saves when they want)
                    var v = Program.ControlledVehicle;
                    if (v != null) Autopilot.Start(v);
                }
                if (Autopilot.WantLoad)
                {
                    Autopilot.WantLoad = false;
                    Autopilot.Active = false;
                    Autopilot.StackPhase = "RELOADING";
                    DroneShip.Ship = null;
                    GameSaves.LoadSaveGame(Autopilot.SaveName);
                    Autopilot.OnLoaded();
                    Autopilot.AutoStartAfterLoad = true;
                    Log.Info("mission: reloaded " + Autopilot.SaveName);
                }
            }
            catch (Exception e) { Log.Error("mission save/load: " + e); }
        }
    }

    [HarmonyPatch(typeof(Program), "OnDrawUiFrame")]
    internal static class AutopilotWindow
    {
        internal static bool Open = true;   // also toggled from the Sterling Orbital menu (SterlingMenu.cs)

        [HarmonyPostfix]
        static void Postfix()
        {
            try
            {
                var v = Program.ControlledVehicle;
                var io = ImGui.GetIO();
                if (io.KeyCtrl && io.KeyShift && ImGui.IsKeyPressed(ImGuiKey.F, repeat: false)) Open = !Open;
                bool sparrow = Autopilot.Active || Autopilot.AwaitingRecovery || (v != null && Autopilot.HasPart(v, Parts.Octaweb));
                if (!sparrow || !Open) return;
                MissionPanel.Draw(v, ref Open);
            }
            catch (Exception e) { Log.ErrorOnce("apwindow", e.ToString()); }
        }
    }
}
