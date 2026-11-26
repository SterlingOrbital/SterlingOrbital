// Landing leg suspension for "SparrowLandingLegPart".
//
// KSA has no suspension: every contact uses one global Bepu spring. So each deployed leg gets a spring-damper
// at its crush pad, acting along the local vertical, with tweakable stiffness, compression damping, rebound
// damping and travel (right-click the leg). The part's foot-stop collider (fully compressed position) is the
// hard stop underneath; the spring works over the "travel" above it, and the leg visibly extends while
// airborne and folds up as the spring compresses.
//   * VehicleUpdateState.UpdateActiveChutes postfix: gather deployed legs (pad point in the body frame).
//   * PhysicsStates.ComputeDerivatives postfix: pad height above the terrain (vehicle radius + pad offset along
//     the local up - PhysicsEnvironment.TerrainRadius), spring/damper + ground friction via AddForceBody.
//   * KeyframeAnimationModule ApplyAnimationTransforms postfix: visual leg extension/compression, strut follows.
//   * Part.ShowContextMenu postfix: sliders; settings saved to landing_legs.json in the mod folder.
using System.Collections.Concurrent;
using System.Reflection;
using System.Runtime.CompilerServices;
using System.Text.Json;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace SterlingFlight
{
    internal static class LegGeo
    {
        public const string ModuleId = "SparrowLandingLegDeploy";
        // KSA part space (from the landing leg model and animation, v3 2026-10-11:
        // hinge = the octaweb lug pin line, pad = the crush-pad hub on the tip cap)
        public static readonly double3 Hinge = new(0.0, 0.0, 0.303);
        public static readonly double3 StrutPivot = new(2.729, 0.0, -0.14);
        public static readonly double3 StrutLegPt = new(8.689, 0.0, -0.14);
        public static readonly double3 PadStowed = new(8.613, 0.0, -0.4242);
        public const double Deploy = 132.0 * Math.PI / 180.0;
        public const double DeploySeconds = 4.2;

        public static double3 RotY(double3 v, double a) =>
            new(v.X * Math.Cos(a) + v.Z * Math.Sin(a), v.Y, -v.X * Math.Sin(a) + v.Z * Math.Cos(a));

        public static double3 PadAt(double phi) => V.Add(Hinge, RotY(V.Sub(PadStowed, Hinge), phi));
        public static double Alpha(double3 v) => Math.Atan2(-v.Z, v.X);
        public static doubleQuat QY(double a) => doubleQuat.CreateFromAxisAngle(new double3(0, 1, 0), a);

        /// extra leg rotation that lowers the pad (along part -X) by e metres
        public static double ExtensionAngle(double e)
        {
            if (e <= 0) return 0;
            double d0 = -PadAt(Deploy).X, lo = 0, hi = 20 * Math.PI / 180;
            for (int i = 0; i < 30; i++)
            {
                var mid = (lo + hi) / 2;
                if (-PadAt(Deploy + mid).X - d0 < e) lo = mid; else hi = mid;
            }
            return (lo + hi) / 2;
        }
    }

    internal sealed class LegSettings
    {
        // v3 (2026-11-24): mass-relative. Spring rate comes from the vehicle's current mass, local gravity and the
        // number of deployed legs so the static sag is always SagFraction of the travel (empty or fuelled booster);
        // damping is a ratio of critical. v2 used fixed rates tuned for a 27 t empty booster and bottomed out on
        // heavier ones, leaving the vehicle on the rigid foot-stop colliders (jitter, contact-pressure failures).
        public int Version { get; set; } = 3;                 // settings saved by an older build are dropped
        public float SagFraction { get; set; } = 0.35f;       // static compression / travel
        public float CompressionRatio { get; set; } = 0.45f;  // damping ratio while compressing
        public float ReboundRatio { get; set; } = 0.9f;       // damping ratio while extending (kills the bounce)
        public float TravelM { get; set; } = 1.0f;
        public const double Progression = 8.0;                // rate x (1 + 8 (x/travel)^4): bump stop at the end
        public LegSettings Clone() => (LegSettings)MemberwiseClone();
    }

    internal static class LegConfig
    {
        static string PathCfg => ModPaths.File("landing_legs.json");
        static readonly ConcurrentDictionary<string, LegSettings> ByPart = new();
        public static LegSettings Defaults = new();
        static bool _loaded;

        public static LegSettings For(string partId)
        {
            Load();
            return ByPart.GetOrAdd(partId ?? "", _ => Defaults.Clone());
        }

        public static void Set(string partId, LegSettings s) { ByPart[partId] = s; Defaults = s.Clone(); Save(); }

        static void Load()
        {
            if (_loaded) return;
            _loaded = true;
            try
            {
                if (!System.IO.File.Exists(PathCfg)) return;
                var d = JsonSerializer.Deserialize<Dictionary<string, LegSettings>>(System.IO.File.ReadAllText(PathCfg));
                if (d == null) return;
                foreach (var kv in d)
                {
                    if (kv.Value.Version < new LegSettings().Version) continue;     // tuned for the old leg
                    if (kv.Key == "_defaults") Defaults = kv.Value; else ByPart[kv.Key] = kv.Value;
                }
            }
            catch (Exception e) { Log.ErrorOnce("legcfg", e.ToString()); }
        }

        static void Save()
        {
            try
            {
                var d = new Dictionary<string, LegSettings>(ByPart) { ["_defaults"] = Defaults };
                System.IO.File.WriteAllText(PathCfg, JsonSerializer.Serialize(d, new JsonSerializerOptions { WriteIndented = true }));
            }
            catch (Exception e) { Log.ErrorOnce("legsave", e.ToString()); }
        }
    }

    internal sealed class Leg
    {
        public ulong Key;
        public string PartId = "";
        public KeyframeAnimationModule? Module;
        public LegSettings Settings = new();
        public double3 Pad;                 // pad contact point, body frame (origin CoM), fully compressed pose
        public bool Settled;                // fully deployed and animation finished
        public long GoalSince;              // ms when TimeGoal went > 0
        public double Height = double.PositiveInfinity;   // pad height above ground (from physics)
        public double LoadKN;               // last spring force
        public double VisualExt;            // current visual extension angle (rad)
        public bool Shown;
        // diagnostics (written by the physics step, logged by GatherLegs)
        public bool Engaged;                // spring in contact this touchdown
        public double TouchSpeed;           // pad sink speed when the spring first engaged (m/s)
        public double MaxComp, PeakKN;      // this touchdown
        public bool Bottomed;               // compression reached the full travel (foot stop takes over)
        public double SinkSpeed;            // latest pad sink speed (m/s, + = down)
        public volatile bool NewTouch;      // spring just engaged -> log right away
        public bool RayGround;              // height came from the physics ray (false: heightmap fallback)
        public int SettledFrames;           // visual frames since fully deployed (collider rebuild gate)
    }

    internal static class ContactMath
    {
        /// inverse effective mass of the rigid vehicle for a force along t applied at r (body frame, origin CoM)
        public static double InvMass(in VehicleProperties props, double3 r, double3 t)
        {
            var rt = V.Cross(r, t);
            ref readonly var invI = ref props.TotalMassPropsInvAsmb.Props.Inertia;
            return props.TotalMassInv + V.Dot(rt, rt.Transform(in invI));
        }

        /// KSA re-creates a part's Bepu colliders every time its animation is re-applied; once a leg/fin is settled
        /// the collider-bearing subparts no longer move, so drop that request (it reset every contact each frame)
        public static void KeepColliders(Part part)
        {
            var cols = part.FullPart.SubtreeModules.Get<ColliderModule>();
            for (int i = 0; i < cols.Length; i++) cols[i].NeedsColliderUpdate = false;
        }
    }

    internal static class LegRegistry
    {
        public static readonly ConcurrentDictionary<VehicleUpdateState, Leg[]> Sets = new();
        public static volatile VehicleUpdateState[] Snapshot = Array.Empty<VehicleUpdateState>();
        public static readonly ConcurrentDictionary<ulong, Leg> Legs = new();
        public static void Publish() => Snapshot = Sets.Keys.ToArray();
        public static long PhysicsHits;     // physics steps that found a leg set (health check)
    }

    internal static class LegDiag
    {
        static long _lastLog, _lastHits, _lastHitCheck;
        static bool _warnedNoPhysics;

        /// once per vehicle update: approach / touchdown / health lines in SterlingFlight.log (rate-limited)
        public static void Report(VehicleUpdateState vs, Leg[] legs)
        {
            long now = Environment.TickCount64;
            bool anySettled = false, anyEngaged = false; double minH = double.PositiveInfinity;
            foreach (var l in legs)
            {
                if (!l.Settled) continue;
                anySettled = true; anyEngaged |= l.Engaged;
                if (!double.IsInfinity(l.Height)) minH = Math.Min(minH, l.Height);
            }
            if (anySettled && now - _lastHitCheck > 3000)
            {
                long hits = Interlocked.Read(ref LegRegistry.PhysicsHits);
                if (hits == _lastHits && !_warnedNoPhysics)
                {
                    _warnedNoPhysics = true;
                    Log.Error($"vehicle {vs.Id}: legs deployed but the physics step never matched this vehicle - suspension inactive");
                }
                _lastHits = hits; _lastHitCheck = now;
            }
            bool newTouch = false;
            foreach (var l in legs) if (l.NewTouch) { newTouch = true; l.NewTouch = false; }
            if (!anySettled || (!newTouch && now - _lastLog < (anyEngaged ? 100 : 500))) return;
            if (!anyEngaged && minH > 8.0) return;
            _lastLog = now;
            var parts = legs.Where(l => l.Settled).Select(l =>
                l.Engaged ? $"now {Math.Clamp(l.Settings.TravelM - l.Height, 0, l.Settings.TravelM):0.00} m {l.LoadKN:0} kN, max {l.MaxComp:0.00}/{l.Settings.TravelM:0.00} m peak {l.PeakKN:0} kN touch {l.TouchSpeed:0.0} m/s{(l.Bottomed ? " BOTTOMED" : "")}"
                          : $"h {l.Height:0.00} m{(l.RayGround ? "" : " (map)")} sink {l.SinkSpeed:0.0} m/s");
            Log.Info($"legs {vs.Id}: " + string.Join(" | ", parts));
        }
    }

    // logs any part whose contact pressure gets near what KSA (5554+) breaks it at
    [HarmonyPatch(typeof(PartFailure), nameof(PartFailure.Detect))]
    internal static class ContactPressureLog
    {
        static readonly ConcurrentDictionary<string, long> Last = new();

        [HarmonyPrefix]
        static void Prefix(VehicleUpdateState vehicleState)
        {
            try
            {
                ref var h = ref vehicleState.BepuHandles;
                var parts = h.ContactParts; var acc = h.ContactAccumulators;
                long now = Environment.TickCount64;
                for (int i = 0; i < parts.Length && i < acc.Length; i++)
                {
                    var p = parts[i]; if (p == null) continue;
                    double pr = PartStructuralLimits.AccumulatedPressure(acc[i]), tol = p.CrashTolerancePascals;
                    if (tol <= 0 || pr < 0.3 * tol) continue;
                    var key = p.Template?.Id + "#" + i;
                    if (Last.TryGetValue(key, out var t) && now - t < 500) continue;
                    Last[key] = now;
                    Log.Info($"contact pressure {p.Template?.Id}: {pr / 1e6:0.0} MPa of {tol / 1e6:0.0} MPa{(pr >= tol ? "  -> PART FAILS" : "")}");
                }
            }
            catch (Exception e) { Log.ErrorOnce("contactlog", e.ToString()); }
        }
    }

    [HarmonyPatch(typeof(VehicleUpdateState), nameof(VehicleUpdateState.UpdateActiveChutes))]
    internal static class GatherLegs
    {
        [HarmonyPostfix]
        static void Postfix(VehicleUpdateState __instance)
        {
            try
            {
                var vehicle = __instance.ReadOnlyVehicle;
                if (vehicle == null) return;
                var mods = vehicle.Parts.Modules.Get<KeyframeAnimationModule>();
                List<Leg>? legs = null;
                // simulation time (ms), not wall clock: under time warp the wall-clock settle came after touchdown
                // (flight 6: landed with the suspension still off, then it switched on fully compressed and launched it)
                var now = (long)(Universe.GetElapsedTime().Seconds() * 1000.0);
                var com = __instance.GetReadOnlyProps().CenterOfMassAsmb;
                var comD = new double3(com.X, com.Y, com.Z);
                for (int i = 0; i < mods.Length; i++)
                {
                    var m = mods[i];
                    if (m.TemplateId != LegGeo.ModuleId || m.Parent == null) continue;
                    var leg = LegRegistry.Legs.GetOrAdd(m.InstanceId, id => new Leg { Key = id });
                    leg.Module = m;
                    leg.PartId = m.Parent.Id ?? "";
                    leg.Settings = LegConfig.For(leg.PartId);
                    if (m.TimeGoal > 0f) { if (leg.GoalSince == 0) leg.GoalSince = now; }
                    else leg.GoalSince = 0;
                    leg.Settled = leg.GoalSince != 0 && now - leg.GoalSince > LegGeo.DeploySeconds * 1000;
                    leg.Pad = V.Sub(Double3Ex.Transform(LegGeo.PadAt(LegGeo.Deploy), m.Parent.MatrixAsmb2VehicleAsmb), comD);
                    if (!leg.Settled) { leg.Height = double.PositiveInfinity; leg.LoadKN = 0; }
                    (legs ??= new()).Add(leg);
                }
                if (legs == null)
                {
                    if (LegRegistry.Sets.TryRemove(__instance, out _)) LegRegistry.Publish();
                    return;
                }
                bool isNew = !LegRegistry.Sets.ContainsKey(__instance);
                var arr = legs.ToArray();
                LegRegistry.Sets[__instance] = arr;
                if (isNew) { LegRegistry.Publish(); Log.Info($"vehicle {__instance.Id}: {legs.Count} landing leg(s)"); }
                LegDiag.Report(__instance, arr);
            }
            catch (Exception e) { Log.ErrorOnce("gatherlegs", e.ToString()); }
        }
    }

    [HarmonyPatch]
    internal static class LegSuspension
    {
        static MethodBase TargetMethod() =>
            typeof(PhysicsStates).GetMethods(BindingFlags.Public | BindingFlags.Static)
                .First(m => m.Name == nameof(PhysicsStates.ComputeDerivatives) && m.GetParameters().Length == 11);

        static bool _loggedDt;

        [HarmonyPostfix]
        static void Postfix(ref BubbleOrigin origin, ref KinematicStates kinematic, ref VehicleProperties props,
                            ref PhysicsEnvironment environment, double dt, PartTree? parts, ref Disturbances __result)
        {
            try
            {
                if (!environment.InPhysicsRadius || environment.TerrainRadius <= 0) return;
                var snap = LegRegistry.Snapshot;
                Leg[]? legs = null;
                VehicleUpdateState? vs = null;
                for (int i = 0; i < snap.Length; i++)
                    if ((parts != null && ReferenceEquals(snap[i].ReadOnlyVehicle?.Parts, parts)) || Unsafe.AreSame(ref kinematic, ref snap[i].Kinematic))
                    { vs = snap[i]; LegRegistry.Sets.TryGetValue(snap[i], out legs); break; }
                if (legs == null) return;
                Interlocked.Increment(ref LegRegistry.PhysicsHits);

                // mass share per deployed leg and the explicit-integration limit for velocity-proportional forces
                int nLegs = 0;
                foreach (var l in legs) if (l.Settled) nLegs++;
                if (nLegs == 0) return;
                double mLeg = Math.Max(props.TotalMass, 1.0f) / nLegs;
                double g = Math.Max(V.Len(environment.GravitationBub), 0.1);
                double step = Math.Max(dt, 1e-4);
                double cLimitMass = 0.8 * mLeg / step;             // c*dt/m above ~1 overshoots every step = jitter
                if (!_loggedDt) { _loggedDt = true; Log.Info($"leg suspension: physics dt {dt * 1000:0.0} ms, {props.TotalMass / 1000:0.0} t on {nLegs} legs, g {g:0.00}"); }

                // local vertical and height of the CoM above the planet centre
                var posBub = PhysicsStates.GetPositionBub(in origin, in kinematic);
                var rel = PhysicsStates.GetPositionClosestParentBub(in origin, in environment, posBub);
                var r = V.Len(rel);
                if (r < 1) return;
                var inv = QuaternionEx.Inverse(kinematic.Body2Phys);
                var up = V.Norm(Double3Ex.Transform(V.Mul(rel, 1.0 / r), inv));          // body frame
                var ground = Math.Max(environment.TerrainRadius, environment.OceanRadius);
                var air = PhysicsStates.ComputeAirVelocityBody(in origin, in kinematic, in environment);  // vs. the surface
                var vcm = new double3(air.X, air.Y, air.Z);
                var w = PhysicsStates.GetBodyRates(in origin, in kinematic, in environment);

                // terrain under each pad (the pads sit ~6 m out from the axis: on a slope or a tilted touchdown the
                // height under the CoM was off by a metre or more, so a leg met the ground on its rigid foot stop)
                var cel = environment.ClosestParent as Celestial;
                bool ccf = origin.BubFrame.IsCcf();

                foreach (var leg in legs)
                {
                    if (!leg.Settled) continue;
                    var s = leg.Settings;
                    double h;                                      // pad (compressed position) height above ground
                    // 1st choice: ray-cast the real physics ground (terrain blocks, pads, structures, other vehicles)
                    // straight down from the pad; the heightmap missed launch/landing pads by ~1.75 m
                    var padPhys = V.Add(kinematic.PositionPhys, Double3Ex.Transform(leg.Pad, kinematic.Body2Phys));
                    var downPhys = V.Mul(rel, -1.0 / r);
                    var hit = GroundRay.Cast(vs, padPhys, downPhys, s.TravelM + 6.0);
                    leg.RayGround = hit.HasValue;
                    if (hit.HasValue) h = hit.Value;
                    else if (cel != null)
                    {
                        var padBub = V.Add(rel, Double3Ex.Transform(leg.Pad, kinematic.Body2Phys));
                        var rp = V.Len(padBub);
                        var dir = V.Mul(padBub, 1.0 / rp);
                        var th = ccf ? cel.GetTerrainHeightFromDirCcf(dir) : cel.GetTerrainHeightFromDirCci(dir);
                        h = rp - Math.Max(cel.MeanRadius + th, environment.OceanRadius);
                    }
                    else h = r + V.Dot(leg.Pad, up) - ground;
                    leg.Height = h;
                    double travel = s.TravelM;
                    var x = travel - h;                            // spring compression
                    var vfoot = V.Add(vcm, V.Cross(w, leg.Pad));
                    var hdot = V.Dot(vfoot, up);
                    leg.SinkSpeed = -hdot;
                    if (x <= 0)
                    {
                        leg.LoadKN = 0;
                        if (leg.Engaged && h > travel + 0.5) { leg.Engaged = false; leg.Bottomed = false; }   // lifted off
                        continue;
                    }
                    if (!leg.Engaged) { leg.Engaged = true; leg.NewTouch = true; leg.TouchSpeed = -hdot; leg.MaxComp = 0; leg.PeakKN = 0; leg.Bottomed = false; }
                    if (x >= travel) leg.Bottomed = true;
                    x = Math.Min(x, travel);
                    double sagQ = Math.Clamp(s.SagFraction, 0.05, 0.9);
                    double k = mLeg * g / (sagQ * travel * (1.0 + LegSettings.Progression * Math.Pow(sagQ, 4)));
                    double wn = Math.Sqrt(k / mLeg);
                    // damping limit from the vehicle's real response at this pad (mass AND rotation), shared by the legs
                    double cLimit = Math.Min(cLimitMass, 0.8 / (step * ContactMath.InvMass(in props, leg.Pad, up) * nLegs));
                    var c = Math.Min(2.0 * (hdot < 0 ? s.CompressionRatio : s.ReboundRatio) * mLeg * wn, cLimit);
                    if (hdot < 0) c *= Math.Min(1.0, x / 0.04);       // ramp the damper in over 4 cm: no slam at first contact
                    var q = x / travel;
                    var fs = k * x * (1.0 + LegSettings.Progression * q * q * q * q);
                    var fn = Math.Max(0.0, fs - c * hdot);
                    // big physics steps (time warp: 100 ms) make the stiff progressive spring explode (flight 6: 5585 kN on
                    // one bottomed leg, 55 g). Never push harder than stopping this leg's share of the sink in one step
                    // plus 2.5 g, so a step can not turn compression into a launch.
                    double mEffUp = 1.0 / (ContactMath.InvMass(in props, leg.Pad, up) * nLegs);
                    fn = Math.Min(fn, mEffUp * Math.Max(-hdot, 0.0) / step + 2.5 * mLeg * g);
                    leg.LoadKN = fn / 1000.0;
                    leg.MaxComp = Math.Max(leg.MaxComp, x); leg.PeakKN = Math.Max(leg.PeakKN, leg.LoadKN);
                    // ground friction: stop the foot sliding (viscous, capped by mu * normal force)
                    var vlat = V.Sub(vfoot, V.Mul(up, hdot));
                    var vl = V.Len(vlat);
                    // remove at most half the foot's slip per step (stable at any dt), capped by mu * normal force
                    // the pads sit ~9 m off the axis: for spin the vehicle responds far more than its mass suggests, so
                    // the slip budget uses the effective mass (mass + yaw/pitch inertia) at the pad, split over the legs
                    double3 flat = new(0, 0, 0);
                    if (vl > 1e-4)
                    {
                        var tdir = V.Mul(vlat, 1.0 / vl);
                        var mEff = 1.0 / (ContactMath.InvMass(in props, leg.Pad, tdir) * nLegs);
                        flat = V.Mul(tdir, -Math.Min(0.9 * fn, 0.4 * mEff * vl / step));
                    }
                    __result.AddForceBody(leg.Pad, V.Add(V.Mul(up, fn), flat));
                }
            }
            catch (Exception e) { Log.ErrorOnce("suspension", e.ToString()); }
        }
    }

    // ---------------------------------------------------------------- visuals: leg extends/compresses, strut follows
    [HarmonyPatch(typeof(KeyframeAnimationModule), nameof(KeyframeAnimationModule.UpdateModules))]
    internal static class KeepLegAnimationLive
    {
        static readonly AccessTools.FieldRef<KeyframeAnimationModule, bool> Dirty =
            AccessTools.FieldRefAccess<KeyframeAnimationModule, bool>("_transformsDirty");

        // No longer forces the deploy animation to re-run every step (KSA re-applied the raw pose on a worker
        // thread before our pose, and the renderer caught it in between = the legs/struts flickering). The settled
        // pose is set directly on the main thread instead (PoseSync).
        [HarmonyPrefix]
        static void Prefix() { }
    }

    [HarmonyPatch(typeof(KeyframeAnimationModule), nameof(KeyframeAnimationModule.ApplyAnimationTransforms))]
    internal static class LegVisual
    {
        [HarmonyPostfix]
        static void Postfix(KeyframeAnimationModule __instance)
        {
            if (LegRegistry.Legs.TryGetValue(__instance.InstanceId, out var leg)) Apply(__instance, leg, true);
        }

        /// fromAnim: called right after KSA applied the animation (worker thread); otherwise the main-thread sync
        internal static void Apply(KeyframeAnimationModule __instance, Leg leg, bool fromAnim)
        {
            try
            {
                var part = __instance.Parent;
                if (part == null) return;
                if (!fromAnim && !leg.Settled) return;
                // only once fully deployed, and from the known deployed angle: reading the angle back from the subpart
                // fed our own previous output back in on frames where KSA did not re-sample the animation, so the
                // extension stacked up and the leg drifted off its strut until the animation snapped it back.
                if (!leg.Settled)
                {
                    leg.VisualExt = 0; leg.Shown = false; leg.SettledFrames = 0;
                    if (__instance.TimeGoal > 0f) ContactMath.KeepColliders(part);   // deploying: no per-frame rebuilds
                    return;
                }
                // fully out: rebuild the colliders exactly once in the deployed pose, then leave them alone
                if (leg.SettledFrames++ == 0)
                {
                    var cols = part.FullPart.SubtreeModules.Get<ColliderModule>();
                    for (int i = 0; i < cols.Length; i++) cols[i].NeedsColliderUpdate = true;
                }
                else if (fromAnim) ContactMath.KeepColliders(part);
                // airborne: pad lowered by the full travel; on the ground: pad sits on the ground
                var e = double.IsInfinity(leg.Height) ? leg.Settings.TravelM : Math.Clamp(leg.Height, 0.0, leg.Settings.TravelM);
                double target = LegGeo.ExtensionAngle(e);
                double dExt = target - leg.VisualExt;
                leg.VisualExt += dExt > 0 ? Math.Min(dExt, 0.002) : Math.Max(dExt, -0.03);   // extend gently, compress fast
                leg.Shown = true;

                Part? legSp = null, cylSp = null, rodSp = null;
                var subs = part.SubParts;
                for (int i = 0; i < subs.Length; i++)
                {
                    var id = subs[i].Id ?? "";
                    if (id.Contains("Leg1", StringComparison.Ordinal)) legSp = subs[i];
                    else if (id.Contains("Cyl1", StringComparison.Ordinal)) cylSp = subs[i];
                    else if (id.Contains("Rod1", StringComparison.Ordinal)) rodSp = subs[i];
                }
                if (legSp == null) return;
                var phi = LegGeo.Deploy + leg.VisualExt;
                var Rl = LegGeo.QY(phi);
                legSp.Asmb2ParentAsmb = Rl;
                legSp.PositionParentAsmb = V.Sub(LegGeo.Hinge, Double3Ex.Transform(LegGeo.Hinge, Rl));
                var P = V.Add(LegGeo.Hinge, LegGeo.RotY(V.Sub(LegGeo.StrutLegPt, LegGeo.Hinge), phi));
                var th = LegGeo.Alpha(V.Sub(P, LegGeo.StrutPivot)) - LegGeo.Alpha(V.Sub(LegGeo.StrutLegPt, LegGeo.StrutPivot));
                var Rc = LegGeo.QY(th);
                if (cylSp != null)
                {
                    cylSp.Asmb2ParentAsmb = Rc;
                    cylSp.PositionParentAsmb = V.Sub(LegGeo.StrutPivot, Double3Ex.Transform(LegGeo.StrutPivot, Rc));
                }
                if (rodSp != null)
                {
                    rodSp.Asmb2ParentAsmb = Rc;
                    rodSp.PositionParentAsmb = V.Sub(P, Double3Ex.Transform(LegGeo.StrutLegPt, Rc));
                }
            }
            catch (Exception e) { Log.ErrorOnce("legvisual", e.ToString()); }
        }
    }

    // ---------------------------------------------------------------- main-thread pose sync (legs + grid fins)
    [HarmonyPatch(typeof(Vehicle), nameof(Vehicle.PrepareWorker))]
    internal static class PoseSync
    {
        [HarmonyPostfix]
        static void Postfix(Vehicle __instance)
        {
            try
            {
                var tree = __instance.Parts;
                foreach (var leg in LegRegistry.Legs.Values)
                    if (leg.Settled && leg.Module?.Parent?.FullPart?.Tree == tree) LegVisual.Apply(leg.Module, leg, false);
                foreach (var fin in Registry.FinObjs.Values)
                    if (fin.Module?.Parent?.FullPart?.Tree == tree) DeflectFinVisual.Apply(fin);
            }
            catch (Exception e) { Log.ErrorOnce("posesync", e.ToString()); }
        }
    }

    // ---------------------------------------------------------------- right-click tweakables
    [HarmonyPatch(typeof(Part), nameof(Part.ShowContextMenu))]
    internal static class LegMenu
    {
        [HarmonyPostfix]
        static void Postfix(Part __instance, Vehicle vehicle)
        {
            try
            {
                var mods = __instance.Modules.Get<KeyframeAnimationModule>();
                KeyframeAnimationModule? mine = null;
                for (int i = 0; i < mods.Length; i++) if (mods[i].TemplateId == LegGeo.ModuleId) mine = mods[i];
                if (mine == null) return;
                var s = LegConfig.For(__instance.Id ?? "").ToEdit();
                ImGui.PushID("SparrowLegSusp" + __instance.Id);
                ImGui.SeparatorText("Landing leg suspension");
                bool ch = false;
                ch |= ImGui.SliderFloat("Sag (fraction of travel)", ref s.Sag_, 0.1f, 0.7f, "%.2f", ImGuiSliderFlags.None);
                ch |= ImGui.SliderFloat("Compression damping (ratio)", ref s.Comp_, 0f, 1.5f, "%.2f", ImGuiSliderFlags.None);
                ch |= ImGui.SliderFloat("Rebound damping (ratio)", ref s.Reb_, 0f, 2.0f, "%.2f", ImGuiSliderFlags.None);
                ch |= ImGui.SliderFloat("Travel (m)", ref s.TravelM_, 0.2f, 2.0f, "%.2f", ImGuiSliderFlags.None);
                if (LegRegistry.Legs.TryGetValue(mine.InstanceId, out var leg) && leg.Settled && !double.IsInfinity(leg.Height))
                {
                    var comp = Math.Clamp(leg.Settings.TravelM - leg.Height, 0, leg.Settings.TravelM);
                    ImGui.Text($"Compression {comp:0.00} m   Load {leg.LoadKN:0} kN");
                }
                ImGui.PopID();
                if (ch)
                {
                    var ns = s.ToSettings();
                    // apply to every leg on the vehicle (symmetry)
                    foreach (var p in vehicle.Parts.Parts)
                    {
                        var pm = p.Modules.Get<KeyframeAnimationModule>();
                        for (int i = 0; i < pm.Length; i++)
                            if (pm[i].TemplateId == LegGeo.ModuleId) LegConfig.Set(p.Id ?? "", ns.Clone());
                    }
                }
            }
            catch (Exception e) { Log.ErrorOnce("legmenu", e.ToString()); }
        }
    }

    internal static class LegSettingsRefExt
    {
        // ImGui needs ref float fields; a small mutable mirror of LegSettings
        public sealed class Edit
        {
            public float Sag_, Comp_, Reb_, TravelM_;
            public LegSettings ToSettings() => new() { SagFraction = Sag_, CompressionRatio = Comp_, ReboundRatio = Reb_, TravelM = TravelM_ };
        }
        public static Edit ToEdit(this LegSettings s) => new() { Sag_ = s.SagFraction, Comp_ = s.CompressionRatio, Reb_ = s.ReboundRatio, TravelM_ = s.TravelM };
    }
}
