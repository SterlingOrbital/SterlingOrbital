// SterlingFlight - flight code for the Sterling Orbital Sparrow rocket (StarMap code mod for Kitten Space Agency).
//
// KSA has no aerodynamic control surfaces, so this adds them for the "SparrowGridFinPart":
//   * VehicleUpdateState.UpdateActiveChutes (runs once per vehicle update, like the parachute module feed):
//     find the vehicle's grid fins (KeyframeAnimationModule "SparrowGridFinDeploy"), their deploy state, their
//     centre of pressure / span / cell axes in the body frame (vehicle assembly axes, origin at the CoM -
//     same convention KSA uses for ActiveChute.AttachBody) and the pilot's rotation request.
//   * PhysicsStates.ComputeDerivatives (static; every physics sub-step): for each deployed fin compute the
//     local airflow, pick a deflection that pushes toward the requested rotation, and add the fin's normal
//     force + drag at the fin with Disturbances.AddForceBody.
// The rotation request is built exactly like FlightComputer.ComputeTvcControl builds the manual gimbal
// command (flags 2/4 -> +/-X, 8/16 -> +/-Y, 32/64 -> +/-Z in the control frame, then Ctrl2Asmb), so the fins
// always agree with the engines about which way "pitch up" is. No input -> fins trail neutral (drag only).
using System.Collections.Concurrent;
using System.Reflection;
using System.Runtime.CompilerServices;
using Brutal.Numerics;
using HarmonyLib;
using KSA;
using StarMap.API;

namespace SterlingFlight
{
    [StarMapMod]
    public class SterlingFlightMod
    {
        internal static Harmony? H;

        [StarMapAllModsLoaded]
        public void OnFullyLoaded()
        {
            Log.Info("loading, KSA " + typeof(Vehicle).Assembly.GetName().Version);
            H = new Harmony("SterlingFlight");
            try
            {
                // patch class by class so one changed game method doesn't take the rest down
                foreach (var t in typeof(SterlingFlightMod).Assembly.GetTypes())
                {
                    if (t.GetCustomAttributes(typeof(HarmonyPatch), false).Length == 0) continue;
                    try { H.CreateClassProcessor(t).Patch(); }
                    catch (Exception e) { Log.Error("patch " + t.Name + " failed: " + (e.InnerException?.InnerException?.Message ?? e.InnerException?.Message ?? e.Message)); }
                }
                Webcast.RequestFontRebuild();   // the atlas was built before we loaded: rebuild once to add Bai Jamjuree
                _ = FinAero.Mesh;   // build the fin aero mesh now so the log shows it
                Log.Info("patched: " + string.Join(", ", H.GetPatchedMethods().Select(m => m.DeclaringType?.Name + "." + m.Name)));
            }
            catch (Exception e) { Log.Error("patching failed: " + e); }
        }

        [StarMapUnload]
        public void Unload()
        {
            H?.UnpatchAll(H.Id);
            H = null;
        }
    }

    internal static class Tuning
    {
        public const string ModuleId = "SparrowGridFinDeploy";
        public const string PartId = "SparrowGridFin";
        // grid fin in its part space (KSA / glTF axes): hinge at (0,0,-0.212); deployed the fin sticks out
        // along -Z, its lattice cells look along +X (the vehicle axis).
        public static readonly double3 CpPart = new(0.0, 0.0, -1.10);
        public static readonly double3 SpanPart = new(0.0, 0.0, -1.0);
        public static readonly double3 CellPart = new(1.0, 0.0, 0.0);
        public const double Area = 2.0;               // m^2, lattice planform (~1.4 m x 1.5 m)
        public const double NormalCoeff = 1.8;        // normal force per unit sin(alpha) (grid fins are very effective)
        public const double DragCoeff = 0.8;          // lattice drag
        public const double MaxDeflection = 20.0 * Math.PI / 180.0;
        public const double Gain = 1.5;               // command -> deflection fraction
        public const double DeploySeconds = 3.0;      // matches the deploy animation
        public const double SlewRate = 60.0 * Math.PI / 180.0; // fin actuator rad/s
    }

    internal sealed class Fin
    {
        public ulong Key;
        public double3 Pos, Span, Cell;   // body frame (vehicle assembly axes, origin at CoM)
        public double Deploy;             // 0..1
        public double Deflection;         // rad, slewed (drives both the forces and the visual)
        public double Target;             // rad, commanded
        public long Stamp;                // last update (ms)
        public double SlewDt;             // s since the previous update
        public bool Shown;                // a deflection is currently applied to the visual
        public KeyframeAnimationModule? Module;
        // game-aero model (FinAero.cs)
        public volatile FinPose? Pose;     // live fin frame, vehicle assembly axes
        public KSA.Aero.AeroSurfaceFlow? Flow;
        public double3 LastForce;
        public bool HasAnimPose;           // fin subpart pose from the animation, before our deflection
        public doubleQuat AnimQ;
        public double3 AnimT;
    }

    internal sealed class FinSet
    {
        public Fin[] Fins = Array.Empty<Fin>();
        public double3 Command;           // requested torque direction, body frame, components -1..1
        public long Stamp;
    }

    internal static class Registry
    {
        public static readonly ConcurrentDictionary<VehicleUpdateState, FinSet> Sets = new();
        public static volatile VehicleUpdateState[] Snapshot = Array.Empty<VehicleUpdateState>();
        // one persistent Fin per animation module instance, so deploy fraction and actuator position carry over
        public static readonly ConcurrentDictionary<ulong, Fin> FinObjs = new();

        public static void Publish()
        {
            var now = Environment.TickCount64;
            foreach (var kv in Sets)
                if (now - kv.Value.Stamp > 5000) Sets.TryRemove(kv.Key, out _);
            Snapshot = Sets.Keys.ToArray();
        }
    }

    internal static class V
    {
        public static double3 Add(double3 a, double3 b) => new(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
        public static double3 Sub(double3 a, double3 b) => new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
        public static double3 Mul(double3 a, double s) => new(a.X * s, a.Y * s, a.Z * s);
        public static double Dot(double3 a, double3 b) => a.X * b.X + a.Y * b.Y + a.Z * b.Z;
        public static double3 Cross(double3 a, double3 b) => new(a.Y * b.Z - a.Z * b.Y, a.Z * b.X - a.X * b.Z, a.X * b.Y - a.Y * b.X);
        public static double Len(double3 a) => Math.Sqrt(Dot(a, a));
        public static double3 Norm(double3 a) { var l = Len(a); return l > 1e-12 ? Mul(a, 1.0 / l) : new double3(0, 0, 0); }
    }

    [HarmonyPatch(typeof(VehicleUpdateState), nameof(VehicleUpdateState.UpdateActiveChutes))]
    internal static class GatherFins
    {
        static bool _loggedIds;

        [HarmonyPostfix]
        static void Postfix(VehicleUpdateState __instance)
        {
            try { Gather(__instance); }
            catch (Exception e) { Log.ErrorOnce("gather", e.ToString()); }
        }

        static void Gather(VehicleUpdateState vus)
        {
            var vehicle = vus.ReadOnlyVehicle;
            if (vehicle == null) return;
            var mods = vehicle.Parts.Modules.Get<KeyframeAnimationModule>();
            List<Fin>? fins = null;
            var now = Environment.TickCount64;
            for (int i = 0; i < mods.Length; i++)
            {
                var m = mods[i];
                var part = m.Parent;
                bool isFin = m.TemplateId == Tuning.ModuleId ||
                             (part?.Template?.Id?.Contains(Tuning.PartId, StringComparison.Ordinal) ?? false);
                if (!_loggedIds)
                    Log.Info($"animation module '{m.TemplateId}' on part '{part?.Template?.Id}' -> grid fin: {isFin}");
                if (!isFin || part == null) continue;

                // deploy fraction follows the animation goal at the animation's pace
                var target = m.TimeGoal > 0.0f ? 1.0 : 0.0;
                var fin = Registry.FinObjs.GetOrAdd(m.InstanceId, id => new Fin { Key = id, Deploy = target, Stamp = now });
                var dt = Math.Clamp((now - fin.Stamp) / 1000.0, 0.0, 0.5);
                fin.Deploy += Math.Clamp(target - fin.Deploy, -dt / Tuning.DeploySeconds, dt / Tuning.DeploySeconds);
                fin.Stamp = now;
                fin.Module = m;
                fin.SlewDt = dt;

                var M = part.MatrixAsmb2VehicleAsmb;
                var com = vus.GetReadOnlyProps().CenterOfMassAsmb;
                var comD = new double3(com.X, com.Y, com.Z);
                var cp = Double3Ex.Transform(Tuning.CpPart, M);
                var span = V.Norm(V.Sub(Double3Ex.Transform(V.Add(Tuning.CpPart, Tuning.SpanPart), M), cp));
                var cell = V.Norm(V.Sub(Double3Ex.Transform(V.Add(Tuning.CpPart, Tuning.CellPart), M), cp));
                fin.Pos = V.Sub(cp, comD); fin.Span = span; fin.Cell = cell;
                fin.Flow ??= FinAero.NewFlow();
                fin.Pose = FinAero.ComputePose(part, fin);
                (fins ??= new()).Add(fin);
            }
            if (mods.Length > 0) _loggedIds = true;

            if (fins == null)
            {
                if (Registry.Sets.TryRemove(vus, out _)) Registry.Publish();
                return;
            }

            // pilot rotation request, built like FlightComputer.ComputeTvcControl (manual mode)
            var cmd = new double3(0, 0, 0);
            if (Autopilot.FinCommand.TryGetValue(vehicle, out var apCmd)) cmd = apCmd;      // mission autopilot
            else if (vus.IsControlled)
            {
                var f = ThrusterMapFlagsEx.WithNoTranslation(ThrusterMapFlagsEx.WithCanceledOpposingCommands(vus.ManualControlInputs.ThrusterCommandFlags));
                int bits = (int)f;
                double x = ((bits & 2) != 0 ? 1 : 0) - ((bits & 4) != 0 ? 1 : 0);
                double y = ((bits & 8) != 0 ? 1 : 0) - ((bits & 16) != 0 ? 1 : 0);
                double z = ((bits & 32) != 0 ? 1 : 0) - ((bits & 64) != 0 ? 1 : 0);
                var fc = vus.FlightComputer;
                cmd = fc != null ? Double3Ex.Transform(new double3(x, y, z), fc.Ctrl2Asmb) : new double3(x, y, z);
            }

            // fin commands: for each fin, which deflection sign turns the vehicle the requested way. That depends
            // on the airflow direction, so it works nose-first and tail-first; no airflow or no input -> neutral.
            var air = vus.GetReadOnlyStates().ComputeAirVelocityBody();
            var vel = new double3(air.X, air.Y, air.Z);
            var speed = V.Len(vel);
            var flow = speed > 1.0 ? V.Mul(vel, -1.0 / speed) : new double3(0, 0, 0);
            var cmdLen = V.Len(cmd);
            foreach (var fin in fins)
            {
                double target = 0;
                if (cmdLen > 1e-6 && speed > 1.0 && fin.Deploy > 0.5)
                {
                    var n0 = V.Norm(V.Cross(fin.Span, fin.Cell));
                    var g = V.Sub(V.Cross(fin.Pos, ControlModel.Force(1.0, fin, n0, flow, +Tuning.MaxDeflection)),
                                  V.Cross(fin.Pos, ControlModel.Force(1.0, fin, n0, flow, -Tuning.MaxDeflection)));
                    var gl = V.Len(g);
                    if (gl > 1e-12)
                        target = Tuning.MaxDeflection * Math.Clamp(Tuning.Gain * V.Dot(cmd, g) / gl, -1.0, 1.0);
                }
                fin.Target = target;
                var step = Tuning.SlewRate * fin.SlewDt;            // hydraulic actuator slew rate
                fin.Deflection += Math.Clamp(target - fin.Deflection, -step, step);
            }

            bool isNew = !Registry.Sets.ContainsKey(vus);
            Registry.Sets[vus] = new FinSet { Fins = fins.ToArray(), Command = cmd, Stamp = now };
            if (isNew) { Registry.Publish(); Log.Info($"vehicle {vus.Id}: {fins.Count} grid fin(s) under control"); }
        }
    }

    internal static class ControlModel
    {
        // grid fin as a lattice whose cells look along the (deflected) cell axis: normal force along the
        // deflected tangential axis proportional to sin(angle of attack), plus lattice drag along the flow
        internal static double3 Force(double k, Fin fin, double3 n0, double3 flow, double defl)
        {
            var c = V.Add(V.Mul(fin.Cell, Math.Cos(defl)), V.Mul(n0, Math.Sin(defl)));
            var n = V.Sub(V.Mul(n0, Math.Cos(defl)), V.Mul(fin.Cell, Math.Sin(defl)));
            _ = c;
            var normal = V.Mul(n, k * Tuning.NormalCoeff * V.Dot(flow, n));
            var drag = V.Mul(flow, k * Tuning.DragCoeff);
            return V.Add(normal, drag);
        }
    }

    // ---------------------------------------------------------------- visual fin deflection
    // KSA only re-places animated subparts while an animation plays (or when _transformsDirty is set). Keep the
    // grid fin animations "dirty" so KSA re-places the fin every update, then turn the fin about its own span
    // axis by the actuator deflection right after KSA has placed it.
    [HarmonyPatch(typeof(KeyframeAnimationModule), nameof(KeyframeAnimationModule.UpdateModules))]
    internal static class KeepFinAnimationLive
    {
        static readonly AccessTools.FieldRef<KeyframeAnimationModule, bool> Dirty =
            AccessTools.FieldRefAccess<KeyframeAnimationModule, bool>("_transformsDirty");

        // No longer forces the animation to re-run: KSA re-applies it on a worker thread (raw pose first, our pose
        // after), and the renderer caught the raw pose in between = flicker. The deflection is now set directly on
        // the main thread (PoseSync); this hook is kept only so older saves' dirty flags are left alone.
        [HarmonyPrefix]
        static void Prefix() { }
    }

    [HarmonyPatch(typeof(KeyframeAnimationModule), nameof(KeyframeAnimationModule.ApplyAnimationTransforms))]
    internal static class DeflectFinVisual
    {
        // fin subpart local frame = part space of the stowed fin: hinge shaft centre, fin hangs along -X
        internal static readonly double3 HingeLocal = new(0.0, 0.0, -0.212);
        internal static readonly double3 SpanLocal = new(-1.0, 0.0, 0.0);
        internal const string FinSubPart = "Fin1";
        static bool _logged;

        [HarmonyPostfix]
        static void Postfix(KeyframeAnimationModule __instance)
        {
            try
            {
                if (!Registry.FinObjs.TryGetValue(__instance.InstanceId, out var fin)) return;
                var parent = __instance.Parent;
                if (parent == null) return;
                // the fin's colliders are part-level (they never move with the animation): no rebuilds
                ContactMath.KeepColliders(parent);
                var sp = FinSub(parent);
                if (sp == null) return;
                // pose straight from the animation (deploy state), before our deflection: aero + visual use it
                fin.AnimQ = sp.Asmb2ParentAsmb; fin.AnimT = sp.PositionParentAsmb; fin.HasAnimPose = true;
                Apply(fin, sp);
            }
            catch (Exception e) { Log.ErrorOnce("visual", e.ToString()); }
        }

        internal static Part? FinSub(Part parent)
        {
            var subs = parent.SubParts;
            for (int i = 0; i < subs.Length; i++)
                if (subs[i].Id != null && subs[i].Id.Contains(FinSubPart, StringComparison.Ordinal)) return subs[i];
            return null;
        }

        /// main thread, every sim step: animation pose x actuator deflection, written in one go
        internal static void Apply(Fin fin, Part? sp = null)
        {
            try
            {
                var parent = fin.Module?.Parent;
                if (parent == null) return;
                sp ??= FinSub(parent);
                if (sp == null) return;
                if (!fin.HasAnimPose)
                {
                    if (fin.Shown) return;                                    // can't tell the raw pose apart yet
                    fin.AnimQ = sp.Asmb2ParentAsmb; fin.AnimT = sp.PositionParentAsmb; fin.HasAnimPose = true;
                }
                bool deflect = Math.Abs(fin.Deflection) > 1e-5;
                if (!deflect && !fin.Shown) return;                          // nothing to show, nothing to undo
                fin.Shown = deflect;
                {
                    if (!_logged && deflect) { _logged = true; Log.Info("deflecting fin subpart '" + sp.Id + "'"); }
                    var Q = fin.AnimQ;
                    var T = fin.AnimT;
                    var axis = V.Norm(Double3Ex.Transform(SpanLocal, Q));      // span axis, parent-assembly frame
                    var hingeRel = Double3Ex.Transform(HingeLocal, Q);         // hinge relative to the subpart origin
                    var D = doubleQuat.CreateFromAxisAngle(axis, fin.Deflection);
                    // rotate about the span axis through the hinge P = T + hingeRel:
                    //   T' = D (T - P) + P = T + hingeRel - D hingeRel,   Q' = D * Q
                    sp.PositionParentAsmb = V.Sub(V.Add(T, hingeRel), Double3Ex.Transform(hingeRel, D));
                    sp.Asmb2ParentAsmb = doubleQuat.Concatenate(Q, D);
                }
            }
            catch (Exception e) { Log.ErrorOnce("visualapply", e.ToString()); }
        }
    }

    /// Where this mod's own files live. Never a hard-coded Documents path: launchers such as Borea run separate
    /// game instances, so ask the game which folder it loaded the mod from (ModLibrary), fall back to the folder this
    /// DLL was loaded from, and only then to the game's own profile folder (Constants.DocumentsFolderPath).
    internal static class ModPaths
    {
        public const string ModId = "SterlingOrbital";
        static string? _dir;

        public static string Dir
        {
            get
            {
                if (_dir != null) return _dir;
                try
                {
                    var m = ModLibrary.Find(ModId);
                    if (m != null && !string.IsNullOrEmpty(m.DirectoryPath)) return _dir = m.DirectoryPath;
                }
                catch { }
                try
                {
                    var asm = System.IO.Path.GetDirectoryName(typeof(ModPaths).Assembly.Location);
                    if (!string.IsNullOrEmpty(asm) && System.IO.File.Exists(System.IO.Path.Combine(asm, "mod.toml"))) return _dir = asm;
                }
                catch { }
                return System.IO.Path.Combine(Constants.DocumentsFolderPath, "mods", ModId);   // not cached: retry once the library is up
            }
        }

        public static string File(params string[] parts) => System.IO.Path.Combine(new[] { Dir }.Concat(parts).ToArray());
    }

    internal static class Log
    {
        static string PathLog => ModPaths.File("SterlingFlight.log");
        static readonly ConcurrentDictionary<string, bool> Once = new();
        static readonly object Lock = new();

        public static void Info(string s) => Write("INFO  " + s);
        public static void Error(string s) => Write("ERROR " + s);
        public static void ErrorOnce(string key, string s) { if (Once.TryAdd(key, true)) Error(s); }

        static void Write(string s)
        {
            var line = $"{DateTime.Now:HH:mm:ss.fff} {s}";
            Console.WriteLine("[SterlingFlight] " + line);
            try { lock (Lock) System.IO.File.AppendAllText(PathLog, line + Environment.NewLine); } catch { }
        }
    }
}
