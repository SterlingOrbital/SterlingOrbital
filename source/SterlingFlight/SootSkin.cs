// Entry-burn soot on the Sparrow first stage: while a vehicle carrying the S1 tank fires its engines flying engines-
// first through the upper atmosphere (entry burn, then the landing burn adds the rest; cold-gas attitude thrusters never count), soot builds up on the tank
// and its texture steps through a ladder of sooted versions (SparrowPartsAssets.xml: SparrowS1_SootLadderSubPart,
// textures from branding/make_s1_soot.py).
//
// Per vehicle, not per template: each part's PartModelModule.PartModel (the render model KSA batches by) is pointed
// at the PartModel of the matching ladder step and that vehicle's PartTreeRenderData membership is rebuilt, so other
// Sparrow tanks in the scene (a clean booster on the pad) keep their own look. The editor "soot" variants are
// separate parts and are never touched. Soot is kept for the session; it is not saved.
//
// Hook: Vehicle.AddVolumetricExhaustInstances prefix (main thread, once per vehicle per frame, gives frame dt).
using System.Runtime.CompilerServices;
using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace SterlingFlight
{
    internal static class SootSkin
    {
        public const int Steps = 20;
        public const double FullEntrySeconds = 30.0;      // entry-burn regime rate (a ~23 s entry burn leaves ~0.75)
        public const double FullLandingSeconds = 45.0;    // landing-burn regime rate (adds the last ~0.25)
        public const double MinSpeed = 30.0;              // m/s; below this the landing plume no longer reaches the tank
        public const float MinMainNozzleRadius = 0.25f;   // m; main engines ~0.46, cold-gas attitude thrusters ~0.07
        public const double MaxAltitude = 90_000.0;       // m; above this there is no air to carry soot back onto the tank

        // clean model key -> ladder model key prefix (prefix + step + "Model")
        static readonly Dictionary<string, string> Ladder = new()
        {
            ["SparrowS1_SkinModel"] = "SparrowS1_SkinSoot",
            ["SparrowS1_SkinPlainModel"] = "SparrowS1_SkinPlainSoot",
            ["SparrowS1_DetailsModel"] = "SparrowS1_DetailsSoot",
            ["SparrowS1_DetailsPlainModel"] = "SparrowS1_DetailsSoot",
        };
        static Dictionary<string, PartModelModule.Template>? _ladderTemplates;
        static bool _ladderFailed;

        sealed class State { public double Level; public int Applied; public int PartCount = -1; public bool HasTank; public bool Depositing; }
        static readonly ConditionalWeakTable<Vehicle, State> States = new();
        static readonly ConditionalWeakTable<PartModelModule, PartModel> Original = new();

        // test switch: environment variable STERLING_SOOT_TEST=1 soots every Sparrow tank in view over 10 s
        static readonly bool TestMode = Environment.GetEnvironmentVariable("STERLING_SOOT_TEST") == "1";

        public static double LevelOf(Vehicle v) => States.TryGetValue(v, out var s) ? s.Level : 0;

        static Dictionary<string, PartModelModule.Template>? LadderTemplates()
        {
            if (_ladderTemplates != null || _ladderFailed) return _ladderTemplates;
            const string LadderId = "SparrowS1_SootLadderSubPart";
            if (!ModLibrary.TryGet<SubPartTemplate>(LadderId, out var sp) || sp == null) sp = ScanParts(LadderId);
            if (sp == null) { _ladderFailed = true; Log.Error("soot: ladder subpart " + LadderId + " not found"); return null; }
            var d = new Dictionary<string, PartModelModule.Template>();
            foreach (var c in sp.Components)
                if (c is PartModelModule.Template t && t.Key != null) d[t.Key] = t;
            Log.Info("soot: " + d.Count + " ladder models");
            return _ladderTemplates = d;
        }

        /// fallback lookup: walk ModLibrary.AllParts (internal) for a SubPartTemplate with this Id
        static SubPartTemplate? ScanParts(string id)
        {
            try
            {
                var f = typeof(ModLibrary).GetField("AllParts", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Static);
                var coll = f?.GetValue(null);
                var list = coll?.GetType().GetMethod("GetList")?.Invoke(coll, null) as System.Collections.IEnumerable;
                if (list == null) { Log.Error("soot: AllParts not readable"); return null; }
                int n = 0; var near = new List<string>();
                foreach (var o in list)
                {
                    n++;
                    if (o is SubPartTemplate s)
                    {
                        if (string.Equals(s.Id, id, StringComparison.OrdinalIgnoreCase)) return s;
                        if (s.Id != null && s.Id.Contains("SparrowS1")) near.Add(s.Id);
                    }
                }
                Log.Error($"soot: {id} not in AllParts ({n} entries); SparrowS1 subparts: {string.Join(", ", near)}");
            }
            catch (Exception e) { Log.Error("soot: scan failed " + e.Message); }
            return null;
        }

        static bool IsEngineFiring(Vehicle v)
        {
            var e = v.Parts.RocketNozzles.ModulesAndAllStates.GetEnumerator();
            while (e.MoveNext())
            {
                var c = e.Current;
                if (c.Module is TurbineExhaustNozzle) continue;
                if (c.Module.FxExitRadius < MinMainNozzleRadius) continue;   // attitude thrusters (flip) do not soot
                if (c.State.DutyCycle > 0f) return true;
            }
            return false;
        }

        /// soot per second for this vehicle right now (0 when nothing is depositing)
        static double Rate(Vehicle v)
        {
            if (v.Orbit?.Parent == null || !IsEngineFiring(v)) return 0;
            var r = v.GetPositionCci(); double rmag = V.Len(r);
            if (rmag - v.Orbit.Parent.MeanRadius > MaxAltitude) return 0;
            var vs = v.GetSurfaceVelocityCci(); double speed = V.Len(vs);
            if (speed < MinSpeed) return 0;
            var nose = Double3Ex.Transform(new double3(1, 0, 0), v.GetBody2Cci());
            if (V.Dot(vs, nose) > -0.5 * speed) return 0;                 // not flying engines-first: the plume trails away
            double t = Math.Clamp((speed - 150) / 650, 0, 1);              // fast (entry burn) -> slow (landing burn)
            return t / FullEntrySeconds + (1 - t) / FullLandingSeconds;
        }

        internal static void Update(Vehicle v, double frameDt)
        {
            var s = States.GetValue(v, _ => new State());
            var tree = v.Parts;
            if (tree.Count != s.PartCount)                                // vehicle changed (staging): re-check for the tank
            {
                s.PartCount = tree.Count; s.HasTank = false; s.Applied = -1;
                foreach (var m in tree.Modules.Get<PartModelModule>())
                    if (Ladder.ContainsKey(OriginalOf(m).Template.Key ?? "")) { s.HasTank = true; break; }
            }
            if (!s.HasTank) return;
            double dt = frameDt * Universe.GetSimulationSpeed();
            double rate = TestMode ? 0.1 : Rate(v);
            if ((rate > 0) != s.Depositing)
            {
                s.Depositing = rate > 0;
                double alt = v.Orbit?.Parent != null ? V.Len(v.GetPositionCci()) - v.Orbit.Parent.MeanRadius : 0;
                Log.Info($"soot: {v.Id} deposit {(s.Depositing ? "start" : "stop")} at {alt / 1000:0.0} km, {V.Len(v.GetSurfaceVelocityCci()):0} m/s, level {s.Level:0.00}");
            }
            if (dt > 0 && dt < 5 && rate > 0 && s.Level < 1.0)
            {
                s.Level = Math.Min(1.0, s.Level + rate * dt);
                _liveOwner = new WeakReference<Vehicle>(v);                 // the booster gaining soot gets the live fade
            }
            bool owner = _liveOwner != null && _liveOwner.TryGetTarget(out var ov) && ReferenceEquals(ov, v);
            int code;
            if (owner && s.Level > 0 && !SootLive.Failed)
            {
                bool plain = IsPlain(v);
                SootLive.Request((float)s.Level, plain);
                code = SootLive.ReadyFor(plain, (float)s.Level) ? LiveCode : StepOf(s.Level);  // ladder until a live upload at this level lands
            }
            else code = StepOf(s.Level);
            if (code != s.Applied) { Apply(v, code); s.Applied = code; }
        }

        const int LiveCode = 1000;
        static WeakReference<Vehicle>? _liveOwner;
        static int StepOf(double level) => (int)Math.Clamp(Math.Floor(level * Steps + 0.5), 0, Steps);

        static bool IsPlain(Vehicle v)
        {
            foreach (var m in v.Parts.Modules.Get<PartModelModule>())
                if (OriginalOf(m).Template.Key == "SparrowS1_SkinPlainModel") return true;
            return false;
        }

        static PartModel OriginalOf(PartModelModule m) => Original.GetValue(m, mm => mm.PartModel);

        // code: 0 = clean, 1..Steps = ladder step, LiveCode = live (continuously blended) models
        static void Apply(Vehicle v, int code)
        {
            var lad = code > 0 ? LadderTemplates() : null;
            if (code > 0 && lad == null) return;
            bool changed = false;
            foreach (var m in v.Parts.Modules.Get<PartModelModule>())
            {
                var orig = OriginalOf(m);
                if (!Ladder.TryGetValue(orig.Template.Key ?? "", out var prefix)) continue;
                PartModel target = orig;
                string key = code == LiveCode ? prefix + "LiveModel" : prefix + code + "Model";
                if (code > 0 && lad!.TryGetValue(key, out var t)) target = PartModel.Get(t);
                if (!ReferenceEquals(m.PartModel, target)) { m.PartModel = target; changed = true; }
            }
            if (changed)
            {
                v.Parts.RenderData.InvalidateMembership();
                Log.Info($"soot: {v.Id} {(code == LiveCode ? "live" : "step " + code)} (level {LevelOf(v):0.00})");
            }
        }
    }

    [HarmonyPatch(typeof(Vehicle), nameof(Vehicle.AddVolumetricExhaustInstances))]
    internal static class SootSkinTick
    {
        [HarmonyPrefix]
        static void Prefix(Vehicle __instance, double frameDeltaTime)
        {
            try { SootLive.Tick(); SootSkin.Update(__instance, frameDeltaTime); }
            catch (Exception e) { Log.ErrorOnce("soot-skin", e.ToString()); }
        }
    }
}
