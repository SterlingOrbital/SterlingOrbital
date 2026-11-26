// Sparrow octaweb engine sets (9 / 3 / 1), right-click the octaweb.
//
// SparrowOctawebAssets.xml splits the nine Anvil engines over three RocketEngineControllers (each one is its own on/off
// engine group in KSA):
//   SparrowOW_Ctrl_Center  centre engine            -> 1-engine set (landing burn)
//   SparrowOW_Ctrl_Pair    two opposite outer ones  -> + centre = 3-engine set (boostback / entry burn)
//   SparrowOW_Ctrl_Outer6  the other six            -> all = 9
// The chosen set is remembered per octaweb. Once the engines are lit (the centre group is active, e.g. after
// staging lights all three groups), every frame the groups outside the set are switched off and the groups in
// it switched on, through KSA's own engine activation event (EngineController.SetIsActive). Shutting the
// engines down (the centre group inactive) leaves everything alone, so staging and engine cut-off still work.
using System.Collections.Concurrent;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace SterlingFlight
{
    internal static class EngineSets
    {
        public const string Center = "SparrowOW_Ctrl_Center", Pair = "SparrowOW_Ctrl_Pair", Outer = "SparrowOW_Ctrl_Outer6";
        // key: the centre controller's module instance id; value: 9, 3 or 1
        public static readonly ConcurrentDictionary<ulong, int> Mode = new();

        public static bool Find(Part p, out EngineController? c, out EngineController? pair, out EngineController? outer)
        {
            c = pair = outer = null;
            var m = p.Modules.Get<EngineController>();
            for (int i = 0; i < m.Length; i++)
            {
                if (m[i].TemplateId == Center) c = m[i];
                else if (m[i].TemplateId == Pair) pair = m[i];
                else if (m[i].TemplateId == Outer) outer = m[i];
            }
            return c != null && pair != null && outer != null;
        }

        public static int Get(EngineController c) => Mode.TryGetValue(c.InstanceId, out var n) ? n : 9;

        /// bring the groups in line with the set; only while the engines are lit
        public static void Enforce(Vehicle v, EngineController c, EngineController pair, EngineController outer)
        {
            if (!c.IsActive) return;
            int n = Get(c);
            bool wantPair = n >= 3, wantOuter = n >= 9;
            if (pair.IsActive != wantPair) pair.SetIsActive(v, wantPair);
            if (outer.IsActive != wantOuter) outer.SetIsActive(v, wantOuter);
        }
    }

    [HarmonyPatch(typeof(InputEvents), nameof(InputEvents.ApplyInputEvents))]
    internal static class EngineSetTick
    {
        [HarmonyPostfix]
        static void Postfix()
        {
            try
            {
                if (EngineSets.Mode.IsEmpty) return;
                var sys = Universe.CurrentSystem;
                if (sys == null) return;
                var all = sys.All.AsSpan();
                for (int i = 0; i < all.Length; i++)
                {
                    if (all[i] is not Vehicle v || v.IsDisposed) continue;
                    foreach (var p in v.Parts.Parts)
                        if (EngineSets.Find(p, out var c, out var pair, out var outer))
                            EngineSets.Enforce(v, c!, pair!, outer!);
                }
            }
            catch (Exception e) { Log.ErrorOnce("enginesets", e.ToString()); }
        }
    }

    [HarmonyPatch(typeof(Part), nameof(Part.ShowContextMenu))]
    internal static class EngineSetMenu
    {
        [HarmonyPostfix]
        static void Postfix(Part __instance, Vehicle vehicle)
        {
            try
            {
                if (!EngineSets.Find(__instance, out var c, out var pair, out var outer)) return;
                int n = EngineSets.Get(c!);
                ImGui.PushID("SparrowEngineSet" + __instance.Id);
                ImGui.SeparatorText("Engine set");
                foreach (var k in new[] { 9, 3, 1 })
                {
                    float2? size = null;
                    string label = (k == n ? "> " : "  ") + k + (k == 1 ? " engine" : " engines");
                    if (ImGui.Button(label, in size))
                    {
                        EngineSets.Mode[c!.InstanceId] = k;
                        EngineSets.Enforce(vehicle, c!, pair!, outer!);
                        Log.Info($"octaweb {__instance.Id}: engine set {k}");
                    }
                    if (k != 1) ImGui.SameLine();
                }
                int lit = (c!.IsActive ? 1 : 0) + (pair!.IsActive ? 2 : 0) + (outer!.IsActive ? 6 : 0);
                ImGui.Text(c.IsActive ? $"Running: {lit} of 9" : "Engines off (the set applies once they are lit)");
                ImGui.PopID();
            }
            catch (Exception e) { Log.ErrorOnce("enginesetmenu", e.ToString()); }
        }
    }
}

namespace SterlingFlight
{
    /// The octaweb's three engine groups (centre / pair / outer six) are separate EngineControllers, so the staging list
    /// showed three engine icons. Only the centre group is drawn now; the pair and outer-six groups stay in the same
    /// stage (staging still lights them, and delta-v / TWR still count all nine engines) and EngineSets switches them.
    [HarmonyPatch(typeof(SequenceList), "IsActionVisible")]
    internal static class HideOctawebSubGroups
    {
        [HarmonyPostfix]
        static void Postfix(ISequenced module, ref bool __result)
        {
            if (__result && module is EngineController ec && (ec.TemplateId == EngineSets.Pair || ec.TemplateId == EngineSets.Outer))
                __result = false;
        }
    }

    /// Only the centre group's icon is shown (HideOctawebSubGroups), so a player dragging "the octaweb" to another stage
    /// really drags just the centre EngineController; KSA's MoveModule then left the hidden pair and outer six behind in
    /// the old stage (1 engine lit in one stage, 8 in another). Move the other two groups with it.
    [HarmonyPatch(typeof(SequenceList), nameof(SequenceList.MoveModule))]
    internal static class MoveOctawebGroupsTogether
    {
        static bool IsGroup(string? id) => id == EngineSets.Center || id == EngineSets.Pair || id == EngineSets.Outer;

        [HarmonyPostfix]
        static void Postfix(SequenceList __instance, ISequenced module, Part part, int number)
        {
            try
            {
                if (module is not EngineController ec || !IsGroup(ec.TemplateId)) return;
                bool moved = false;
                var e = part.GetSubtreeSequencedModules().GetEnumerator();
                while (e.MoveNext())
                {
                    var m = e.Current;
                    if (ReferenceEquals(m, module) || m is not EngineController o || !IsGroup(o.TemplateId) || m.Sequence == number) continue;
                    m.MoveToSequence(number);
                    moved = true;
                }
                if (moved) __instance.ResetCaches();
            }
            catch (Exception ex) { Log.ErrorOnce("octaweb-stage-move", "octaweb stage move: " + ex.Message); }
        }
    }
}
