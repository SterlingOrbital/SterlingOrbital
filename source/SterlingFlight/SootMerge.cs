// Gas-generator soot plume is drawn into its engine's main plume and absorbed there, instead of running on beside it.
//
// KSA's plume hierarchy only merges plumes whose exits are near-coplanar (ExhaustInteraction.ComputeCoplanarity),
// so a turbine exhaust that exits up beside the bell never interacts with the main jet. Merging them through the
// hierarchy would also average the two into one combined plume. Instead, per turbine plume (TurbineExhaustNozzle)
// whose engine (RocketNozzle.Rocket) also has a live main plume:
//   - its axis is turned slightly so it converges on a point inside the main jet, AbsorbStation main-exit radii
//     downstream of the main exit and AbsorbRadius radii off the main axis (a few degrees for the Anvil 1D);
//   - its density is faded out over the last stretch before that point (axial fade-out, also trims the volume), so
//     the dark stream thins and vanishes into the bright core like entrained gas.
// Everything is in the renderer's camera-relative frame. Hooks: AddPlume prefix (which nozzle/engine a plume
// belongs to), BeginPlumeGrouping prefix (new frame), AddInstanceCore prefix (axis + fade for the soot plume).
using System.Runtime.CompilerServices;
using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace SterlingFlight
{
    internal static class SootMerge
    {
        public const float AbsorbStation = 2.0f;     // main-exit radii downstream of the main exit
        public const float AbsorbRadius = 0.6f;      // main-exit radii off the main axis (inside the jet)
        public const float FadeBefore = 1.3f;        // fade starts this many main radii before the absorb point
        public const float FadeAfter = 0.15f;        // ... and is complete this many after it
        const float MaxTurnCos = 0.866f;             // never turn a soot plume more than 30 deg

        internal sealed class Info { public RocketNozzle Nozzle = null!; }
        internal static readonly ConditionalWeakTable<VolumetricExhaustInstance, Info> Nozzles = new();
        internal static readonly Dictionary<Rocket, VolumetricExhaustInstance> MainOf = new();

        internal static bool IsTurbine(RocketNozzle n) => n is TurbineExhaustNozzle;

        /// new axis + fade for a soot plume, or false to leave it alone
        internal static bool Plan(VolumetricExhaustInstance soot, float3 pos, ref float3 axis, ref ExhaustAxialFade fade)
        {
            if (!Nozzles.TryGetValue(soot, out var info) || !IsTurbine(info.Nozzle) || info.Nozzle.Rocket == null) return false;
            if (!MainOf.TryGetValue(info.Nozzle.Rocket, out var main) || main == null || !main.IsLive || !main.IsActive) return false;
            float rm = main.LastPlumeData.NozzleExitRadius;
            if (!(rm > 0.01f)) return false;
            var am = main.EmitterAxis.NormalizeOrZero(); if (am.IsNearlyZero()) return false;
            var pm = main.EmitterPosition;
            var v = pos - pm;
            var perp = v - am * float3.Dot(v, am);
            float d = perp.Length(); if (d < 1e-4f) return false;
            var target = pm + am * (AbsorbStation * rm) + perp * (AbsorbRadius * rm / d);
            var dir = target - pos;
            float sc = dir.Length(); if (sc < 0.05f) return false;
            var na = dir / sc;
            if (float3.Dot(na, axis.NormalizeOrZero()) < MaxTurnCos) return false;
            axis = na;
            float start = MathF.Max(0f, sc - FadeBefore * rm), end = sc + FadeAfter * rm;
            if (start < fade.Start) { fade.Start = start; fade.End = MathF.Min(fade.End, end); if (fade.End <= fade.Start) fade.End = fade.Start + 0.01f; }
            return true;
        }
    }

    [HarmonyPatch(typeof(VolumetricExhaustRenderer), nameof(VolumetricExhaustRenderer.BeginPlumeGrouping))]
    internal static class SootMergeFrame
    {
        [HarmonyPrefix]
        static void Prefix() => SootMerge.MainOf.Clear();
    }

    [HarmonyPatch(typeof(VolumetricExhaustRenderer), nameof(VolumetricExhaustRenderer.AddPlume))]
    internal static class SootMergeTag
    {
        [HarmonyPrefix]
        static void Prefix(VolumetricExhaustInstance instance, RocketNozzle nozzle)
        {
            try
            {
                if (instance == null || nozzle == null) return;
                SootMerge.Nozzles.AddOrUpdate(instance, new SootMerge.Info { Nozzle = nozzle });
                if (!SootMerge.IsTurbine(nozzle) && nozzle.Rocket != null && !SootMerge.MainOf.ContainsKey(nozzle.Rocket))
                    SootMerge.MainOf[nozzle.Rocket] = instance;
            }
            catch (Exception e) { Log.ErrorOnce("soot-tag", e.ToString()); }
        }
    }

    [HarmonyPatch(typeof(VolumetricExhaustRenderer), "AddInstanceCore")]
    internal static class SootMergeBuild
    {
        [HarmonyPrefix]
        static void Prefix(float3 emitterPosition, ref float3 axis, VolumetricExhaustInstance volumetricExhaustInstance,
                           ref ExhaustAxialFade fade, bool isMerged)
        {
            try
            {
                if (isMerged || volumetricExhaustInstance == null) return;
                SootMerge.Plan(volumetricExhaustInstance, emitterPosition, ref axis, ref fade);
            }
            catch (Exception e) { Log.ErrorOnce("soot-merge", e.ToString()); }
        }
    }
}
