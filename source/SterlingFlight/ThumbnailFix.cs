// ThumbnailFix - part thumbnails show every model of a (sub)part, not just the last one.
//
// KSA's ThumbnailPart keeps a single PartModel and a single PartModelDynamic per (sub)part (the constructor loops
// over the template's components and the last one of each kind wins), and both CollectDraws and
// ComputeBoundingSphereRadius only use those. Stock parts never have more than one model per subpart, but ours do
// (engine assemblies, the octaweb bells, the S1 tank skin + details), so the editor thumbnails showed one fragment,
// framed on that fragment alone. These two postfixes draw the missing models and frame on all of them.
using Brutal.Numerics;
using HarmonyLib;
using KSA;
using KSA.Rendering;
using KSA.Rendering.Thumbnails;

namespace SterlingFlight
{
    internal static class ThumbnailModels
    {
        // every static / dynamic model template of this (sub)part, including the one KSA already kept
        internal static IEnumerable<PartModelModule.Template> Static(ThumbnailPart p)
        {
            if (p.Template?.Components == null) yield break;
            foreach (var c in p.Template.Components)
                if (c is PartModelModule.Template t && Ok(t.Mesh?.Value)) yield return t;
        }

        internal static IEnumerable<PartModelDynamicModule.Template> Dynamic(ThumbnailPart p)
        {
            if (p.Template?.Components == null) yield break;
            foreach (var c in p.Template.Components)
                if (c is PartModelDynamicModule.Template t && Ok(t.Mesh?.Value)) yield return t;
        }

        // 5949: meshes are DataReference<MeshReference>; AddDraw needs the interleaved device mesh (MeshFile
        // <Interleaved>true</Interleaved>) - skip anything without it instead of crashing the thumbnail pass
        static bool Ok(MeshReference? m)
        {
            try { return m != null && m.DeviceMeshInterleaved != null; } catch { return false; }
        }

        internal static IEnumerable<ThumbnailPart> RootAndChildren(ThumbnailPart root)
        {
            yield return root;
            if (root.Children == null) yield break;
            foreach (var c in root.Children) yield return c;
        }
    }

    [HarmonyPatch(typeof(ThumbnailCreator), nameof(ThumbnailCreator.CollectDraws))]
    internal static class ThumbnailDrawAllModels
    {
        [HarmonyPostfix]
        static void Postfix(ThumbnailPart inPart, ThumbnailRenderResources inResources)
        {
            try
            {
                if (inPart?.Template == null) return;
                float4x4 m = inPart.GetMatrix();
                foreach (var t in ThumbnailModels.Static(inPart))
                    if (!ReferenceEquals(t, inPart.Model?.Template)) inResources.AddDraw(m, t);
                foreach (var t in ThumbnailModels.Dynamic(inPart))
                    if (!ReferenceEquals(t, inPart.ModelDynamic?.Template)) inResources.AddDraw(m, t);
            }
            catch (Exception e) { Log.ErrorOnce("thumb-draw", "thumbnail draws: " + e); }
        }
    }

    [HarmonyPatch(typeof(ThumbnailPart), nameof(ThumbnailPart.ComputeBoundingSphereRadius))]
    internal static class ThumbnailFrameAllModels
    {
        // same maths as the original (bbox of the mesh bounds -> centre, farthest bbox corner -> radius, over the
        // root and its direct children), just over every model instead of one per part
        [HarmonyPostfix]
        static void Postfix(ThumbnailPart __instance, ref float3 outCenter, ref float __result)
        {
            try
            {
                var min = new float3(float.PositiveInfinity);
                var max = new float3(float.NegativeInfinity);
                int count = 0;
                foreach (var p in ThumbnailModels.RootAndChildren(__instance))
                {
                    if (p.Template == null) continue;
                    float4x4 m = p.GetMatrix();
                    foreach (var t in ThumbnailModels.Static(p)) { ThumbnailPart.ExpandBoundsForMesh(t.Mesh!.Value!, m, ref min, ref max); count++; }
                    foreach (var t in ThumbnailModels.Dynamic(p)) { ThumbnailPart.ExpandBoundsForMesh(t.Mesh!.Value!, m, ref min, ref max); count++; }
                }
                if (count == 0) return;

                var center = (min + max) * 0.5f;
                float r2 = 0f;
                foreach (var p in ThumbnailModels.RootAndChildren(__instance))
                {
                    if (p.Template == null) continue;
                    float4x4 m = p.GetMatrix();
                    foreach (var t in ThumbnailModels.Static(p)) r2 = MathF.Max(r2, ThumbnailPart.ComputeMaxDistanceSqToMeshCorners(t.Mesh!.Value!, m, center));
                    foreach (var t in ThumbnailModels.Dynamic(p)) r2 = MathF.Max(r2, ThumbnailPart.ComputeMaxDistanceSqToMeshCorners(t.Mesh!.Value!, m, center));
                }
                outCenter = center;
                __result = MathF.Sqrt(r2);
            }
            catch (Exception e) { Log.ErrorOnce("thumb-bounds", "thumbnail bounds: " + e); }
        }
    }
}
