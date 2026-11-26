// Rocket plume meets the surface: when a vehicle's engines fire within reach of the deck of the drone ship, a
// landing pad or the ground, its volumetric plume is turned out along the surface instead of passing through it.
//
// KSA's exhaust already models a plume folding back on itself in a headwind (ExhaustDeformation.ComputePlumeFold
// -> private ApplyFold: fold start station, fold radius, fold angle). This reuses that shape: the fold starts where
// the plume axis meets the surface and turns ~100 deg, so the gas fans out radially along the deck like the real
// touchdown photos. Everything is in the renderer's camera-relative frame (CCE axes, origin at the camera).
//
// Hooks: Vehicle.AddVolumetricExhaustInstances prefix/postfix (per vehicle: find the surface plane under it),
// VolumetricExhaustRenderer.AddPlume prefix (tag that vehicle's plumes), AddInstanceCore prefix (which plume is
// being built), ExhaustDeformation.ComputePlumeFold prefix (our fold instead of the wind fold when it hits).
using System.Reflection;
using System.Runtime.CompilerServices;
using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace SterlingFlight
{
    internal static class PlumeGround
    {
        public const double Range = 300.0;               // only look for a surface this close under the vehicle (m)
        const float TurnAngle = 1.75f;                   // ~100 deg: out along the surface, lifting a little

        internal struct Plane { public float3 Point, Normal; }
        internal static readonly ConditionalWeakTable<VolumetricExhaustInstance, StrongBox<Plane>> Planes = new();
        [ThreadStatic] internal static Plane? VehiclePlane;
        [ThreadStatic] internal static Plane? Current;

        internal delegate void ApplyFoldFn(ref ExhaustInstance s, float3 axisOffset, float foldStart, float foldRadius, float foldAngle);
        internal static readonly ApplyFoldFn? ApplyFold = MakeApplyFold();

        static ApplyFoldFn? MakeApplyFold()
        {
            try
            {
                var m = typeof(ExhaustDeformation).GetMethod("ApplyFold", BindingFlags.NonPublic | BindingFlags.Static);
                return m == null ? null : (ApplyFoldFn)Delegate.CreateDelegate(typeof(ApplyFoldFn), m);
            }
            catch (Exception e) { Log.Error("plume fold hook: " + e.Message); return null; }
        }

        /// surface under the vehicle in the renderer frame, or null when nothing is in reach
        internal static Plane? FindPlane(Vehicle v, Camera camera)
        {
            if (v.Orbit?.Parent == null) return null;
            var posCci = v.GetPositionCci();
            double r = V.Len(posCci); if (r < 1) return null;
            var up = V.Mul(posCci, 1.0 / r);
            // landings only: not on the pad / at liftoff (descending, or the booster's landing burn incl. a hover)
            bool landingBurn = v == Autopilot.Booster && (Autopilot.StackPhase == "LANDING BURN" || Autopilot.StackPhase == "LANDED");
            if (!landingBurn && V.Dot(v.GetSurfaceVelocityCci(), up) > -0.5) return null;
            double surfR = double.NaN;
            // the drone ship's deck, when the vehicle is over it
            if (DroneShip.Alive && v != DroneShip.Ship && v.Orbit.Parent is Celestial cel)
            {
                var station = Double3Ex.Transform(DroneShip.DeckTargetCcf, cel.GetCcf2Cci());
                double deckR = DroneShip.DeckRadius > 0 ? DroneShip.DeckRadius : V.Len(station);
                var off = V.Sub(posCci, V.Mul(V.Norm(station), r));
                if (V.Len(off) < 60 && r - deckR < Range && r > deckR) surfR = deckR;
            }
            if (double.IsNaN(surfR))
            {
                double radar = v.GetRadarAltitude();
                if (!(radar > 0 && radar < Range)) return null;
                surfR = r - radar;
            }
            var cci2cce = v.Parent.GetCci2Cce();
            var pEgo = V.Add(camera.GetPositionEgo(v), Double3Ex.Transform(V.Mul(up, surfR - r), cci2cce));
            var n = Double3Ex.Transform(up, cci2cce);
            return new Plane { Point = new float3((float)pEgo.X, (float)pEgo.Y, (float)pEgo.Z), Normal = new float3((float)n.X, (float)n.Y, (float)n.Z) };
        }
    }

    [HarmonyPatch(typeof(Vehicle), nameof(Vehicle.AddVolumetricExhaustInstances))]
    internal static class PlumeGroundVehicle
    {
        [HarmonyPrefix]
        static void Prefix(Vehicle __instance, Camera camera)
        {
            try { PlumeGround.VehiclePlane = PlumeGround.FindPlane(__instance, camera); }
            catch (Exception e) { PlumeGround.VehiclePlane = null; Log.ErrorOnce("plumeplane", e.ToString()); }
        }

        [HarmonyPostfix]
        static void Postfix() => PlumeGround.VehiclePlane = null;
    }

    [HarmonyPatch(typeof(VolumetricExhaustRenderer), nameof(VolumetricExhaustRenderer.AddPlume))]
    internal static class PlumeGroundTag
    {
        [HarmonyPrefix]
        static void Prefix(VolumetricExhaustInstance instance)
        {
            if (instance == null) return;
            var p = PlumeGround.VehiclePlane;
            PlumeGround.Planes.Remove(instance);
            if (p.HasValue) PlumeGround.Planes.Add(instance, new StrongBox<PlumeGround.Plane>(p.Value));
        }
    }

    [HarmonyPatch(typeof(VolumetricExhaustRenderer), "AddInstanceCore")]
    internal static class PlumeGroundBuild
    {
        [HarmonyPrefix]
        static void Prefix(VolumetricExhaustInstance volumetricExhaustInstance)
        {
            PlumeGround.Current = volumetricExhaustInstance != null && PlumeGround.Planes.TryGetValue(volumetricExhaustInstance, out var b) ? b.Value : null;
        }

        [HarmonyPostfix]
        static void Postfix() => PlumeGround.Current = null;
    }

    [HarmonyPatch(typeof(ExhaustDeformation), nameof(ExhaustDeformation.ComputePlumeFold))]
    internal static class PlumeGroundFold
    {
        [HarmonyPrefix]
        static bool Prefix(ref ExhaustInstance shaderData)
        {
            try
            {
                var p = PlumeGround.Current;
                if (!p.HasValue || PlumeGround.ApplyFold == null || shaderData.coreLength <= 0.001f) return true;
                var e = shaderData.volumeTopAndLength.XYZ;
                var axis = shaderData.axisAndBoundingRadius.XYZ;
                float down = -float3.Dot(axis, p.Value.Normal);           // plume pointing into the surface
                if (down < 0.35f) return true;
                // the fold is round about the plume axis: square the axis to the surface so the spread lies flat on the
                // deck when the booster is tilted (landing tilts are < ~12 deg; partial above 20 deg so it never kinks)
                float tilt = MathF.Acos(Math.Clamp(down, -1f, 1f));
                float w = tilt < 0.35f ? 1f : 0.6f;
                var squared = (axis * (1 - w) - p.Value.Normal * w).NormalizeOrZero();
                if (!squared.IsNearlyZero())
                {
                    axis = squared;
                    shaderData.axisAndBoundingRadius = new float4(axis, shaderData.axisAndBoundingRadius.W);
                    down = -float3.Dot(axis, p.Value.Normal);
                }
                float t = float3.Dot(e - p.Value.Point, p.Value.Normal) / down;   // distance along the axis to the surface
                if (t <= 0.05f || t >= shaderData.coreLength * 0.97f) return true;
                // turn starts a little above the deck; the bend radius is the plume's width there
                float radius = MathF.Max(0.6f * shaderData.nozzleRadius + 0.12f * t, 0.5f);
                float start = MathF.Max(t - radius, 0.02f * shaderData.coreLength);
                PlumeGround.ApplyFold(ref shaderData, float3.Zero, start, radius, 1.75f);
                return false;                                            // replaces the wind fold for this plume
            }
            catch (Exception ex) { Log.ErrorOnce("plumefold", ex.ToString()); return true; }
        }
    }
}
