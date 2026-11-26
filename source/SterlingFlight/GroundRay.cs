// Ray-cast the physics ground under a landing-leg pad (Bepu simulation of the vehicle's physics bubble).
// Bepu space = the bubble's phys frame (ConstraintSim poses bodies at Kinematic.PositionPhys / Body2Phys).
// The vehicle's own body is skipped; anything else (terrain blocks, ground clutter, pads, other vehicles) is ground.
using System.Numerics;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Trees;
using BepuUtilities.Memory;
using Brutal.Numerics;
using KSA;

namespace SterlingFlight
{
    internal static class GroundRay
    {
        const double Above = 3.0;
        [ThreadStatic] static BufferPool? _pool;          // the simulation's pool is not thread-safe              // start this far above the pad, so ground higher than expected is caught

        struct Handler : IRayHitHandler
        {
            public int Self;                   // own body handle value, -1 = none
            public float T;

            public bool AllowTest(CollidableReference c) =>
                !(Self >= 0 && c.Mobility != CollidableMobility.Static && c.BodyHandle.Value == Self);
            public bool AllowTest(CollidableReference c, int childIndex) => AllowTest(c);
            public void OnRayHit(in RayData ray, ref float maximumT, float t, Vector3 normal, CollidableReference c, int childIndex)
            {
                if (t < T) { T = t; maximumT = t; }
            }
        }

        /// pad height above the physics ground (m), or null if there is no simulation / nothing within range
        public static double? Cast(VehicleUpdateState? vs, double3 padPhys, double3 downPhys, double below)
        {
            var sim = vs?.Bubble?.ConstraintSim?.Simulation;
            if (sim == null) return null;
            var h = new Handler { Self = vs!.BepuHandles.Body.Value, T = float.MaxValue };
            var start = new Vector3((float)(padPhys.X - downPhys.X * Above), (float)(padPhys.Y - downPhys.Y * Above), (float)(padPhys.Z - downPhys.Z * Above));
            var dir = new Vector3((float)downPhys.X, (float)downPhys.Y, (float)downPhys.Z);
            float maxT = (float)(Above + below);
            sim.RayCast(start, dir, maxT, _pool ??= new BufferPool(), ref h);
            if (h.T > maxT) return null;
            return h.T - Above;
        }
    }
}
