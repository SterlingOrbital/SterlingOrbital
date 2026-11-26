// FinAero - grid fin forces from KSA's own aerodynamics (build 5901+ panel-method aero).
//
// KSA builds one aero panel mesh per vehicle from its colliders and only rebuilds it when the vehicle's shape
// changes (and the physics step waits for that rebuild), so a fin that turns every frame can't live in that mesh.
// Instead every grid fin gets its own aero mesh, built ONCE by KSA's own builder (AerodynamicsBuilder) from a
// lattice of thin plates in the fin's local frame. Every physics sub-step the fin's loads come from KSA's own
// LocalInclinationLaw + SkinFrictionLaw (same Mach / density / shielding model the rest of the vehicle uses),
// with the air direction expressed in the fin's current, deflected frame. The fin parts are left out of the
// vehicle mesh (ExcludeFinsFromVehicleAero) so nothing is counted twice.
//
// The game's "Draw Aerodynamics" overlay is extended to draw the fin meshes too (pressure-coloured in flight).
using System.Numerics;
using System.Reflection;
using System.Runtime.CompilerServices;
using BepuUtilities.Memory;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using HarmonyLib;
using KSA;
using KSA.Aero;

namespace SterlingFlight
{
    /// <summary>fin frame in the vehicle assembly frame (body axes, origin at the vehicle assembly origin)</summary>
    internal sealed record FinPose(double3 Origin, double3 Ex, double3 Ey, double3 Ez)
    {
        public Vector3 ToLocal(double3 v) => new((float)V.Dot(v, Ex), (float)V.Dot(v, Ey), (float)V.Dot(v, Ez));
        public double3 ToBody(Vector3 v) => V.Add(V.Add(V.Mul(Ex, v.X), V.Mul(Ey, v.Y)), V.Mul(Ez, v.Z));
    }

    internal static class FinAero
    {
        // Lattice in the fin subpart's local frame (= stowed fin, KSA/glTF axes): span along local -X from the
        // hinge (tip at X -1.714), chord along Y (+/-0.69), cells look along local Z (lattice depth, centre -0.2475).
        // Plates are thin boxes -> KSA treats them as sheets (two-sided panels).
        const float X0 = -1.69f, X1 = -0.22f, YHalf = 0.675f, ZCentre = -0.2475f, Depth = 0.30f, Plate = 0.02f;
        const int SpanCells = 7, ChordCells = 6;

        static Aerodynamics? _aero;
        static bool _failed;
        static readonly object Lock = new();

        public static Aerodynamics? Mesh
        {
            get
            {
                if (_aero != null || _failed) return _aero;
                lock (Lock)
                {
                    if (_aero == null && !_failed)
                    {
                        try { _aero = Build(); }
                        catch (Exception e) { _failed = true; Log.Error("fin aero mesh build failed: " + e); }
                        if (_aero == null) _failed = true;
                    }
                }
                return _aero;
            }
        }

        static Aerodynamics? Build()
        {
            var b = new AeroShapeSetBuilder();
            // spanwise stations (plates normal to the span) incl. root + tip frame
            for (int i = 0; i <= SpanCells; i++)
            {
                float x = X0 + (X1 - X0) * i / SpanCells;
                b.Add(new AeroShape(AeroShapeKind.Box, Quaternion.Identity, new Vector3(x, 0f, ZCentre),
                    new Vector3(0.5f * Plate, YHalf, 0.5f * Depth)), 0);
            }
            // chordwise stations (plates normal to the chord) incl. both side frames: these make the lift
            for (int j = 0; j <= ChordCells; j++)
            {
                float y = -YHalf + 2f * YHalf * j / ChordCells;
                b.Add(new AeroShape(AeroShapeKind.Box, Quaternion.Identity, new Vector3(0.5f * (X0 + X1), y, ZCentre),
                    new Vector3(0.5f * (X1 - X0), 0.5f * Plate, 0.5f * Depth)), 0);
            }
            var shapes = b.Build();
            var source = new AeroMeshSource(shapes, new[] { "SparrowGridFinLattice" }, 0xF9F1A7713Eul);
            var builder = new AerodynamicsBuilder();
            builder.Start(source, null, null, null);   // no worker pool -> builds synchronously
            var aero = builder.AwaitFinished();
            if (aero == null || aero.Mesh == null) { Log.Error("fin aero mesh: builder returned nothing (failed=" + builder.HasFailed + ")"); return null; }
            var m = aero.Mesh;
            Log.Info($"fin aero mesh built in {aero.BuildMillis:F0} ms: {m.SolidPanels.Length} solid + {m.SheetPanels.Length} sheet panels, faces {m.Surface.FaceCount}");
            return aero;
        }

        public static AeroSurfaceFlow? NewFlow()
        {
            var a = Mesh;
            if (a == null || a.Mesh.Surface.FaceCount == 0) return null;
            return new AeroSurfaceFlow(a.Mesh.Surface, new BufferPool());
        }

        /// <summary>live fin frame: part transform * animation pose * actuator deflection about the span axis</summary>
        public static FinPose? ComputePose(Part part, Fin? fin)
        {
            Part? sp = null;
            var subs = part.SubParts;
            for (int i = 0; i < subs.Length; i++)
                if (subs[i].Id != null && subs[i].Id.Contains(DeflectFinVisual.FinSubPart, StringComparison.Ordinal)) { sp = subs[i]; break; }
            if (sp == null) return null;

            doubleQuat Q; double3 T;
            if (fin != null && fin.HasAnimPose) { Q = fin.AnimQ; T = fin.AnimT; }
            else { Q = sp.Asmb2ParentAsmb; T = sp.PositionParentAsmb; }
            double defl = fin?.Deflection ?? 0.0;
            if (Math.Abs(defl) > 1e-7)
            {
                var axis = V.Norm(Double3Ex.Transform(DeflectFinVisual.SpanLocal, Q));
                var hingeRel = Double3Ex.Transform(DeflectFinVisual.HingeLocal, Q);
                var D = doubleQuat.CreateFromAxisAngle(axis, defl);
                T = V.Sub(V.Add(T, hingeRel), Double3Ex.Transform(hingeRel, D));
                Q = doubleQuat.Concatenate(Q, D);
            }
            var M = part.ComputeMatrixAsmb2VehicleAsmb();
            var o = Double3Ex.Transform(T, M);
            double3 Axis(double3 e) => V.Norm(V.Sub(Double3Ex.Transform(V.Add(T, Double3Ex.Transform(e, Q)), M), o));
            return new FinPose(o, Axis(new double3(1, 0, 0)), Axis(new double3(0, 1, 0)), Axis(new double3(0, 0, 1)));
        }

        public static bool IsFinPart(Part p) =>
            p.Template?.Id?.Contains(Tuning.PartId, StringComparison.Ordinal) ?? false;
    }

    // ------------------------------------------------------------------ physics: fin loads every sub-step
    [HarmonyPatch]
    internal static class ApplyFinForces
    {
        static MethodBase TargetMethod() =>
            typeof(PhysicsStates).GetMethods(BindingFlags.Public | BindingFlags.Static)
                .First(m => m.Name == nameof(PhysicsStates.ComputeDerivatives) && m.GetParameters().Length == 11);

        [HarmonyPostfix]
        static void Postfix(ref BubbleOrigin origin, ref KinematicStates kinematic, ref VehicleProperties props,
                            ref PhysicsEnvironment environment, double dt, PartTree? parts, ref Disturbances __result)
        {
            try
            {
                if (environment.AtmosphericDensity <= 0f || !environment.InPhysicsRadius || environment.SpeedOfSound <= 0f) return;
                var snap = Registry.Snapshot;
                FinSet? set = null;
                for (int i = 0; i < snap.Length; i++)
                    if ((parts != null && ReferenceEquals(snap[i].ReadOnlyVehicle?.Parts, parts)) || Unsafe.AreSame(ref kinematic, ref snap[i].Kinematic))
                    { Registry.Sets.TryGetValue(snap[i], out set); break; }
                if (set == null) return;

                var atm = environment.ClosestParent.GetPhysicalAtmosphere();
                if (atm == null) return;
                var air = PhysicsStates.ComputeAirVelocityBody(in origin, in kinematic, in environment);
                var vel = new double3(air.X, air.Y, air.Z);          // vehicle velocity through the air, body frame
                var speed = V.Len(vel);
                if (speed < 1.0) return;
                var flow = V.Mul(vel, -1.0 / speed);                   // direction the air moves past the vehicle
                var q = 0.5 * environment.AtmosphericDensity * speed * speed;
                double temp = environment.AtmosphericTemperature;
                var law = new LocalInclinationLaw((float)(speed / environment.SpeedOfSound), (float)atm.Mixture.ComputeGamma(temp));
                var friction = new SkinFrictionLaw(atm.Mixture, temp, environment.AtmosphericDensity, speed, null);
                var com = props.CenterOfMassAsmb;
                var comD = new double3(com.X, com.Y, com.Z);
                var mass = Math.Max(props.TotalMass, 1.0f);

                foreach (var fin in set.Fins)
                {
                    var pose = fin.Pose;
                    var sf = fin.Flow;
                    if (pose == null || sf == null) continue;
                    var flowLocal = pose.ToLocal(flow);
                    SurfaceLoads loads;
                    lock (sf) { sf.Update(flowLocal); loads = law.ComputeLoads(sf, in friction, flowLocal, Vector3.Zero, Vector3.Zero); }
                    var F = V.Mul(pose.ToBody(loads.ForcePerDynamicPressure), q);
                    var Mo = V.Mul(pose.ToBody(loads.MomentPerDynamicPressure), q);
                    // KSA's large-timestep drag limiter, same form as for the vehicle's own drag
                    var limiter = 1.0 / (1.0 + Math.Max(V.Dot(F, flow), 0.0) * Math.Max(dt, 0.0) / (mass * speed));
                    __result.AddForceBody(V.Sub(pose.Origin, comD), V.Mul(F, limiter));
                    __result.AddPureTorqueBody(V.Mul(Mo, limiter));
                    fin.LastForce = V.Mul(F, limiter);
                }
            }
            catch (Exception e) { Log.ErrorOnce("forces", e.ToString()); }
        }
    }

    // ------------------------------------------------------------------ keep fins out of the vehicle mesh
    [HarmonyPatch(typeof(AeroMeshSource), nameof(AeroMeshSource.Gather))]
    internal static class ExcludeFinsFromVehicleAero
    {
        static readonly MethodInfo HashFn = AccessTools.Method(typeof(AeroMeshSource), "ComputeHash");
        internal static readonly ConditionalWeakTable<AeroMeshSource, PartTree> TreeOf = new();
        static bool _logged;

        [HarmonyPrefix]
        static bool Prefix(PartTree tree, ref AeroMeshSource __result)
        {
            try
            {
                var builder = new AeroShapeSetBuilder();
                var names = new List<string>();
                int skipped = 0;
                var parts = tree.Parts;
                for (int i = 0; i < parts.Length; i++)
                {
                    var part = parts[i];
                    if (part.IsSubPart) continue;
                    if (FinAero.IsFinPart(part)) { skipped++; continue; }
                    int idx = names.Count;
                    names.Add(part.Template.Id);
                    var cols = part.SubtreeModules.Get<ColliderModule>();
                    for (int j = 0; j < cols.Length; j++)
                    {
                        var c = cols[j];
                        Decompose(c.Parent.ComputeMatrixAsmb2VehicleAsmb(), out var orient, out var pos);
                        var o2 = Quaternion.Concatenate(c.Collider2PartAsmb.ToBepu(), orient);
                        var p2 = pos + Vector3.Transform(c.PositionPartAsmb.ToBepu(), orient);
                        builder.TryAdd(c.ShapeTemplate, (float)c.Scale, o2, p2, idx);
                    }
                }
                var shapes = builder.Build();
                __result = new AeroMeshSource(shapes, names.ToArray(), (ulong)HashFn.Invoke(null, new object[] { shapes })!);
                TreeOf.AddOrUpdate(__result, tree);
                if (skipped > 0 && !_logged) { _logged = true; Log.Info($"vehicle aero mesh: {skipped} grid fin part(s) handled by the fin aero model"); }
                return false;
            }
            catch (Exception e) { Log.ErrorOnce("gather-aero", e.ToString()); return true; }
        }

        static void Decompose(in double4x4 m, out Quaternion orientation, out Vector3 position)
        {
            var x = Vector3.Normalize(new Vector3((float)m.M11, (float)m.M12, (float)m.M13));
            var y = Vector3.Normalize(new Vector3((float)m.M21, (float)m.M22, (float)m.M23));
            var z = Vector3.Normalize(new Vector3((float)m.M31, (float)m.M32, (float)m.M33));
            var r = new Matrix4x4(x.X, x.Y, x.Z, 0f, y.X, y.Y, y.Z, 0f, z.X, z.Y, z.Z, 0f, 0f, 0f, 0f, 1f);
            orientation = Quaternion.Normalize(Quaternion.CreateFromRotationMatrix(r));
            position = new Vector3((float)m.M41, (float)m.M42, (float)m.M43);
        }
    }

    // ------------------------------------------------------------------ "Draw Aerodynamics" overlay for the fins
    [HarmonyPatch(typeof(AeroMeshDebugDraw), nameof(AeroMeshDebugDraw.Draw))]
    internal static class DrawFinAero
    {
        static readonly ConditionalWeakTable<Part, AeroMeshDebugDraw> Drawers = new();
        [ThreadStatic] static bool _inside;

        [HarmonyPostfix]
        static void Postfix(ImDrawListPtr drawList, Aerodynamics? aerodynamics, in double4x4 matrixAsmb2Ego, Camera camera,
                            float2 viewportPos, bool drawNormals, AeroFreestream? freestream)
        {
            if (_inside || aerodynamics?.Source == null) return;
            try
            {
                if (!ExcludeFinsFromVehicleAero.TreeOf.TryGetValue(aerodynamics.Source, out var tree)) return;
                var finAero = FinAero.Mesh;
                if (finAero == null) return;
                _inside = true;
                var parts = tree.Parts;
                for (int i = 0; i < parts.Length; i++)
                {
                    var part = parts[i];
                    if (part.IsSubPart || !FinAero.IsFinPart(part)) continue;
                    Fin? fin = FindFin(part);
                    var pose = FinAero.ComputePose(part, fin);
                    if (pose == null) continue;
                    var finToVeh = Matrix(pose);
                    var m = finToVeh * matrixAsmb2Ego;
                    AeroFreestream? fs = null;
                    if (freestream is { } f)
                    {
                        var d = pose.ToLocal(new double3(f.Direction.X, f.Direction.Y, f.Direction.Z));
                        fs = new AeroFreestream(d, f.Law);
                    }
                    var drawer = Drawers.GetValue(part, _ => new AeroMeshDebugDraw());
                    drawer.Draw(drawList, finAero, in m, camera, viewportPos, drawNormals, fs);
                }
            }
            catch (Exception e) { Log.ErrorOnce("draw", e.ToString()); }
            finally { _inside = false; }
        }

        static Fin? FindFin(Part part)
        {
            foreach (var f in Registry.FinObjs.Values)
                if (f.Module?.Parent == part) return f;
            return null;
        }

        // row-vector convention (as KSA's Decompose: rows = basis, row 4 = translation)
        static double4x4 Matrix(FinPose p)
        {
            var m = double4x4.Identity;
            m.M11 = p.Ex.X; m.M12 = p.Ex.Y; m.M13 = p.Ex.Z;
            m.M21 = p.Ey.X; m.M22 = p.Ey.Y; m.M23 = p.Ey.Z;
            m.M31 = p.Ez.X; m.M32 = p.Ez.Y; m.M33 = p.Ez.Z;
            m.M41 = p.Origin.X; m.M42 = p.Origin.Y; m.M43 = p.Origin.Z;
            return m;
        }
    }
}
