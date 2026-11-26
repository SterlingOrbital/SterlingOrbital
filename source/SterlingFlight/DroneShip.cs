// Autonomous drone ship for Sparrow drone ship landings (part "SterlingDroneShipPart").
//
// Spawn: LAUNCH (DRONESHIP) asks for a ship; it is created on the main thread (InputEvents.ApplyInputEvents
// postfix) the way the editor launches a vehicle - new Part(template) -> CreateOwnTree -> Vehicle.CreateVehicle -
// but at sea level instead of on the terrain (the stock helper would put it on the seafloor). It sits on the
// launch azimuth, Cfg.ShipKm downrange of the pad. If a ship already exists (relaunch, or a save that has one)
// it is moved there with Vehicle.Teleport instead of making another.
//
// Station keeping (PhysicsStates.ComputeDerivatives postfix, every physics sub-step): KSA's buoyancy is a
// bounding-sphere immersion fraction pushing at the centre of mass, so a barge has no righting moment and bobs on
// a soft spring. Like the real ship's thrusters, this cancels that (feed-forward gravity minus buoyancy), holds
// the deck at a fixed freeboard, keeps the deck level and damps drift - so a booster landing off-centre does not
// tip it. Part space: +X = deck normal, origin = centre of the landing target on the deck surface.
using System.Reflection;
using System.Runtime.CompilerServices;
using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace SterlingFlight
{
    internal static class DroneShip
    {
        public const string PartId = "SterlingDroneShipPart";
        public const string VehicleId = "It Was Never Goodbye";
        public const double Freeboard = 3.0;              // deck height above the mean sea level (m)

        public static Vehicle? Ship;
        public static volatile bool WantSpawn;
        static double _distKm, _azDeg;
        static double3 _padCcf;
        public static double3 DeckTargetCcf;              // where the deck centre was put (planet fixed)

        public static bool Alive => Ship != null && !Ship.IsDisposed;

        public static bool IsShip(Vehicle v) => Autopilot.HasPart(v, PartId);

        /// remember a ship found in the world (e.g. after loading a save)
        public static void Notice(Vehicle v)
        {
            if (!Alive && IsShip(v)) { Ship = v; Log.Info("drone ship found: " + v.Id); }
        }

        public static void Request(double3 padCcf, double distKm, double azDeg)
        {
            _padCcf = padCcf; _distKm = distKm; _azDeg = azDeg; WantSpawn = true;
        }

        /// main thread, between frames
        public static void ApplyPending()
        {
            if (!WantSpawn) return;
            WantSpawn = false;
            try { Spawn(); }
            catch (Exception e) { Log.Error("drone ship spawn failed: " + e); Autopilot.Event("drone ship could not be placed: " + e.Message); }
        }

        static void Spawn()
        {
            var launcher = Autopilot.Stack ?? Program.ControlledVehicle;
            if (launcher?.Orbit?.Parent is not Celestial cel) throw new Exception("no planet");
            var time = Universe.GetElapsedTime();
            // point on the sea, distKm along the launch azimuth (great circle) from the pad
            var up = V.Norm(_padCcf);
            var axis = new double3(0, 0, 1);                                      // CCF z = north pole (GetDirCcfFromLatLon)
            var east = V.Norm(V.Cross(axis, up)); var north = V.Cross(up, east);
            double az = _azDeg * Math.PI / 180, ang = _distKm * 1000 / cel.MeanRadius;
            var hor = V.Add(V.Mul(north, Math.Cos(az)), V.Mul(east, Math.Sin(az)));
            var dir = V.Norm(V.Add(V.Mul(up, Math.Cos(ang)), V.Mul(hor, Math.Sin(ang))));
            double lat = Math.Asin(Math.Clamp(dir.Z, -1, 1)) * 180 / Math.PI, lon = Math.Atan2(dir.Y, dir.X) * 180 / Math.PI;
            double sea = SeaRadius(cel, dir);

            Part root;
            if (Alive) root = Ship!.Parts.Root;
            else
            {
                var tmpl = ModLibrary.Get<PartTemplate>(PartId);
                root = new Part(tmpl.Id, tmpl);
                root.CreateOwnTree().ReinitializeDerivedValues();
            }
            var tree = root.Tree;
            var mp = tree.ComputeInertMassPropertiesAsmb();
            var com = new double3(mp.Offset.X, mp.Offset.Y, mp.Offset.Z);
            var (bmin, bmax) = tree.ComputeSpawnBoundingBoxAsmb();
            var st = Vehicle.GetInitialKinematicStateForLocation(cel, time, lat, lon, bmin, bmax, com, new byte4(40, 200, 255, 255));
            // body +X is the local vertical here; the vehicle's position is its centre of mass
            var deckCcf = V.Mul(dir, sea + Freeboard);
            var comCcf = V.Add(deckCcf, V.Mul(dir, com.X - root.PositionVehicleAsmb.X));
            var ccf2cci = cel.GetCcf2Cci(time);
            var posCci = Double3Ex.Transform(comCcf, ccf2cci);
            var velCci = V.Cross(((IParentBody)cel).GetAngularVelocityCci(), posCci);
            var orbit = Orbit.CreateFromStateCci(cel, time, posCci, velCci, new byte4(40, 200, 255, 255));
            DeckTargetCcf = deckCcf; DeckRadius = sea + Freeboard;
            DroneShipStationKeeping._iz = double.NaN;
            Log.Info($"drone ship: water surface {sea - cel.MeanRadius:+0.0;-0.0} m from the mean radius here");
            if (Alive)
            {
                InputEvents.TeleportInputBuffer.Add(new InputEvents.TeleportInputData { Vehicle = Ship!, Orbit = orbit, Body2Cce = st.Body2Cce, BodyRates = st.BodyRates });
                Log.Info($"drone ship moved to {lat:0.0000}, {lon:0.0000} ({_distKm:0} km at {_azDeg:0.0} deg)");
            }
            else
            {
                Ship = Vehicle.CreateVehicle(Universe.CurrentSystem, st.Body2Cce, st.BodyRates, cel, VehicleId, root, orbit);
                orbit.Parent.Children.Add(Ship);
                Ship.UpdatePerFrameData();
                Log.Info($"drone ship spawned at {lat:0.0000}, {lon:0.0000} ({_distKm:0} km at {_azDeg:0.0} deg), {tree.ComputeInertMassPropertiesAsmb().Props.Mass / 1000:0} t");
            }
            Autopilot.Event($"drone ship on station {_distKm:0} km downrange");
        }

        /// the water surface the game's buoyancy uses (renderer height incl. local offsets; flight 2: ~24 m below the mean radius)
        public static double SeaRadius(Celestial cel, double3 dirCcf)
        {
            try
            {
                var oc = Program.GetOceanRenderer();
                if (oc != null) { double h = oc.GetOceanHeightAtPositionCcf(cel, dirCcf); if (double.IsFinite(h) && Math.Abs(h) < 5000) return cel.MeanRadius + h; }
            }
            catch { }
            return cel.MeanRadius + OceanLevel(cel);
        }

        public static double DeckRadius;                  // live: water surface + freeboard, kept by the station keeping

        public static double OceanLevel(Celestial cel)
        {
            var oc = cel.BodyTemplate?.OceanReference;
            return oc == null ? 0.0 : (double)oc.Level;
        }

        /// centre of the landing target on the deck, CCI. Horizontal position = the planet-fixed station the ship is
        /// held on (a parked vehicle's inertial position goes stale while the planet turns: flight 1 aimed 7 km off).
        /// Height = the live deck height when the ship is there, so a ship riding high or low is still met right.
        public static double3 DeckCci(Nav n)
        {
            var station = n.Body is Celestial cel ? Double3Ex.Transform(DeckTargetCcf, cel.GetCcf2Cci()) : DeckTargetCcf;
            if (DeckRadius > 0) station = V.Mul(V.Norm(station), DeckRadius);
            if (Alive && Ship!.Orbit?.Parent != null)
            {
                var s = Ship!;
                var c = s.FlightComputer.CenterOfMassAsmb;
                var offAsmb = V.Sub(s.Parts.Root.PositionVehicleAsmb, new double3(c.X, c.Y, c.Z));
                var live = V.Add(s.GetPositionCci(), Double3Ex.Transform(offAsmb, s.GetBody2Cci()));
                double rLive = V.Len(live);
                if (Math.Abs(rLive - V.Len(station)) < 30) return V.Mul(V.Norm(station), rLive);
            }
            return station;
        }

        internal static double LastStatusLog;
    }

    /// thrusters: hold freeboard, level and position against KSA's centre-of-mass buoyancy
    [HarmonyPatch]
    internal static class DroneShipStationKeeping
    {
        static MethodBase TargetMethod() =>
            typeof(PhysicsStates).GetMethods(BindingFlags.Public | BindingFlags.Static)
                .First(m => m.Name == nameof(PhysicsStates.ComputeDerivatives) && m.GetParameters().Length == 11);

        static double _sea;
        internal static double _iz = double.NaN;   // learned vertical accel (gravity - lift), m/s2

        [HarmonyPostfix]
        static void Postfix(ref BubbleOrigin origin, ref KinematicStates kinematic, ref VehicleProperties props,
                            ref PhysicsEnvironment environment, double dt, PartTree? parts, ref Disturbances __result)
        {
            try
            {
                var ship = DroneShip.Ship;
                if (ship == null || !environment.InPhysicsRadius) return;
                double m = Math.Max(props.TotalMass, 1.0f);
                // match the ship by its part tree, or - when the step does not hand one over - by its unique mass
                bool isShip = parts != null ? ReferenceEquals(parts, ship.Parts) : (m > 2.5e7 && m < 8e7);
                if (!isShip) return;
                if (environment.ClosestParent is not Celestial cel) return;
                parts ??= ship.Parts;
                var posBub = PhysicsStates.GetPositionBub(in origin, in kinematic);
                var rel = PhysicsStates.GetPositionClosestParentBub(in origin, in environment, posBub);
                double r = V.Len(rel); if (r < 1) return;
                var inv = QuaternionEx.Inverse(kinematic.Body2Phys);
                var up = V.Norm(Double3Ex.Transform(V.Mul(rel, 1.0 / r), inv));                 // body frame
                double g = V.Len(environment.GravitationBub);
                var air = PhysicsStates.ComputeAirVelocityBody(in origin, in kinematic, in environment);
                var vel = new double3(air.X, air.Y, air.Z);                                     // vs. the surface
                var w = PhysicsStates.GetBodyRates(in origin, in kinematic, in environment);
                var com = props.CenterOfMassAsmb;
                var deckOff = V.Sub(parts.Root.PositionVehicleAsmb, new double3(com.X, com.Y, com.Z));

                // vertical: cancel gravity - buoyancy, then a critically damped hold of the deck height
                double deckR = r + V.Dot(deckOff, up);
                double seaNow = environment.OceanRadius > 0 ? environment.OceanRadius : cel.MeanRadius + DroneShip.OceanLevel(cel);
                if (_sea <= 0 || Math.Abs(seaNow - _sea) > 200) _sea = seaNow; else _sea += (seaNow - _sea) * Math.Min(1.0, dt / 6.0);
                double target = _sea + DroneShip.Freeboard;
                DroneShip.DeckRadius = target;
                double vz = V.Dot(vel, up);
                // PD + integral. No buoyancy feed-forward: the step's OceanVolume can read 0 while the game still
                // applies ~0.5 g of lift (unfocused ship rode 36 m high, flight 2). The integral learns the real lift.
                const double wn = 0.6, ki = 0.06;
                double buoy = environment.OceanDensity * environment.OceanVolume * g / m;   // log only
                double e = target - deckR;
                if (double.IsNaN(_iz)) _iz = g * 0.5;
                if (dt > 0 && dt < 1) _iz = Math.Clamp(_iz + ki * Math.Clamp(e, -20, 20) * dt, -2 * g, 2 * g);
                double az = _iz + wn * wn * e - 2 * wn * vz;
                // horizontal: hold the planet-fixed station (slow, like azimuth thrusters) and damp drift
                var relCcf = origin.BubFrame.IsCcf() ? rel : Double3Ex.Transform(rel, cel.GetCci2Ccf());
                var eCcf = V.Sub(DroneShip.DeckTargetCcf, relCcf);
                var eBub = origin.BubFrame.IsCcf() ? eCcf : Double3Ex.Transform(eCcf, cel.GetCcf2Cci());
                var eBody = Double3Ex.Transform(eBub, inv);
                var eh = V.Sub(eBody, V.Mul(up, V.Dot(eBody, up)));
                var vh = V.Sub(vel, V.Mul(up, vz));
                var ah = V.Sub(V.Mul(eh, 0.04), V.Mul(vh, 0.5));
                double ahm = V.Len(ah); if (ahm > 0.6) ah = V.Mul(ah, 0.6 / ahm);
                var a = V.Add(V.Mul(up, Math.Clamp(az, -20, 20)), ah);
                __result.AddForceBody(double3.Zero, V.Mul(a, m));
                double now = Autopilot.Now;
                if (now - DroneShip.LastStatusLog > 10)
                {
                    DroneShip.LastStatusLog = now;
                    Log.Info($"ship: deck {deckR - (target - DroneShip.Freeboard):0.0} m above sea (want {DroneShip.Freeboard:0.0}), {V.Len(eh):0.0} m off station, " +
                             $"buoyancy {buoy / Math.Max(g, 0.1):0.00} g, hold {_iz / Math.Max(g, 0.1):0.00} g, vz {vz:0.00} m/s, {m / 1000:0} t");
                }

                // attitude: deck normal (+X) to the local vertical, damp all rates (yaw only damped)
                var err = V.Cross(new double3(1, 0, 0), up);
                var rates = new double3(w.X, w.Y, w.Z);
                const double wa = 0.8;
                // box inertia of a 91 x 40 x 8 m barge, body axes X up, Y along the hull, Z across
                double ix = m * (91.0 * 91.0 + 40.0 * 40.0) / 12, iy = m * (40.0 * 40.0 + 8.0 * 8.0) / 12, iz = m * (91.0 * 91.0 + 8.0 * 8.0) / 12;
                var alpha = V.Sub(V.Mul(err, wa * wa), V.Mul(rates, 2 * wa));
                __result.AddPureTorqueBody(new double3(alpha.X * ix, alpha.Y * iy, alpha.Z * iz));
            }
            catch (Exception e) { Log.ErrorOnce("shipkeep", e.ToString()); }
        }
    }
}
