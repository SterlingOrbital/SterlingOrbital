# SterlingFlight (StarMap code mod, part of Sterling Orbital)

`SterlingFlight.dll` is the code half of the Sterling Orbital mod for Kitten Space Agency: grid fin aerodynamics,
landing leg suspension, octaweb engine sets, the Sparrow mission autopilot (RTLS and drone ship landings), the
autonomous drone ship, the Mission Control panel and webcast overlay, soot effects and editor thumbnail fixes.
It loads through the StarMap mod loader (`[StarMap] EntryAssembly = "SterlingFlight"` in `mod.toml`).

## Build

Needs the .NET 10 SDK, a Kitten Space Agency install and StarMap.

```
dotnet build -c Release
```

The project compiles against the game's and StarMap's assemblies; nothing from them is copied into the mod.
Paths are MSBuild properties with Windows defaults (see `SterlingFlight.csproj`); override them when yours differ:

```
dotnet build -c Release -p:GameDir="D:\Games\Kitten Space Agency" -p:StarMapDir="D:\StarMap"
```

| Property | Default | Meaning |
|---|---|---|
| `GameDir` | `C:\Program Files\Ahwoo\Games\Kitten Space Agency` | folder holding `KSA.dll` |
| `StarMapDir` | `%USERPROFILE%\Documents\KSA_StarMap` | folder holding `StarMap.API.dll` and `0Harmony.dll` |
| `ModOut` | `%USERPROFILE%\Documents\My Games\Kitten Space Agency\mods\SterlingOrbital` | where the DLL and `mod.toml` are copied after the build |
| `DeployMod` | `true` | `false` skips the copy |

Close the game before building: it locks the DLL. The parts, models and textures are not in this folder; take them
from the release zip.

At run time the mod finds its own folder through the game's mod library (`ModPaths` in `SterlingFlight.cs`), so it
works wherever a mod manager installs it. Log: `SterlingFlight.log` in the mod folder.

## Grid fins (SterlingFlight.cs, FinAero.cs)

KSA has no aerodynamic control surfaces, so the grid fins (`SparrowGridFinPart`, animation module
`SparrowGridFinDeploy`) get them here.

- Postfix `VehicleUpdateState.UpdateActiveChutes` (per vehicle update): collects the grid fins, their deploy
  fraction, CP / span / cell axes in the body frame (vehicle assembly axes, origin at the CoM, like
  `ActiveChute.AttachBody`) and the pilot rotation request, built like `FlightComputer.ComputeTvcControl` manual
  mode (flags 2/4, 8/16, 32/64 -> +/-X/Y/Z, then `Ctrl2Asmb`).
- Postfix static `PhysicsStates.ComputeDerivatives` (every physics sub-step): matches the vehicle by
  `Unsafe.AreSame(ref kinematic, ref vus.Kinematic)`, computes q from `PhysicsEnvironment.AtmosphericDensity` and
  `ComputeAirVelocityBody`, picks each fin's deflection from the torque it would give at +/-20 deg (so it works
  nose-first or tail-first), slews at 60 deg/s and adds normal force + lattice drag with
  `Disturbances.AddForceBody`, scaled by KSA's own drag limiter `1/(1 + F*dt/(m*v))`.
- Tuning constants are in `Tuning` (area 2 m^2, CN 1.8 per sin(alpha), CD 0.8, max 20 deg, gain 1.5).

Visual: a prefix on `KeyframeAnimationModule.UpdateModules` keeps the fin animations' private `_transformsDirty`
set while a deflection is shown, and a postfix on `ApplyAnimationTransforms` turns the fin subpart about its span
axis through the hinge. Same sign convention as the force model, so what you see is what pushes.

## Landing legs (LandingLegs.cs)

Suspension for `SparrowLandingLegPart`: a spring-damper at each deployed leg's crush pad along the local vertical,
plus viscous ground friction capped at 0.9 x normal load. The part's foot-stop collider is the hard stop; the
spring works over `Travel` above it. Right-click a leg for sliders (Spring kN/m, Compression damping, Rebound
damping, Travel), saved in `landing_legs.json` in the mod folder. Defaults per leg: 300 kN/m, 60 / 90 kN*s/m,
0.6 m. Terrain only: on structures (pads) the foot stop lands rigidly.

## Engine sets (EngineSets.cs)

The octaweb's nine engines are three engine controllers (centre, opposite pair, outer six). Right-click the
octaweb to pick the 9 / 3 / 1 engine set; staging moves all three groups together.

## Autopilot, drone ship, panel (Autopilot.cs, DroneShip.cs, MissionPanel.cs, SterlingMenu.cs, Webcast.cs)

One button flies launch, MECO, stage separation, flip, boostback (RTLS) or entry burn (drone ship), grid fin
descent, legs, landing burn and touchdown, while the upper stage flies itself to orbit. Steering goes through
KSA's own flight computer. The drone ship is spawned at sea level downrange on the launch azimuth and holds
station. Open the panel from the top bar: `Sterling Orbital > Sparrow Mission Control`.

## Editor thumbnails (ThumbnailFix.cs)

KSA's `ThumbnailPart` keeps one `PartModel` + one `PartModelDynamic` per (sub)part, so parts with several models
in one SubPart showed a single fragment. Postfixes on `ThumbnailCreator.CollectDraws` and
`ThumbnailPart.ComputeBoundingSphereRadius` fix it.

## License

MIT, see `LICENSE` at the repository root.
