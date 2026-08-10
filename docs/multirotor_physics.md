# AirSim multirotor physics settings

Vehicle plant parameters can be set under each multirotor entry in `settings.json` via a `Physics` object. Omitted fields keep the frame defaults (Generic, Blacksheep, Flamewheel, Hex, Octo).

For live PX4 with ROS 2 vision, use `"ClockType": "ScalableClock"`, `"ClockSpeed": 1`, and `"LockStep": false`. Physics settings do not depend on lockstep.

## Vehicle `Physics` block

```json
"drone1": {
  "VehicleType": "PX4Multirotor",
  "Physics": {
    "Mass": 1.0,
    "MotorAssemblyWeight": 0.055,
    "ArmLength": 0.2275,
    "RotorZ": 0.025,
    "BodyBox": { "X": 0.18, "Y": 0.11, "Z": 0.04 },
    "LinearDragCoefficient": 0.325,
    "AngularDragCoefficient": 0.02,
    "Restitution": 0.55,
    "Friction": 0.5,

    "RotorCT": 0.109919,
    "RotorCP": 0.040164,
    "RotorMaxRpm": 6396.667,
    "PropellerDiameter": 0.2286,
    "PropellerHeight": 0.01,
    "ControlSignalFilterTC": 0.005,

    "EnableGroundEffect": true,
    "GroundEffectMaxHeight": 2.0,
    "GroundEffectMaxBoost": 0.25,
    "GroundEffectMinHeight": 0.02,

    "EnableThrustAirSpeed": true,
    "ThrustAirSpeedCoeff": 0.15,
    "ThrustAirSpeedMinScale": 0.35,

    "EnableBattery": false,
    "BatteryCapacityMah": 5000,
    "BatteryMaxVoltage": 16.8,
    "BatteryMinVoltage": 13.2,
    "BatteryInternalResistance": 0.015,
    "BatteryIdleCurrent": 0.3,
    "BatteryMaxCurrentPerMotor": 25,

    "InertiaDiagonal": { "X": 0.01, "Y": 0.01, "Z": 0.02 }
  }
}
```

### Hover check

If `max_thrust * rotor_count < mass * g * 1.05`, AirSim logs a warning. Raise `RotorMaxRpm` / `RotorCT` or lower `Mass`.

### Ground effect AGL

AGL uses the flat-world proxy `max(0, -NED.z)` with the origin on the ground plane.

## Wind turbulence

```json
"Wind": { "X": 2, "Y": 0, "Z": 0 },
"WindTurbulence": {
  "Enabled": true,
  "Sigma": 1.5,
  "Tau": 2.0
}
```

Mean wind plus first-order Gauss–Markov turbulence is applied in `FastPhysicsEngine`.

## Angular drag

`AngularDragCoefficient` applies body-frame quadratic drag `τ = -c · ρ · ω|ω|`. Default `0.02`. Set `0` to disable.

## Sensors

### GPS

```json
"gps": {
  "SensorType": 3,
  "Enabled": true,
  "GenerateNoise": true,
  "PositionSigmaScale": 0.5,
  "VelocitySigma": 0.1,
  "BiasSigma": 0.5,
  "BiasTau": 60
}
```

### IMU

```json
"imu": {
  "SensorType": 2,
  "Enabled": true,
  "GenerateNoise": true,
  "EnableVibration": true,
  "VibrationAccelSigma": 0.5,
  "VibrationGyroSigma": 0.02,
  "VibrationAngularGain": 0.15
}
```

`GenerateNoise` defaults to true for the IMU. Set `false` for clean IMU output.

## Battery

When `EnableBattery` is true:

1. Thrust scales approximately with `(V/Vmax)²` and SOC drains from motor demand.
2. Values appear in the physics state reporter (`Batt SOC`, `Batt V`, `Batt I`).
3. For **PX4 / MAVLink** vehicles, AirSim publishes `BATTERY_STATUS` (~5 Hz) on the **control** MAVLink link (not the HIL sensor socket), using plant SOC, voltage, current, and consumed mAh.

Example:

```json
"Physics": {
  "EnableBattery": true,
  "BatteryCapacityMah": 5000,
  "BatteryMaxVoltage": 16.8,
  "BatteryMinVoltage": 13.2
}
```

### PX4 failsafe (RTL on low battery)

AirSim only feeds telemetry. PX4 still needs failsafe parameters, for example low-battery action = RTL/Land. PX4 SITL often runs its **own** battery model; if QGC still shows a different pack, disable or override the SITL battery simulation for your PX4 version so external `BATTERY_STATUS` is used. Param names vary by PX4 release — check `BAT_*` / simulation battery docs for your build.

SimpleFlight does not use MAVLink; battery still affects plant thrust only.

## Code map

| Feature | Location |
|---------|----------|
| Settings schema | `AirLib/include/common/MultirotorPhysicsConfig.hpp`, `AirSimSettings.hpp` |
| Apply to frame | `MultiRotorParams::applyPhysicsConfig` |
| Rotors | `RotorActuator.hpp` |
| Body / battery | `MultiRotorPhysicsBody.hpp` |
| Angular drag + turbulence | `FastPhysicsEngine.hpp` |
| GPS noise | `GpsSimple.hpp` |
| IMU vibration | `ImuSimple.hpp` |
