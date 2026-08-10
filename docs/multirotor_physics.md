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
3. For **PX4 / MAVLink** vehicles, AirSim publishes `BATTERY_STATUS` (~5 Hz) on the **control** MAVLink link (not the HIL sensor socket), using plant SOC, voltage, current, and consumed mAh. Packets are sent as **sysid = that vehicle’s PX4 `MAV_SYS_ID`**, **compid = 191** (onboard companion). PX4 only accepts external battery when `sysid` matches the vehicle and `compid` is not the autopilot.

### Multi-vehicle PX4 + GCS

| Concern | Behavior |
|---------|----------|
| Distinguish drones in QGC / GCS | Each PX4 SITL instance uses `MAV_SYS_ID = instance + 1` (1, 2, 3…). Different `TcpPort` / control ports per vehicle. GCS lists separate vehicles by **sysid**. |
| AirSim vehicle names | Settings keys (`Drone1`, `Drone2`) are independent of MAVLink sysid; map ports carefully. |
| Battery per drone | Each AirSim vehicle has its own plant battery and its own MAVLink connection; battery is published with **that instance’s target sysid** (from the PX4 heartbeat), not a hardcoded `1`. |
| Failsafe params | Set `Parameters` (or per-vehicle params) on each vehicle; each PX4 instance applies its own `COM_LOW_BAT_ACT` / `BAT_*_THR`. |

Example (two vehicles — ports and spawn only; sysids come from PX4 instances 0 and 1):

```json
"Vehicles": {
  "Drone1": {
    "VehicleType": "PX4Multirotor",
    "UseTcp": true, "TcpPort": 4560,
    "ControlPortLocal": 14540, "ControlPortRemote": 14580,
    "Physics": { "EnableBattery": true, "BatteryCapacityMah": 5000 },
    "Parameters": { "SIM_BAT_ENABLE": 0, "COM_LOW_BAT_ACT": 3, "BAT_CRIT_THR": 0.07 }
  },
  "Drone2": {
    "VehicleType": "PX4Multirotor",
    "UseTcp": true, "TcpPort": 4561,
    "ControlPortLocal": 14541, "ControlPortRemote": 14581,
    "Y": 2,
    "Physics": { "EnableBattery": true, "BatteryCapacityMah": 5000 },
    "Parameters": { "SIM_BAT_ENABLE": 0, "COM_LOW_BAT_ACT": 3, "BAT_CRIT_THR": 0.07 }
  }
}
```

See also [multi-vehicle PX4](px4_multi_vehicle.md).

Example:

```json
"Physics": {
  "EnableBattery": true,
  "BatteryCapacityMah": 5000,
  "BatteryMaxVoltage": 16.8,
  "BatteryMinVoltage": 13.2
}
```

### PX4 battery failsafe

AirSim publishes plant battery over MAVLink. **Failsafe action is configured in PX4**, typically via vehicle `Parameters` in `settings.json` (sent at connect) or QGC.

Recommended SITL values (PX4 with `COM_LOW_BAT_ACT` / `BAT_*_THR` — verified against common PX4 SITL builds):

```json
"Parameters": {
  "SIM_BAT_ENABLE": 0,
  "CBRK_SUPPLY_CHK": 894281,
  "COM_LOW_BAT_ACT": 3,
  "BAT_LOW_THR": 0.15,
  "BAT_CRIT_THR": 0.07,
  "BAT_EMERGEN_THR": 0.05,
  "COM_ARM_BAT_MIN": 0.05,
  "COM_FAIL_ACT_T": 1.0,
  "COM_ARM_WO_GPS": 1,
  "COM_RCL_EXCEPT": 7,
  "NAV_RCL_ACT": 0,
  "NAV_DLL_ACT": 0
}
```

| Param | Meaning |
|-------|---------|
| `SIM_BAT_ENABLE` | `0` = do not use PX4’s internal SITL battery (use AirSim `BATTERY_STATUS`) |
| `COM_LOW_BAT_ACT` | `0` = warn only; `2` = land; **`3` = RTL at critical, land at emergency** |
| `BAT_LOW_THR` | Remaining fraction for “low” warning (e.g. `0.15` = 15%) |
| `BAT_CRIT_THR` | Critical fraction (triggers return when action = 3) |
| `BAT_EMERGEN_THR` | Emergency fraction (land in place when action = 3) |
| `CBRK_SUPPLY_CHK` | `894281` disables power-module arming check (common SITL need) |

Example profile: `settings.px4-battery-test.json` (local / gitignored).

WSL2: set `PX4_SIM_HOST_ADDR` to the Windows vEthernet (WSL) IPv4 and AirSim `LocalHostIp` to the same address. Start AirSim first, then PX4 `none_iris`.

SimpleFlight does not use MAVLink; battery only scales plant thrust.

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
