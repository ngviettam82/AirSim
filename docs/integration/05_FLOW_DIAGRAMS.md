# Implementation Flow Diagrams

## Visual Guides for ProjectAirSim Feature Integration

---

## 1. Sensor Data Flow Architecture

### Current Cosys-AirSim Sensor Flow

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                     SENSOR DATA FLOW (Current)                               │
└─────────────────────────────────────────────────────────────────────────────┘

  UNREAL ENGINE                    AIRLIB                         PYTHON CLIENT
  ═══════════════                  ══════                         ═════════════

  ┌─────────────┐
  │ World State │
  │ (Position,  │
  │  Rotation,  │
  │  Collisions)│
  └──────┬──────┘
         │
         ▼
  ┌─────────────┐              ┌─────────────────┐
  │ UE Sensor   │◄────────────►│ SensorBase      │
  │ Components  │              │ (Abstract)      │
  │ - Camera    │              └────────┬────────┘
  │ - LiDAR     │                       │
  │ - Echo      │                       ▼
  └─────────────┘              ┌─────────────────┐
                               │ SensorFactory   │
                               │ createSensor()  │
                               └────────┬────────┘
                                        │
                                        ▼
                               ┌─────────────────┐
                               │ Specific Sensor │
                               │ getOutput()     │
                               └────────┬────────┘
                                        │
                                        ▼
                               ┌─────────────────┐
                               │ VehicleApiBase  │
                               │ getSensorData() │
                               └────────┬────────┘
                                        │
                                        ▼
                               ┌─────────────────┐         ┌─────────────┐
                               │ RpcLibServer    │────────►│ RPC Network │
                               │ bind()          │         │ (msgpack)   │
                               └─────────────────┘         └──────┬──────┘
                                                                  │
                                                                  ▼
                                                           ┌─────────────┐
                                                           │ client.py   │
                                                           │ getSensor() │
                                                           └─────────────┘
```

### New Sensor Integration Flow

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                     NEW SENSOR INTEGRATION FLOW                              │
└─────────────────────────────────────────────────────────────────────────────┘

  Step 1: Create Sensor Files
  ═══════════════════════════

  AirLib/include/sensors/
          │
          ├── battery/
          │   ├── BatterySensorBase.hpp    ◄── Define abstract interface
          │   ├── BatterySensorSimple.hpp  ◄── Implement sensor logic
          │   └── BatterySensorParams.hpp  ◄── Define configuration
          │
          ├── airspeed/
          │   ├── AirspeedSensorBase.hpp
          │   ├── AirspeedSensorSimple.hpp
          │   └── AirspeedSensorParams.hpp
          │
          └── radar/
              ├── RadarSensorBase.hpp
              ├── RadarSensorSimple.hpp
              └── RadarSensorParams.hpp


  Step 2: Register Sensor Type
  ════════════════════════════

  SensorBase.hpp:
  ┌─────────────────────────────────────────┐
  │ enum SensorType {                       │
  │     Barometer = 1,                      │
  │     Imu = 2,                            │
  │     Gps = 3,                            │
  │     ...                                 │
  │     Battery = 12,    ◄── ADD           │
  │     Airspeed = 13,   ◄── ADD           │
  │     Radar = 14       ◄── ADD           │
  │ };                                      │
  └─────────────────────────────────────────┘


  Step 3: Add Settings Parsing
  ════════════════════════════

  AirSimSettings.hpp:
  ┌─────────────────────────────────────────────────────┐
  │ struct BatterySetting {                             │
  │     float max_capacity_wh = 100.0f;                │
  │     float nominal_voltage = 22.2f;                 │
  │     ...                                            │
  │ };                                                 │
  │                                                    │
  │ BatterySetting loadBatterySetting(Settings& s) {   │
  │     BatterySetting bs;                             │
  │     bs.max_capacity_wh = s.getFloat("MaxCapacity");│
  │     ...                                            │
  │     return bs;                                     │
  │ }                                                  │
  └─────────────────────────────────────────────────────┘


  Step 4: Update Sensor Factory
  ═════════════════════════════

  SensorFactory.hpp:
  ┌─────────────────────────────────────────────────────┐
  │ switch(sensor_type) {                              │
  │     case SensorType::Battery:                      │
  │         return createBatterySensor(settings);      │
  │     case SensorType::Airspeed:                     │
  │         return createAirspeedSensor(settings);     │
  │     case SensorType::Radar:                        │
  │         return createRadarSensor(settings);        │
  │ }                                                  │
  └─────────────────────────────────────────────────────┘


  Step 5: Add Vehicle API Methods
  ═══════════════════════════════

  VehicleApiBase.hpp:
  ┌─────────────────────────────────────────────────────┐
  │ virtual BatteryData getBatteryData(string name);   │
  │ virtual void setBatteryRemaining(float pct);       │
  │ virtual AirspeedData getAirspeedData(string name); │
  │ virtual RadarData getRadarData(string name);       │
  └─────────────────────────────────────────────────────┘


  Step 6: Create RPC Adaptors
  ═══════════════════════════

  RpcLibAdaptorsBase.hpp:
  ┌─────────────────────────────────────────────────────┐
  │ struct BatteryData {                               │
  │     float voltage;                                 │
  │     float current;                                 │
  │     float remaining_percent;                       │
  │     ...                                            │
  │     MSGPACK_DEFINE_MAP(voltage, current, ...);     │
  │ };                                                 │
  └─────────────────────────────────────────────────────┘


  Step 7: Bind RPC Methods
  ════════════════════════

  RpcLibServerBase.cpp:
  ┌─────────────────────────────────────────────────────┐
  │ server_->bind("getBatteryData", [&](string name) { │
  │     return RpcAdaptors::BatteryData(               │
  │         vehicle_api_->getBatteryData(name)         │
  │     );                                             │
  │ });                                                │
  └─────────────────────────────────────────────────────┘


  Step 8: Add Python Client Methods
  ═════════════════════════════════

  types.py:
  ┌─────────────────────────────────────────────────────┐
  │ class BatteryData(MsgpackMixin):                   │
  │     voltage = 0.0                                  │
  │     current = 0.0                                  │
  │     remaining_percent = 100.0                      │
  │     ...                                            │
  └─────────────────────────────────────────────────────┘

  client.py:
  ┌─────────────────────────────────────────────────────┐
  │ def getBatteryData(self, vehicle_name=''):         │
  │     return BatteryData.from_msgpack(               │
  │         self.client.call('getBatteryData', name)   │
  │     )                                              │
  └─────────────────────────────────────────────────────┘
```

---

## 2. Battery Sensor Discharge Model Flow

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                     BATTERY DISCHARGE CALCULATION                            │
└─────────────────────────────────────────────────────────────────────────────┘

                          ┌─────────────────┐
                          │  update() call  │
                          │  (every tick)   │
                          └────────┬────────┘
                                   │
                                   ▼
                          ┌─────────────────┐
                          │  Get Delta Time │
                          │  dt = now - last│
                          └────────┬────────┘
                                   │
                                   ▼
              ┌────────────────────┴────────────────────┐
              │          Select Discharge Mode          │
              └────────────────────┬────────────────────┘
                                   │
        ┌──────────────────────────┼──────────────────────────┐
        │                          │                          │
        ▼                          ▼                          
┌───────────────────────┐  ┌────────────────────────────────┐
│  SIMPLE-DISCHARGE-    │  │  ROTOR-POWER-DISCHARGE-MODE    │
│      MODE             │  │                                │
│                       │  │ P_rotor = sum of all rotor     │
│ drain = drain_rate    │  │           power consumption    │
│       * dt            │  │                                │
│                       │  │ consumed = P_rotor * dt        │
│ consumed = drain *    │  │          (in Joules)           │
│            multiplier │  │                                │
│                       │  │ V_terminal = f(SoC, I, R)      │
│ (abstract units)      │  │                                │
└───────┬───────────────┘  └────────┬───────────────────────┘
        │                           │                          
        └───────────────────────────┘
                                   │
                                   ▼
                          ┌─────────────────┐
                          │ Update State    │
                          │ remaining_J -=  │
                          │   consumed      │
                          └────────┬────────┘
                                   │
                                   ▼
                          ┌─────────────────┐
                          │ Calculate SOC   │
                          │ percent =       │
                          │  remaining /    │
                          │  max_capacity   │
                          └────────┬────────┘
                                   │
                                   ▼
                          ┌─────────────────┐
                          │ Update Status   │
                          │ if pct >= 20    │
                          │   status = OK   │
                          │ if pct < 20     │
                          │   status = LOW  │
                          │ if pct < 5      │
                          │   status = CRIT │
                          └────────┬────────┘
                                   │
                                   ▼
                          ┌─────────────────┐
                          │ Return Output   │
                          │ BatteryOutput { │
                          │   time_stamp,   │
                          │   pct_remaining,│
                          │   est_time_rem, │
                          │   charge_state, │
                          │   voltage,      │
                          │   current_draw  │
                          │ }               │
                          └─────────────────┘
```

---

## 3. Radar Sensor Detection Flow

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                     RADAR DETECTION ALGORITHM                                │
└─────────────────────────────────────────────────────────────────────────────┘

                          ┌─────────────────┐
                          │  update() call  │
                          └────────┬────────┘
                                   │
                                   ▼
                          ┌─────────────────┐
                          │ Get Radar Pose  │
                          │ position,       │
                          │ orientation     │
                          └────────┬────────┘
                                   │
                                   ▼
                    ┌──────────────────────────────┐
                    │  Generate Ray Directions     │
                    │  for az in azimuth_range:    │
                    │    for el in elevation_range:│
                    │      rays.add(direction)     │
                    └──────────────┬───────────────┘
                                   │
                                   ▼
           ┌───────────────────────────────────────────────┐
           │              UNREAL ENGINE                     │
           │                                                │
           │  ┌─────────────────────────────────────────┐  │
           │  │ Batch Line Trace                        │  │
           │  │ for each ray:                           │  │
           │  │   hit = LineTrace(origin, direction,    │  │
           │  │                   max_range)            │  │
           │  │   if hit:                               │  │
           │  │     hits.add(hit_result)               │  │
           │  └─────────────────────────────────────────┘  │
           │                                                │
           └───────────────────────┬───────────────────────┘
                                   │
                                   ▼
                    ┌──────────────────────────────┐
                    │  For Each Hit Result         │
                    └──────────────┬───────────────┘
                                   │
                    ┌──────────────┴──────────────┐
                    │                             │
                    ▼                             │
           ┌─────────────────┐                    │
           │ Calculate Range │                    │
           │ range = |hit -  │                    │
           │         origin| │                    │
           └────────┬────────┘                    │
                    │                             │
                    ▼                             │
           ┌─────────────────┐                    │
           │ Calculate Angles│                    │
           │ azimuth = atan2 │                    │
           │ elevation = asin│                    │
           └────────┬────────┘                    │
                    │                             │
                    ▼                             │
           ┌─────────────────┐                    │
           │ Get Hit Object  │                    │
           │ Velocity        │                    │
           └────────┬────────┘                    │
                    │                             │
                    ▼                             │
           ┌─────────────────┐                    │
           │ Calculate       │                    │
           │ Doppler         │                    │
           │ v_radial =      │                    │
           │   v · ray_dir   │                    │
           └────────┬────────┘                    │
                    │                             │
                    ▼                             │
           ┌─────────────────┐                    │
           │ Estimate RCS    │                    │
           │ from object type│                    │
           │ and orientation │                    │
           └────────┬────────┘                    │
                    │                             │
                    ▼                             │
           ┌─────────────────┐                    │
           │ Calculate Signal│                    │
           │ Strength (dB)   │                    │
           │ P = P_t * G^2 * │                    │
           │ λ^2 * RCS /     │                    │
           │ (4π)^3 * R^4    │                    │
           └────────┬────────┘                    │
                    │                             │
                    ▼                             │
           ┌─────────────────┐                    │
           │ Add Noise       │                    │
           │ if signal >     │                    │
           │   noise_floor:  │                    │
           │   add detection │                    │
           └────────┬────────┘                    │
                    │                             │
                    └──────────────┬──────────────┘
                                   │
                                   ▼
                    ┌──────────────────────────────┐
                    │  Cluster Detections          │
                    │  (merge nearby points)       │
                    └──────────────┬───────────────┘
                                   │
                                   ▼
                    ┌──────────────────────────────┐
                    │  Return RadarOutput          │
                    │  { detections[], timestamp } │
                    └──────────────────────────────┘
```

---

## 4. Clock Control Flow

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                     STEPPABLE CLOCK CONTROL FLOW                             │
└─────────────────────────────────────────────────────────────────────────────┘

  PYTHON CLIENT                     RPC                    SIMULATION CORE
  ═════════════                     ═══                    ═══════════════

  ┌─────────────┐
  │ User Script │
  │             │
  │ # Step mode │
  │ sim.pause() │──────────────────────────────────►┌─────────────────┐
  │             │                                    │ Set paused=true │
  │             │                                    └─────────────────┘
  │             │
  │ # Step for  │
  │ # 100ms     │
  │ sim.        │──────────────────────────────────►┌─────────────────┐
  │ continueFor │                                    │ target_time =   │
  │ Time(100ms) │                                    │ current + 100ms │
  │             │                                    │                 │
  │             │                                    │ while current < │
  │             │                                    │   target:       │
  │             │                                    │   tick()        │
  │             │                                    │                 │
  │             │◄──────────────────────────────────│ return when done│
  │             │                                    └─────────────────┘
  │             │
  │ # Get data  │
  │ imu = sim.  │──────────────────────────────────►┌─────────────────┐
  │ getImuData()│                                    │ Return sensor   │
  │             │◄──────────────────────────────────│ data at current │
  │             │                                    │ sim time        │
  │             │                                    └─────────────────┘
  │             │
  │ # Continue  │
  │ sim.        │──────────────────────────────────►┌─────────────────┐
  │ continueFor │                                    │ Step N physics  │
  │ Steps(10)   │                                    │ ticks           │
  │             │◄──────────────────────────────────│                 │
  │             │                                    └─────────────────┘
  │             │
  │ # Resume    │
  │ sim.        │──────────────────────────────────►┌─────────────────┐
  │ continue()  │                                    │ Set paused=false│
  │             │                                    │ Resume real-time│
  └─────────────┘                                    └─────────────────┘


  Clock Types:
  ════════════

  ┌───────────────┬────────────────────────────────────────────────────────┐
  │ REALTIME      │ Sim time matches wall clock time                       │
  │               │ Good for: Real-time visualization, demos               │
  ├───────────────┼────────────────────────────────────────────────────────┤
  │ STEPPABLE     │ Sim time only advances when explicitly stepped         │
  │               │ Good for: ML training, deterministic testing           │
  ├───────────────┼────────────────────────────────────────────────────────┤
  │ SCALED        │ Sim time runs faster/slower than wall clock            │
  │               │ Good for: Faster than real-time testing                │
  └───────────────┴────────────────────────────────────────────────────────┘
```

---

## 5. Settings.json Integration Flow

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                     SETTINGS.JSON PARSING FLOW                               │
└─────────────────────────────────────────────────────────────────────────────┘

                    ┌────────────────────────────────────┐
                    │          settings.json             │
                    │                                    │
                    │ {                                  │
                    │   "Vehicles": {                    │
                    │     "Drone1": {                    │
                    │       "Sensors": {                 │
                    │         "Battery1": {              │
                    │           "SensorType": 12,        │
                    │           "MaxCapacityWh": 100,    │
                    │           "NominalVoltage": 22.2,  │
                    │           "DischargeMode": "Linear"│
                    │         }                          │
                    │       }                            │
                    │     }                              │
                    │   }                                │
                    │ }                                  │
                    └─────────────────┬──────────────────┘
                                      │
                                      ▼
                    ┌────────────────────────────────────┐
                    │     AirSimSettings::loadSettings() │
                    └─────────────────┬──────────────────┘
                                      │
                                      ▼
                    ┌────────────────────────────────────┐
                    │     loadVehicleSettings()          │
                    │                                    │
                    │  for each vehicle in Vehicles:     │
                    │    loadSensorSettings(vehicle)     │
                    └─────────────────┬──────────────────┘
                                      │
                                      ▼
                    ┌────────────────────────────────────┐
                    │     loadSensorSettings()           │
                    │                                    │
                    │  for each sensor in Sensors:       │
                    │    type = sensor["SensorType"]     │
                    │    switch(type):                   │
                    │      case 12: loadBatterySetting() │
                    │      case 13: loadAirspeedSetting()│
                    │      case 14: loadRadarSetting()   │
                    └─────────────────┬──────────────────┘
                                      │
                                      ▼
                    ┌────────────────────────────────────┐
                    │     loadBatterySetting()           │
                    │                                    │
                    │  BatterySetting bs;                │
                    │  bs.max_capacity_wh =              │
                    │    json.getFloat("MaxCapacityWh"); │
                    │  bs.nominal_voltage =              │
                    │    json.getFloat("NominalVoltage");│
                    │  bs.discharge_mode =               │
                    │    parseDischargeMode(             │
                    │      json.getString("DischargeMode")│
                    │    );                              │
                    │  return bs;                        │
                    └─────────────────┬──────────────────┘
                                      │
                                      ▼
                    ┌────────────────────────────────────┐
                    │     SensorFactory::createSensor()  │
                    │                                    │
                    │  if type == Battery:               │
                    │    params = BatterySensorParams(   │
                    │      setting.max_capacity_wh,      │
                    │      setting.nominal_voltage,      │
                    │      setting.discharge_mode        │
                    │    );                              │
                    │    return BatterySensorSimple(     │
                    │      params                        │
                    │    );                              │
                    └────────────────────────────────────┘
```

---

## 6. RPC Communication Flow

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                     RPC CALL SEQUENCE                                        │
└─────────────────────────────────────────────────────────────────────────────┘

  PYTHON                    NETWORK                    C++ SERVER
  ══════                    ═══════                    ══════════

  client.py                  msgpack                  RpcLibServerBase
  ┌────────┐                ┌───────┐                ┌──────────────┐
  │        │                │       │                │              │
  │ def get│                │       │                │ bind("get    │
  │ Battery│───────────────►│ pack  │───────────────►│ BatteryData")│
  │ Data() │                │       │                │              │
  │        │                │       │                │ handler:     │
  │  name, │                │ [     │                │  vehicle_api │
  │  sensor│                │  "get │                │  ->get       │
  │  _name │                │   Bat │                │  BatteryData │
  │        │                │   tery│                │  (name,      │
  │        │                │   Data│                │   sensor)    │
  │        │                │  ",   │                │              │
  │        │                │  name,│                │              │
  │        │                │  sens │                │              │
  │        │                │ ]     │                │              │
  │        │                │       │                │              │
  │        │                │       │                └───────┬──────┘
  │        │                │       │                        │
  │        │                │       │                        ▼
  │        │                │       │                ┌──────────────┐
  │        │                │       │                │ VehicleApi   │
  │        │                │       │                │ Base::get    │
  │        │                │       │                │ BatteryData()│
  │        │                │       │                │              │
  │        │                │       │                │ sensor =     │
  │        │                │       │                │  findSensor( │
  │        │                │       │                │   Battery,   │
  │        │                │       │                │   name)      │
  │        │                │       │                │              │
  │        │                │       │                │ return sensor│
  │        │                │       │                │  ->getOutput()│
  │        │                │       │                └───────┬──────┘
  │        │                │       │                        │
  │        │                │       │                        ▼
  │        │                │       │                ┌──────────────┐
  │        │                │       │                │ RpcAdaptors::│
  │        │                │       │                │ BatteryData  │
  │        │                │       │                │              │
  │        │                │       │◄───────────────│ pack to      │
  │        │◄───────────────│unpack │                │ msgpack      │
  │        │                │       │                └──────────────┘
  │ return │                │       │
  │ Battery│                │       │
  │ Data.  │                │       │
  │ from_  │                │       │
  │ msgpack│                │       │
  │ (resp) │                │       │
  └────────┘                └───────┘
```

---

## 7. Complete Feature Integration Workflow

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                 FEATURE INTEGRATION WORKFLOW (END-TO-END)                    │
└─────────────────────────────────────────────────────────────────────────────┘

  ┌─────────────────────────────────────────────────────────────────────────┐
  │ STEP 1: DESIGN & PLANNING                                               │
  │                                                                         │
  │  [1.1] Review ProjectAirSim implementation                              │
  │  [1.2] Design Cosys-AirSim compatible interface                        │
  │  [1.3] Document API specification                                       │
  │  [1.4] Create test plan                                                 │
  └───────────────────────────────────┬─────────────────────────────────────┘
                                      │
                                      ▼
  ┌─────────────────────────────────────────────────────────────────────────┐
  │ STEP 2: C++ CORE IMPLEMENTATION                                         │
  │                                                                         │
  │  [2.1] Create sensor/feature header files                              │
  │  [2.2] Implement core logic                                             │
  │  [2.3] Add to SensorBase.hpp enum                                       │
  │  [2.4] Add settings parsing to AirSimSettings.hpp                       │
  │  [2.5] Add factory method to SensorFactory.hpp                          │
  │  [2.6] Add API methods to VehicleApiBase.hpp                            │
  └───────────────────────────────────┬─────────────────────────────────────┘
                                      │
                                      ▼
  ┌─────────────────────────────────────────────────────────────────────────┐
  │ STEP 3: RPC BINDING                                                     │
  │                                                                         │
  │  [3.1] Create data adaptor in RpcLibAdaptorsBase.hpp                    │
  │  [3.2] Add MSGPACK_DEFINE_MAP for serialization                         │
  │  [3.3] Add server bindings in RpcLibServerBase.cpp                      │
  │  [3.4] Test RPC serialization round-trip                                │
  └───────────────────────────────────┬─────────────────────────────────────┘
                                      │
                                      ▼
  ┌─────────────────────────────────────────────────────────────────────────┐
  │ STEP 4: UNREAL INTEGRATION (if needed)                                  │
  │                                                                         │
  │  [4.1] Create Unreal component (for hardware sensors)                   │
  │  [4.2] Implement tick/update logic                                      │
  │  [4.3] Connect to physics engine                                        │
  │  [4.4] Test in Unreal Editor                                            │
  └───────────────────────────────────┬─────────────────────────────────────┘
                                      │
                                      ▼
  ┌─────────────────────────────────────────────────────────────────────────┐
  │ STEP 5: PYTHON CLIENT                                                   │
  │                                                                         │
  │  [5.1] Add data class to types.py                                       │
  │  [5.2] Add client methods to client.py                                  │
  │  [5.3] Add to __init__.py exports                                       │
  │  [5.4] Test Python API                                                  │
  └───────────────────────────────────┬─────────────────────────────────────┘
                                      │
                                      ▼
  ┌─────────────────────────────────────────────────────────────────────────┐
  │ STEP 6: TESTING                                                         │
  │                                                                         │
  │  [6.1] Create unit tests in AirLibUnitTests/                            │
  │  [6.2] Create integration tests in PythonClient/tests/                  │
  │  [6.3] Create example script                                            │
  │  [6.4] Run full test suite                                              │
  │  [6.5] Performance benchmark                                            │
  └───────────────────────────────────┬─────────────────────────────────────┘
                                      │
                                      ▼
  ┌─────────────────────────────────────────────────────────────────────────┐
  │ STEP 7: DOCUMENTATION                                                   │
  │                                                                         │
  │  [7.1] Update docs/sensors/ with new sensor docs                        │
  │  [7.2] Update docs/apis.md with new API methods                         │
  │  [7.3] Update settings.json documentation                               │
  │  [7.4] Add example scripts with comments                                │
  └───────────────────────────────────┬─────────────────────────────────────┘
                                      │
                                      ▼
  ┌─────────────────────────────────────────────────────────────────────────┐
  │ STEP 8: REVIEW & MERGE                                                  │
  │                                                                         │
  │  [8.1] Code review                                                      │
  │  [8.2] Address review feedback                                          │
  │  [8.3] Final testing                                                    │
  │  [8.4] Merge to dev branch                                              │
  │  [8.5] Update changelog                                                 │
  └─────────────────────────────────────────────────────────────────────────┘
```

---

## 8. File Dependency Graph

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                     FILE DEPENDENCY GRAPH                                    │
└─────────────────────────────────────────────────────────────────────────────┘

  ┌─────────────────────────────────────────────────────────────────────────┐
  │                           CORE HEADERS                                   │
  │                                                                         │
  │  common/Common.hpp ◄──────────────────────────────────────────────────┐ │
  │         │                                                             │ │
  │         ▼                                                             │ │
  │  sensors/SensorBase.hpp ◄──────────────────────────────────────────┐  │ │
  │         │                                                          │  │ │
  │         ├─────────────────────────────────────────────────────────┐│  │ │
  │         │                                                         ││  │ │
  │         ▼                                                         ││  │ │
  │  ┌────────────────────┐  ┌────────────────────┐  ┌──────────────┐ ││  │ │
  │  │ battery/           │  │ airspeed/          │  │ radar/       │ ││  │ │
  │  │ BatterySensorBase  │  │ AirspeedSensorBase │  │ RadarSensor  │ ││  │ │
  │  │         │          │  │         │          │  │ Base         │ ││  │ │
  │  │         ▼          │  │         ▼          │  │      │       │ ││  │ │
  │  │ BatterySensorSimple│  │ AirspeedSensorSimp │  │      ▼       │ ││  │ │
  │  │         │          │  │         │          │  │ RadarSensor  │ ││  │ │
  │  │         ▼          │  │         ▼          │  │ Simple       │ ││  │ │
  │  │ BatterySensorParams│  │ AirspeedSensorParam│  │      │       │ ││  │ │
  │  └─────────┬──────────┘  └─────────┬──────────┘  └──────┬───────┘ ││  │ │
  │            │                       │                    │         ││  │ │
  │            └───────────────────────┴────────────────────┘         ││  │ │
  │                                    │                              ││  │ │
  │                                    ▼                              ││  │ │
  │                     sensors/SensorFactory.hpp ◄───────────────────┘│  │ │
  │                                    │                               │  │ │
  └────────────────────────────────────┼───────────────────────────────┼──┘ │
                                       │                               │    │
  ┌────────────────────────────────────┼───────────────────────────────┼────┘
  │                                    │                               │
  │         ┌──────────────────────────┘                               │
  │         │                                                          │
  │         ▼                                                          │
  │  common/AirSimSettings.hpp ◄───────────────────────────────────────┘
  │         │
  │         │
  │         ▼
  │  api/VehicleApiBase.hpp ──────────────────────────────────────────────┐
  │         │                                                             │
  │         ▼                                                             │
  │  api/RpcLibAdaptorsBase.hpp ◄────────────────────────────────────┐    │
  │         │                                                        │    │
  │         ▼                                                        │    │
  │  src/api/RpcLibServerBase.cpp ◄──────────────────────────────────┤    │
  │                                                                  │    │
  └──────────────────────────────────────────────────────────────────┼────┘
                                                                     │
  ┌──────────────────────────────────────────────────────────────────┘
  │
  │  PYTHON CLIENT
  │  ═════════════
  │
  │  PythonClient/cosysairsim/
  │         │
  │         ├── types.py ◄─────────────────────────────────────────────────┐
  │         │                                                              │
  │         ├── client.py ◄────────────────────────────────────────────────┤
  │         │                                                              │
  │         └── __init__.py ◄──────────────────────────────────────────────┘
  │
  └────────────────────────────────────────────────────────────────────────
```

---

## Summary

These diagrams provide visual guidance for:

1. **Sensor Data Flow** - How data moves from simulation to Python client
2. **Battery Discharge Model** - Algorithm for battery state calculation
3. **Radar Detection** - Ray-casting based object detection
4. **Clock Control** - Steppable simulation timing
5. **Settings Parsing** - Configuration file to runtime objects
6. **RPC Communication** - Client-server message passing
7. **Integration Workflow** - End-to-end feature addition process
8. **File Dependencies** - Code organization and includes

Use these diagrams as reference during implementation to ensure correct architecture and data flow.
