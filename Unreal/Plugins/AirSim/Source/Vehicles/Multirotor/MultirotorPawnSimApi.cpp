#include "MultirotorPawnSimApi.h"
#include "AirBlueprintLib.h"
#include "vehicles/multirotor/MultiRotorParamsFactory.hpp"
#include "UnrealSensors/UnrealSensorFactory.h"
#include "sensors/SensorBase.hpp"
#include "common/RecordingCapture.hpp"
#include <exception>
#include <sstream>

using namespace msr::airlib;

MultirotorPawnSimApi::MultirotorPawnSimApi(const Params& params)
    : PawnSimApi(params), pawn_events_(static_cast<MultirotorPawnEvents*>(params.pawn_events))
{
}

void MultirotorPawnSimApi::initialize()
{
    PawnSimApi::initialize();

    //create vehicle API
    std::shared_ptr<UnrealSensorFactory> sensor_factory = std::make_shared<UnrealSensorFactory>(getPawn(), &getNedTransform());
    vehicle_params_ = MultiRotorParamsFactory::createConfig(getVehicleSetting(), sensor_factory);
    vehicle_api_ = vehicle_params_->createMultirotorApi();
    //setup physics vehicle
    multirotor_physics_body_ = std::unique_ptr<MultiRotor>(new MultiRotorPhysicsBody(vehicle_params_.get(), vehicle_api_.get(), getKinematics(), getEnvironment()));
    rotor_count_ = multirotor_physics_body_->wrenchVertexCount();
    rotor_actuator_info_.assign(rotor_count_, RotorActuatorInfo());

    vehicle_api_->setSimulatedGroundTruth(getGroundTruthKinematics(), getGroundTruthEnvironment());

    //initialize private vars
    last_phys_pose_ = Pose::nanPose();
    pending_pose_status_ = PendingPoseStatus::NonePending;
    reset_pending_ = false;
    did_reset_ = false;
    rotor_states_.rotors.assign(rotor_count_, RotorParameters());

    //reset roll & pitch of vehicle as multirotors required to be on plain surface at start
    Pose pose = getPose();
    float pitch, roll, yaw;
    VectorMath::toEulerianAngle(pose.orientation, pitch, roll, yaw);
    pose.orientation = VectorMath::toQuaternion(0, 0, yaw);
    setPose(pose, false);
}

void MultirotorPawnSimApi::pawnTick(float dt)
{
    unused(dt);
    //calls to update* are handled by physics engine and in SimModeWorldBase
}

namespace
{
    using SensorType = msr::airlib::SensorBase::SensorType;

    const msr::airlib::SensorBase* findRecordingSensor(
        const msr::airlib::VehicleApiBase* api,
        const std::string& sensor_name,
        SensorType& out_type)
    {
        if (api == nullptr)
            return nullptr;
        const SensorType candidates[] = {
            SensorType::Imu, SensorType::Gps, SensorType::Barometer,
            SensorType::Magnetometer, SensorType::Distance
        };
        try {
            const auto& sensors = api->getSensors();
            for (SensorType type : candidates) {
                const uint count = sensors.size(type);
                for (uint i = 0; i < count; ++i) {
                    const auto* sensor = sensors.getByType(type, i);
                    if (!sensor)
                        continue;
                    if (sensor_name.empty() ||
                        msr::airlib::RecordingCapture::namesEqualIgnoreCase(sensor->getName(), sensor_name)) {
                        out_type = type;
                        return sensor;
                    }
                }
            }
        }
        catch (...) {
            return nullptr;
        }
        return nullptr;
    }
}

msr::airlib::RecordingCapture MultirotorPawnSimApi::createRecordingCapture(
    uint64_t sequence_id,
    const std::vector<std::string>& sensor_names,
    const std::vector<std::string>& schema_tokens) const
{
    RecordingCapture capture = PawnSimApi::createRecordingCapture(sequence_id, sensor_names, schema_tokens);
    const auto* api = getVehicleApiBase();
    const auto& schema_names = msr::airlib::AirSimSettings::singleton().recording_setting.sensor_schema;

    for (size_t si = 0; si < sensor_names.size(); ++si) {
        const auto& sensor_name = sensor_names[si];
        msr::airlib::RecordingSensorSample sample;
        sample.sensor_name = sensor_name;
        for (size_t ti = 0; ti < schema_names.size() && ti < schema_tokens.size(); ++ti) {
            if (msr::airlib::RecordingCapture::namesEqualIgnoreCase(schema_names[ti], sensor_name)) {
                sample.schema_token = schema_tokens[ti];
                break;
            }
        }
        if (sample.schema_token.empty() && si < schema_tokens.size())
            sample.schema_token = schema_tokens[si];

        SensorType type = SensorType::Imu;
        const auto* sensor = findRecordingSensor(api, sensor_name, type);
        if (!sensor) {
            sample.present = false;
            sample.error = "sensor not found, disabled, or unsupported";
            UAirBlueprintLib::LogMessageString("Recording sensor missing: ", sensor_name, LogDebugLevel::Failure);
            capture.sensors.push_back(sample);
            continue;
        }

        sample.sensor_type = type;
        sample.sensor_name = sensor->getName();
        try {
            switch (type) {
            case SensorType::Imu: {
                const auto& out = api->getImuData(sensor->getName());
                sample.sensor_time_stamp = out.time_stamp;
                sample.imu_linear_acceleration = out.linear_acceleration;
                sample.imu_angular_velocity = out.angular_velocity;
                sample.imu_orientation = out.orientation;
                sample.present = (out.time_stamp != 0);
                if (!sample.present)
                    sample.error = "IMU has no output yet (timestamp 0)";
                break;
            }
            case SensorType::Gps: {
                const auto& out = api->getGpsData(sensor->getName());
                sample.sensor_time_stamp = out.time_stamp;
                sample.gps_lat = out.gnss.geo_point.latitude;
                sample.gps_lon = out.gnss.geo_point.longitude;
                sample.gps_alt = out.gnss.geo_point.altitude;
                sample.gps_velocity = out.gnss.velocity;
                sample.gps_eph = out.gnss.eph;
                sample.gps_epv = out.gnss.epv;
                sample.gps_fix = static_cast<int>(out.gnss.fix_type);
                sample.gps_valid = out.is_valid;
                sample.present = (out.time_stamp != 0);
                if (!sample.present)
                    sample.error = "GPS has no output yet";
                break;
            }
            case SensorType::Barometer: {
                const auto& out = api->getBarometerData(sensor->getName());
                sample.sensor_time_stamp = out.time_stamp;
                sample.baro_altitude = out.altitude;
                sample.baro_pressure = out.pressure;
                sample.baro_qnh = out.qnh;
                sample.present = (out.time_stamp != 0);
                if (!sample.present)
                    sample.error = "Barometer has no output yet";
                break;
            }
            case SensorType::Magnetometer: {
                const auto& out = api->getMagnetometerData(sensor->getName());
                sample.sensor_time_stamp = out.time_stamp;
                sample.mag_field_body = out.magnetic_field_body;
                sample.present = (out.time_stamp != 0);
                if (!sample.present)
                    sample.error = "Magnetometer has no output yet";
                break;
            }
            case SensorType::Distance: {
                const auto& out = api->getDistanceSensorData(sensor->getName());
                sample.sensor_time_stamp = out.time_stamp;
                sample.distance = out.distance;
                sample.distance_min = out.min_distance;
                sample.distance_max = out.max_distance;
                sample.present = (out.time_stamp != 0);
                if (!sample.present)
                    sample.error = "Distance has no output yet";
                break;
            }
            default:
                sample.present = false;
                sample.error = "unsupported sensor type";
                break;
            }
        }
        catch (const std::exception& ex) {
            sample.present = false;
            sample.error = ex.what();
        }

        if (sample.present && sample.sensor_time_stamp != 0) {
            sample.age_ns = msr::airlib::recordingSensorAgeNs(
                capture.frame_time_stamp, sample.sensor_time_stamp);
        }
        capture.sensors.push_back(sample);
    }
    return capture;
}

std::string MultirotorPawnSimApi::getRecordFileLine(bool is_header_line) const
{
    const auto& schema = msr::airlib::AirSimSettings::singleton().recording_setting.sensor_schema;
    const auto tokens = msr::airlib::RecordingCapture::buildSchemaTokens(schema);
    if (is_header_line)
        return msr::airlib::RecordingCapture::headerColumns(tokens);

    std::vector<std::string> names;
    const auto& sensors_map = msr::airlib::AirSimSettings::singleton().recording_setting.sensors;
    const auto it = sensors_map.find(getVehicleName());
    if (it != sensors_map.end())
        names = it->second;
    return createRecordingCapture(0, names, tokens).toRecordLine();
}

void MultirotorPawnSimApi::updateRenderedState(float dt)
{
    //Utils::log("------Render tick-------");

    //if reset is pending then do it first, no need to do other things until next tick
    if (reset_pending_) {
        reset_task_();
        did_reset_ = true;
        return;
    }

    //move collision info from rendering engine to vehicle
    const CollisionInfo& collision_info = getCollisionInfo();
    multirotor_physics_body_->setCollisionInfo(collision_info);

    last_phys_pose_ = multirotor_physics_body_->getPose();

    collision_response = multirotor_physics_body_->getCollisionResponseInfo();

    //update rotor poses
    for (unsigned int i = 0; i < rotor_count_; ++i) {
        const auto& rotor_output = multirotor_physics_body_->getRotorOutput(i);
        // update private rotor variable
        rotor_states_.rotors[i].update(rotor_output.thrust, rotor_output.torque_scaler, rotor_output.speed);
        RotorActuatorInfo* info = &rotor_actuator_info_[i];
        info->rotor_speed = rotor_output.speed;
        info->rotor_direction = static_cast<int>(rotor_output.turning_direction);
        info->rotor_thrust = rotor_output.thrust;
        info->rotor_control_filtered = rotor_output.control_signal_filtered;
    }

    vehicle_api_->getStatusMessages(vehicle_api_messages_);

    if (getRemoteControlID() >= 0)
        vehicle_api_->setRCData(getRCData());
    rotor_states_.timestamp = clock()->nowNanos();
    vehicle_api_->setRotorStates(rotor_states_);
}

void MultirotorPawnSimApi::updateRendering(float dt)
{
    //if we did reset then don't worry about synchronizing states for this tick
    if (reset_pending_) {
        // Continue to wait for reset
        if (!did_reset_) {
            return;
        }
        else {
            reset_pending_ = false;
            did_reset_ = false;
            return;
        }
    }

    if (!VectorMath::hasNan(last_phys_pose_)) {
        if (pending_pose_status_ == PendingPoseStatus::RenderPending) {
            PawnSimApi::setPose(last_phys_pose_, pending_pose_collisions_);
            pending_pose_status_ = PendingPoseStatus::NonePending;
        }
        else
            PawnSimApi::setPose(last_phys_pose_, false);
    }

    //UAirBlueprintLib::LogMessage(TEXT("Collision (raw) Count:"), FString::FromInt(collision_response.collision_count_raw), LogDebugLevel::Unimportant);
    UAirBlueprintLib::LogMessage(TEXT("Collision Count:"),
                                 FString::FromInt(collision_response.collision_count_non_resting),
                                 LogDebugLevel::Informational);

    for (auto i = 0; i < vehicle_api_messages_.size(); ++i) {
        UAirBlueprintLib::LogMessage(FString(vehicle_api_messages_[i].c_str()), TEXT(""), LogDebugLevel::Success, 30);
    }

    try {
        vehicle_api_->sendTelemetry(dt);
    }
    catch (std::exception& e) {
        UAirBlueprintLib::LogMessage(FString(e.what()), TEXT(""), LogDebugLevel::Failure, 30);
    }

    pawn_events_->getActuatorSignal().emit(rotor_actuator_info_);
}

void MultirotorPawnSimApi::setPose(const Pose& pose, bool ignore_collision)
{
    multirotor_physics_body_->lock();
    multirotor_physics_body_->setPose(pose);
    multirotor_physics_body_->setGrounded(false);
    multirotor_physics_body_->unlock();
    pending_pose_collisions_ = ignore_collision;
    pending_pose_status_ = PendingPoseStatus::RenderPending;
}

void MultirotorPawnSimApi::setKinematics(const Kinematics::State& state, bool ignore_collision)
{
    multirotor_physics_body_->lock();
    multirotor_physics_body_->updateKinematics(state);
    multirotor_physics_body_->setGrounded(false);
    multirotor_physics_body_->unlock();
    pending_pose_collisions_ = ignore_collision;
    pending_pose_status_ = PendingPoseStatus::RenderPending;
}

//*** Start: UpdatableState implementation ***//
void MultirotorPawnSimApi::resetImplementation()
{
    PawnSimApi::resetImplementation();

    vehicle_api_->reset();
    multirotor_physics_body_->reset();
    vehicle_api_messages_.clear();
}

//this is high frequency physics tick, flier gets ticked at rendering frame rate
void MultirotorPawnSimApi::update(float delta)
{
    //environment update for current position
    PawnSimApi::update(delta);

    //update forces on vertices
    multirotor_physics_body_->update(delta);

    //update to controller must be done after kinematics have been updated by physics engine
}

void MultirotorPawnSimApi::reportState(StateReporter& reporter)
{
    PawnSimApi::reportState(reporter);

    multirotor_physics_body_->reportState(reporter);
}

MultirotorPawnSimApi::UpdatableObject* MultirotorPawnSimApi::getPhysicsBody()
{
    return multirotor_physics_body_->getPhysicsBody();
}
//*** End: UpdatableState implementation ***//
