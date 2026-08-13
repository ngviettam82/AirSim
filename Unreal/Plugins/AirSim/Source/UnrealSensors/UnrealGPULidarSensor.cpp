// Developed by Cosys-Lab, University of Antwerp

#include "UnrealGPULidarSensor.h"
#include "AirBlueprintLib.h"
#include "common/Common.hpp"
#include "NedTransform.h"
#include "DrawDebugHelpers.h"
#include "UObject/ConstructorHelpers.h"
#include "Engine/Engine.h"
#include <random>

// ctor
UnrealGPULidarSensor::UnrealGPULidarSensor(const AirSimSettings::GPULidarSetting& setting,
	AActor* actor, const NedTransform* ned_transform)
	: GPULidarSimple(setting), actor_(actor), ned_transform_(ned_transform),
	sensor_params_(getParams()),
	draw_time_(1.05f / sensor_params_.horizontal_rotation_frequency),
	external_(getParams().external)
{

	FActorSpawnParameters lidar_spawn_params;

	lidar_spawn_params.Name = FName(*(actor->GetName() + "_gpulidar_" + FString(setting.sensor_name.c_str())));
	lidar_camera_ = actor->GetWorld()->SpawnActor<ALidarCamera>(lidar_spawn_params);
	if (!getParams().external) {
		lidar_camera_->AttachToActor(actor, FAttachmentTransformRules::KeepRelativeTransform);
	}
	lidar_camera_->InitializeSettingsFromAirSim(getParams());
}

// Pause Unreal simulation
void UnrealGPULidarSensor::pause(const bool is_paused) {
	if (is_paused) {
		saved_clockspeed_ = UAirBlueprintLib::getUnrealClockSpeed(actor_);
		UAirBlueprintLib::setUnrealClockSpeed(actor_, 0);
	}
	else {
		UAirBlueprintLib::setUnrealClockSpeed(actor_, saved_clockspeed_);
	}
}

// returns a point-cloud for the tick
bool UnrealGPULidarSensor::getPointCloud(float delta_time, msr::airlib::vector<msr::airlib::real_T>& point_cloud, msr::airlib::vector<msr::airlib::real_T>& point_cloud_final)
{
	// Multirotor async path runs on the physics thread — never draw/debug from here.
	// DrawDebugPoints is ignored for Multirotor in InitializeSettingsFromAirSim.
	if (sensor_params_.draw_sensor && !sensor_params_.async_capture_mode) {
		UAirBlueprintLib::DrawPoint(actor_->GetWorld(), lidar_camera_->GetCachedWorldTransform().GetLocation(), 5, FColor::Black, false, draw_time_);
		UAirBlueprintLib::DrawCoordinateSystem(actor_->GetWorld(), lidar_camera_->GetCachedWorldTransform().GetLocation(),
			lidar_camera_->GetCachedWorldTransform().Rotator(), 25, false, draw_time_, 10);
	}
	return lidar_camera_->Update(delta_time, point_cloud, point_cloud_final);
}

// Get echo pose in Local NED
void UnrealGPULidarSensor::getLocalPose(msr::airlib::Pose& sensor_pose)
{
	// Prefer game-thread-cached transform (safe for Multirotor physics Update).
	sensor_pose = ned_transform_->toLocalNed(lidar_camera_->GetCachedWorldTransform());
}

