// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_Gps_hpp
#define msr_airlib_Gps_hpp

#include <random>
#include "common/Common.hpp"
#include "GpsSimpleParams.hpp"
#include "GpsBase.hpp"
#include "common/FirstOrderFilter.hpp"
#include "common/FrequencyLimiter.hpp"
#include "common/DelayLine.hpp"
#include "common/GaussianMarkov.hpp"
#include "common/EarthUtils.hpp"

namespace msr
{
namespace airlib
{

    class GpsSimple : public GpsBase
    {
    public: //methods
        GpsSimple(const AirSimSettings::GpsSetting& setting = AirSimSettings::GpsSetting())
            : GpsBase(setting.sensor_name)
        {
            // initialize params
            params_.initializeFromSettings(setting);

            //initialize frequency limiter
            freq_limiter_.initialize(params_.update_frequency, params_.startup_delay);
            delay_line_.initialize(params_.update_latency);

            //initialize filters
            eph_filter.initialize(params_.eph_time_constant, params_.eph_final, params_.eph_initial); //starting dilution set to 100 which we will reduce over time to targeted 0.3f, with 45% accuracy within 100 updates, each update occurring at 0.2s interval
            epv_filter.initialize(params_.epv_time_constant, params_.epv_final, params_.epv_initial);

            bias_n_.initialize(params_.bias_tau, params_.bias_sigma, 0);
            bias_e_.initialize(params_.bias_tau, params_.bias_sigma, 0);
            bias_d_.initialize(params_.bias_tau, params_.bias_sigma * 1.5f, 0);
        }

        //*** Start: UpdatableState implementation ***//
        virtual void resetImplementation() override
        {
            freq_limiter_.reset();
            delay_line_.reset();

            eph_filter.reset();
            epv_filter.reset();
            bias_n_.reset();
            bias_e_.reset();
            bias_d_.reset();
            gauss_dist_.reset();

            addOutputToDelayLine(eph_filter.getOutput(), epv_filter.getOutput());
        }

        virtual void update(float delta = 0) override
        {
            GpsBase::update(delta);

            freq_limiter_.update(delta);
            eph_filter.update(delta);
            epv_filter.update(delta);
            if (params_.generate_noise) {
                bias_n_.update(delta);
                bias_e_.update(delta);
                bias_d_.update(delta);
            }

            if (freq_limiter_.isWaitComplete()) { //update output
                addOutputToDelayLine(eph_filter.getOutput(), epv_filter.getOutput());
            }

            delay_line_.update(delta);

            if (freq_limiter_.isWaitComplete())
                setOutput(delay_line_.getOutput());
        }

        //*** End: UpdatableState implementation ***//

        virtual ~GpsSimple() = default;

    private:
        void addOutputToDelayLine(real_T eph, real_T epv)
        {
            Output output;
            const GroundTruth& ground_truth = getGroundTruth();

            //GNSS
            output.gnss.time_utc = static_cast<uint64_t>(clock()->nowNanos() / 1.0E3);
            output.gnss.geo_point = ground_truth.environment->getState().geo_point;
            output.gnss.eph = eph;
            output.gnss.epv = epv;
            output.gnss.velocity = ground_truth.kinematics->twist.linear;
            output.is_valid = true;

            if (params_.generate_noise) {
                // Position noise in local NED, then convert offset to geo deltas (first-order).
                const real_T sig_h = std::max(0.01f, eph * params_.position_sigma_scale);
                const real_T sig_v = std::max(0.01f, epv * params_.position_sigma_scale);
                const Vector3r noise_ned(
                    gauss_dist_.next() * sig_h + bias_n_.getOutput(),
                    gauss_dist_.next() * sig_h + bias_e_.getOutput(),
                    gauss_dist_.next() * sig_v + bias_d_.getOutput());

                // Approximate geo offset: 1 deg lat ~ 111320 m, lon scaled by cos(lat)
                const double lat = output.gnss.geo_point.latitude;
                const double meters_per_deg_lat = 111320.0;
                const double meters_per_deg_lon =
                    std::max(1.0, meters_per_deg_lat * std::cos(Utils::degreesToRadians(lat)));
                output.gnss.geo_point.latitude += static_cast<double>(noise_ned.x()) / meters_per_deg_lat;
                output.gnss.geo_point.longitude += static_cast<double>(noise_ned.y()) / meters_per_deg_lon;
                output.gnss.geo_point.altitude -= noise_ned.z(); // NED z down

                output.gnss.velocity += Vector3r(
                    gauss_dist_.next() * params_.velocity_sigma,
                    gauss_dist_.next() * params_.velocity_sigma,
                    gauss_dist_.next() * params_.velocity_sigma);
            }

            output.gnss.fix_type =
                output.gnss.eph <= params_.eph_min_3d   ? GnssFixType::GNSS_FIX_3D_FIX
                : output.gnss.eph <= params_.eph_min_2d ? GnssFixType::GNSS_FIX_2D_FIX
                                                        : GnssFixType::GNSS_FIX_NO_FIX;

            output.time_stamp = clock()->nowNanos();

            delay_line_.push_back(output);
        }

    private:
        typedef std::normal_distribution<> NormalDistribution;

        GpsSimpleParams params_;

        FirstOrderFilter<real_T> eph_filter, epv_filter;
        FrequencyLimiter freq_limiter_;
        DelayLine<Output> delay_line_;
        GaussianMarkov bias_n_, bias_e_, bias_d_;
        RandomGeneratorGausianR gauss_dist_ = RandomGeneratorGausianR(0.0f, 1.0f);
    };
}
} //namespace
#endif
