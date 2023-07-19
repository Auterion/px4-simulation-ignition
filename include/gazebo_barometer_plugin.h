
/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief Barometer Plugin
 *
 * This plugin simulates barometer data
 *
 * @author Elia Tarasov <elias.tarasov@gmail.com>
 */

#ifndef BAROMETER_PLUGIN_HH_
#define BAROMETER_PLUGIN_HH_

#include <random>

#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Pose.hh>

#include <Pressure.pb.h>

static constexpr auto kDefaultBarometerTopic = "/baro";
static constexpr auto kDefaultPubRate = 50;  // [Hz]. Note: averages the supported Baro device ODR in PX4
static constexpr auto kDefaultAltHome = 488.0; // meters

namespace barometer_plugin
{
    class GZ_SIM_VISIBLE BarometerPlugin:
    // This is class a system.
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
    {
    public: BarometerPlugin();
    public: ~BarometerPlugin() override;
    public: void Configure(const gz::sim::Entity &_entity,
                            const std::shared_ptr<const sdf::Element> &_sdf,
                            gz::sim::EntityComponentManager &_ecm,
                            gz::sim::EventManager &/*_eventMgr*/);
    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;
    public: void PostUpdate(const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) override;
    private:
        gz::sim::Model model_{gz::sim::kNullEntity};
        gz::sim::Entity model_link_{gz::sim::kNullEntity};

        std::chrono::steady_clock::duration last_pub_time_{0};

        std::string namespace_;
        std::string baro_topic_;
        unsigned int pub_rate_{kDefaultPubRate};

        std::default_random_engine random_generator_;
        std::normal_distribution<double> standard_normal_distribution_;

        gz::math::Pose3d pose_model_start_;
        gz::math::Vector3d gravity_W_{gz::math::Vector3d(0.0, 0.0, -9.8)};
        double alt_home_;

        gz::transport::Node node;
        gz::transport::Node::Publisher pub_baro_;

        sensor_msgs::msgs::Pressure baro_msg_;

        // state variables for baro pressure sensor random noise generator
        double baro_rnd_y2_{0.0};
        bool baro_rnd_use_last_{false};
        double baro_drift_pa_per_sec_;
        double baro_drift_pa_{0.0};
    };
}

#endif
