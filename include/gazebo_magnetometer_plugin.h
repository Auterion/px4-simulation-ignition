/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @brief Magnetometer Plugin
 *
 * This plugin simulates magnetometer data
 *
 * @author Elia Tarasov <elias.tarasov@gmail.com>
 */

#ifndef MAGNETOMETER_PLUGIN_HH_
#define MAGNETOMETER_PLUGIN_HH_

#include <random>
#include <string>

#include <Eigen/Core>
#include <boost/shared_array.hpp>

#include <MagneticField.pb.h>
#include <Groundtruth.pb.h>

#include <gz/transport/Node.hh>
#include <gz/math.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Pose.hh>

#include "geo_mag_declination.h"
#include <common.h>

namespace magnetometer_plugin {

  static constexpr auto kDefaultMagnetometerTopic = "mag";
  static constexpr auto kDefaultPubRate = 100.0; // [Hz]. Note: corresponds to most of the mag devices supported in PX4

  // Default values for use with ADIS16448 IMU
  static constexpr auto kDefaultNoiseDensity = 0.4*1e-3; // [gauss / sqrt(hz)]
  static constexpr auto kDefaultRandomWalk = 6.4*1e-6; // [gauss * sqrt(hz)]
  static constexpr auto kDefaultBiasCorrelationTime = 6.0e+2; // [s]

  typedef const boost::shared_ptr<const sensor_msgs::msgs::Groundtruth> GtPtr;

  class GZ_SIM_VISIBLE MagnetometerPlugin:
    // This is class a system.
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
    {
    public: MagnetometerPlugin();
    public: ~MagnetometerPlugin() override;
    public: void Configure(const gz::sim::Entity &_entity,
                            const std::shared_ptr<const sdf::Element> &_sdf,
                            gz::sim::EntityComponentManager &_ecm,
                            gz::sim::EventManager &/*_eventMgr*/);
    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;
    public: void PostUpdate(const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) override;

    private:
      void addNoise(Eigen::Vector3d* magnetic_field, const double dt);
      void GroundtruthCallback(const sensor_msgs::msgs::Groundtruth& gt_msg);
      void getSdfParams(const std::shared_ptr<const sdf::Element> &sdf);

      gz::sim::Model model_{gz::sim::kNullEntity};
      gz::sim::Entity model_link_{gz::sim::kNullEntity};

      std::string mag_topic_;
      gz::transport::Node node;
      gz::transport::Node::Publisher pub_mag_;
      std::string gt_sub_topic_;

      double groundtruth_lat_rad_;
      double groundtruth_lon_rad_;

      sensor_msgs::msgs::MagneticField mag_message_;

      std::chrono::steady_clock::duration last_time_{0};
      std::chrono::steady_clock::duration last_pub_time_{0};
      unsigned int pub_rate_;
      double noise_density_;
      double random_walk_;
      double bias_correlation_time_;

      Eigen::Vector3d bias_;

      std::default_random_engine random_generator_;
      std::normal_distribution<double> standard_normal_distribution_;
  }; // class MagnetometerPlugin
}  // namespace gazebo
#endif // MAGNETOMETER_PLUGIN_HH_
