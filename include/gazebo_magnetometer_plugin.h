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

#include <msgs/MagneticField.pb.h>
#include <msgs/Groundtruth.pb.h>

#include <ignition/transport/Node.hh>
#include <ignition/math.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/Pose.hh>

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

  class IGNITION_GAZEBO_VISIBLE MagnetometerPlugin:
    // This is class a system.
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPreUpdate,
    public ignition::gazebo::ISystemPostUpdate
    {
    public: MagnetometerPlugin();
    public: ~MagnetometerPlugin() override;
    public: void Configure(const ignition::gazebo::Entity &_entity,
                            const std::shared_ptr<const sdf::Element> &_sdf,
                            ignition::gazebo::EntityComponentManager &_ecm,
                            ignition::gazebo::EventManager &/*_eventMgr*/);
    public: void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;
    public: void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                const ignition::gazebo::EntityComponentManager &_ecm) override;

    private:
      void addNoise(Eigen::Vector3d* magnetic_field, const double dt);
      void GroundtruthCallback(const sensor_msgs::msgs::Groundtruth& gt_msg);
      void getSdfParams(const std::shared_ptr<const sdf::Element> &sdf);

      ignition::gazebo::Model model_{ignition::gazebo::kNullEntity};
      ignition::gazebo::Entity model_link_{ignition::gazebo::kNullEntity};

      std::string mag_topic_;
      ignition::transport::Node node;
      ignition::transport::Node::Publisher pub_mag_;
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
