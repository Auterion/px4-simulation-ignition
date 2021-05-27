
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

#include "gazebo_barometer_plugin.h"

#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/Name.hh>

IGNITION_ADD_PLUGIN(
    barometer_plugin::BarometerPlugin,
    ignition::gazebo::System,
    barometer_plugin::BarometerPlugin::ISystemConfigure,
    barometer_plugin::BarometerPlugin::ISystemPreUpdate,
    barometer_plugin::BarometerPlugin::ISystemPostUpdate)

using namespace barometer_plugin;


BarometerPlugin::BarometerPlugin()
{
  pub_baro_ = this->node.Advertise<sensor_msgs::msgs::Pressure>("/world/quadcopter/model/X3/link/base_link/sensor/barometer");
}

BarometerPlugin::~BarometerPlugin() {
}

void BarometerPlugin::Configure(const ignition::gazebo::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      ignition::gazebo::EntityComponentManager &_ecm,
      ignition::gazebo::EventManager &_em) {
    auto linkName = _sdf->Get<std::string>("link_name");
    model_ = ignition::gazebo::Model(_entity);
    // Get link entity
    model_link_ = model_.LinkByName(_ecm, linkName);

  if(!_ecm.EntityHasComponentType(model_link_, ignition::gazebo::components::WorldPose::typeId))
  {
    _ecm.CreateComponent(model_link_, ignition::gazebo::components::WorldPose());
  }
  if(!_ecm.EntityHasComponentType(model_link_, ignition::gazebo::components::WorldLinearVelocity::typeId))
  {
    _ecm.CreateComponent(model_link_, ignition::gazebo::components::WorldLinearVelocity());
  }
}


void BarometerPlugin::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
  ignition::gazebo::EntityComponentManager &_ecm) {
    const std::chrono::steady_clock::duration current_time = _info.simTime;
    const double dt = std::chrono::duration<double>(current_time - last_pub_time_).count();
  if (dt > 1.0 / pub_rate_) {
    // get pose of the model that the plugin is attached to
    const ignition::gazebo::components::WorldPose* pComp = _ecm.Component<ignition::gazebo::components::WorldPose>(model_link_);
    const ignition::math::Pose3d pose_model_world = pComp->Data();
    // const ignition::math::Pose3d pose_model_world;
    // std::cout << "World Pose: " << pose_model_world.Pos().Z() << std::endl;
    ignition::math::Pose3d pose_model; // Z-component pose in local frame (relative to where it started)
    pose_model.Pos().Z() = pose_model_world.Pos().Z() - pose_model_start_.Pos().Z();
    const float pose_n_z = -pose_model.Pos().Z(); // convert Z-component from ENU to NED

    // calculate abs_pressure using an ISA model for the tropsphere (valid up to 11km above MSL)
    const float lapse_rate = 0.0065f; // reduction in temperature with altitude (Kelvin/m)
    const float temperature_msl = 288.0f; // temperature at MSL (Kelvin)
    const float alt_msl = (float)alt_home_ - pose_n_z;
    const float temperature_local = temperature_msl - lapse_rate * alt_msl;
    const float pressure_ratio = powf(temperature_msl / temperature_local, 5.256f);
    const float pressure_msl = 101325.0f; // pressure at MSL
    const float absolute_pressure = pressure_msl / pressure_ratio;

    // generate Gaussian noise sequence using polar form of Box-Muller transformation
    double y1;
    {
      double x1, x2, w;
      if (!baro_rnd_use_last_) {
        do {
          x1 = 2.0 * standard_normal_distribution_(random_generator_) - 1.0;
          x2 = 2.0 * standard_normal_distribution_(random_generator_) - 1.0;
          w = x1 * x1 + x2 * x2;
        } while ( w >= 1.0 );
        w = sqrt( (-2.0 * log( w ) ) / w );
        // calculate two values - the second value can be used next time because it is uncorrelated
        y1 = x1 * w;
        baro_rnd_y2_ = x2 * w;
        baro_rnd_use_last_ = true;
      } else {
        // no need to repeat the calculation - use the second value from last update
        y1 = baro_rnd_y2_;
        baro_rnd_use_last_ = false;
      }
    }

    // Apply noise and drift
    const float abs_pressure_noise = 1.0f * (float)y1;  // 1 Pa RMS noise
    baro_drift_pa_ += baro_drift_pa_per_sec_ * dt;
    const float absolute_pressure_noisy = absolute_pressure + abs_pressure_noise + baro_drift_pa_;

    // convert to hPa
    const float absolute_pressure_noisy_hpa = absolute_pressure_noisy * 0.01f;
    baro_msg_.set_absolute_pressure(absolute_pressure_noisy_hpa);

    // calculate density using an ISA model for the tropsphere (valid up to 11km above MSL)
    const float density_ratio = powf(temperature_msl / temperature_local, 4.256f);
    const float rho = 1.225f / density_ratio;

    // calculate pressure altitude including effect of pressure noise
    baro_msg_.set_pressure_altitude(alt_msl -
                                    (abs_pressure_noise + baro_drift_pa_) /
                                        (gravity_W_.Length() * rho));

    // calculate temperature in Celsius
    baro_msg_.set_temperature(temperature_local - 273.0f);

    // Fill baro msg
    baro_msg_.set_time_usec(std::chrono::duration_cast<std::chrono::microseconds>(current_time).count());

    last_pub_time_ = current_time;

    // Publish baro msg
    pub_baro_.Publish(baro_msg_);
  }
}

void BarometerPlugin::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm) {
}
