/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 * Copyright (C) 2017-2018 PX4 Pro Development Team
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
/**
 * @brief GPS Plugin
 *
 * This plugin publishes GPS and Groundtruth data to be used and propagated
 *
 * @author Amy Wagoner <arwagoner@gmail.com>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */


#ifndef GPS_PLUGIN_HH_
#define GPS_PLUGIN_HH_

#include <random>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/System.hh>

#include <ignition/transport/Node.hh>

#include <msgs/SITLGps.pb.h>
#include <common.h>

static constexpr double kDefaultUpdateRate = 5.0;               // hz
static constexpr double kDefaultGpsXYRandomWalk = 2.0;          // (m/s) / sqrt(hz)
static constexpr double kDefaultGpsZRandomWalk = 4.0;           // (m/s) / sqrt(hz)
static constexpr double kDefaultGpsXYNoiseDensity = 2.0e-4;     // (m) / sqrt(hz)
static constexpr double kDefaultGpsZNoiseDensity = 4.0e-4;      // (m) / sqrt(hz)
static constexpr double kDefaultGpsVXYNoiseDensity = 0.2;       // (m/s) / sqrt(hz)
static constexpr double kDefaultGpsVZNoiseDensity = 0.4;        // (m/s) / sqrt(hz)

namespace gps_plugin
{
    class IGNITION_GAZEBO_VISIBLE GpsPlugin:
    // This is class a system.
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPreUpdate,
    public ignition::gazebo::ISystemPostUpdate
    {
    public: GpsPlugin();
    public: ~GpsPlugin() override;
    public: void Configure(const ignition::gazebo::Entity &_entity,
                            const std::shared_ptr<const sdf::Element> &_sdf,
                            ignition::gazebo::EntityComponentManager &_ecm,
                            ignition::gazebo::EventManager &/*_eventMgr*/);
    public: void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;
    public: void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                const ignition::gazebo::EntityComponentManager &_ecm) override;
    private:
        std::chrono::steady_clock::duration last_pub_time_{0};

        std::string namespace_;
        std::string gps_id_;
        std::default_random_engine random_generator_;
        std::normal_distribution<float> standard_normal_distribution_;

        bool gps_noise_{false};

        std::string model_name_;

        ignition::gazebo::Model model_{ignition::gazebo::kNullEntity};
        ignition::gazebo::Entity model_link_{ignition::gazebo::kNullEntity};

        ignition::transport::Node node;
        ignition::transport::Node::Publisher pub_gps_;

        std::string gps_topic_;
        double update_rate_{1.0};


        std::mutex data_mutex_;

        // Home defaults to Zurich Irchel Park
        // @note The home position can be specified using the environment variables:
        // PX4_HOME_LAT, PX4_HOME_LON, and PX4_HOME_ALT
        double lat_home_ = kDefaultHomeLatitude;
        double lon_home_ = kDefaultHomeLongitude;
        double alt_home_ = kDefaultHomeAltitude;
        double world_latitude_ = 0.0;
        double world_longitude_ = 0.0;
        double world_altitude_ = 0.0;

        // gps delay related
        static constexpr double gps_delay_ = 0.12;           // 120 ms
        static constexpr int gps_buffer_size_max_ = 1000;
        std::queue<sensor_msgs::msgs::SITLGps> gps_delay_buffer_;

        ignition::math::Vector3d gps_bias_;
        ignition::math::Vector3d noise_gps_pos_;
        ignition::math::Vector3d noise_gps_vel_;
        ignition::math::Vector3d random_walk_gps_;
        ignition::math::Vector3d gravity_W_;
        ignition::math::Vector3d velocity_prev_W_;

        // gps noise parameters
        double std_xy_;    // meters
        double std_z_;     // meters
        std::default_random_engine rand_;
        std::normal_distribution<float> randn_;
        static constexpr const double gps_corellation_time_ = 60.0;    // s
        double gps_xy_random_walk_;
        double gps_z_random_walk_;
        double gps_xy_noise_density_;
        double gps_z_noise_density_;
        double gps_vxy_noise_density_;
        double gps_vz_noise_density_;
    };
}

#endif
