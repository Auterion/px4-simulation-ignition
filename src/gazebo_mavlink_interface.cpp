/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2015-2018 PX4 Development Team
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "gazebo_mavlink_interface.h"

#include <ignition/plugin/Register.hh>

IGNITION_ADD_PLUGIN(
    mavlink_interface::GazeboMavlinkInterface,
    ignition::gazebo::System,
    mavlink_interface::GazeboMavlinkInterface::ISystemConfigure,
    mavlink_interface::GazeboMavlinkInterface::ISystemPreUpdate,
    mavlink_interface::GazeboMavlinkInterface::ISystemPostUpdate)
using namespace mavlink_interface;

GazeboMavlinkInterface::GazeboMavlinkInterface() : 
    received_first_actuator_(false),
    namespace_(kDefaultNamespace),
    protocol_version_(2.0),
    motor_velocity_reference_pub_topic_(kDefaultMotorVelocityReferencePubTopic),
    use_propeller_pid_(false),
    use_elevator_pid_(false),
    use_left_elevon_pid_(false),
    use_right_elevon_pid_(false),
    vehicle_is_tailsitter_(false),
    send_vision_estimation_(false),
    send_odometry_(false),
    imu_sub_topic_(kDefaultImuTopic),
    opticalFlow_sub_topic_(kDefaultOpticalFlowTopic),
    irlock_sub_topic_(kDefaultIRLockTopic),
    gps_sub_topic_(kDefaultGPSTopic),
    vision_sub_topic_(kDefaultVisionTopic),
    mag_sub_topic_(kDefaultMagTopic),
    baro_sub_topic_(kDefaultBarometerTopic),
    // sensor_map_ {},
    // model_ {},
    // world_(nullptr),
    // left_elevon_joint_(nullptr),
    // right_elevon_joint_(nullptr),
    // elevator_joint_(nullptr),
    // propeller_joint_(nullptr),
    // gimbal_yaw_joint_(nullptr),
    // gimbal_pitch_joint_(nullptr),
    // gimbal_roll_joint_(nullptr),
    input_offset_ {},
    input_scaling_ {},
    zero_position_disarmed_ {},
    zero_position_armed_ {},
    input_index_ {},
    mag_updated_(false),
    groundtruth_lat_rad(0.0),
    groundtruth_lon_rad(0.0),
    groundtruth_altitude(0.0),
    hil_mode_(false),
    hil_state_level_(false)
    {
      mavlink_interface_ = std::make_shared<MavlinkInterface>();
    }

GazeboMavlinkInterface::~GazeboMavlinkInterface() {
  mavlink_interface_->close();
  // sigIntConnection_->~Connection();
  // updateConnection_->~Connection();
}

void GazeboMavlinkInterface::Configure(const ignition::gazebo::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      ignition::gazebo::EntityComponentManager &_ecm,
      ignition::gazebo::EventManager &/*_eventMgr*/) {

  namespace_.clear();
  if (_sdf->HasElement("robotNamespace")) {
    namespace_ = _sdf->Get<std::string>("robotNamespace");
  } else {
    ignerr << "[gazebo_mavlink_interface] Please specify a robotNamespace.\n";
  }

  if (_sdf->HasElement("protocol_version")) {
    protocol_version_ = _sdf->Get<float>("protocol_version");
  }

//   node_handle_ = transport::NodePtr(new transport::Node());
//   node_handle_->Init(namespace_);

  gazebo::getSdfParam<std::string>(_sdf, "motorSpeedCommandPubTopic", motor_velocity_reference_pub_topic_,
      motor_velocity_reference_pub_topic_);
  gazebo::getSdfParam<std::string>(_sdf, "imuSubTopic", imu_sub_topic_, imu_sub_topic_);
  gazebo::getSdfParam<std::string>(_sdf, "gpsSubTopic", gps_sub_topic_, gps_sub_topic_);
  gazebo::getSdfParam<std::string>(_sdf, "visionSubTopic", vision_sub_topic_, vision_sub_topic_);
  gazebo::getSdfParam<std::string>(_sdf, "opticalFlowSubTopic",
      opticalFlow_sub_topic_, opticalFlow_sub_topic_);
  gazebo::getSdfParam<std::string>(_sdf, "irlockSubTopic", irlock_sub_topic_, irlock_sub_topic_);
  gazebo::getSdfParam<std::string>(_sdf, "magSubTopic", mag_sub_topic_, mag_sub_topic_);
  gazebo::getSdfParam<std::string>(_sdf, "baroSubTopic", baro_sub_topic_, baro_sub_topic_);
  groundtruth_sub_topic_ = "/groundtruth";

  // set input_reference_ from inputs.control
  input_reference_.resize(n_out_max);
  // joints_.resize(n_out_max);
  // pids_.resize(n_out_max);
  // joint_max_errors_.resize(n_out_max);
//   for (int i = 0; i < n_out_max; ++i)
//   {
//     pids_[i].Init(0, 0, 0, 0, 0, 0, 0);
//     input_reference_[i] = 0;
//   }

//   if (_sdf->HasElement("control_channels")) {
//     sdf::ElementPtr control_channels = _sdf->GetElement("control_channels");
//     sdf::ElementPtr channel = control_channels->GetElement("channel");
//     while (channel)
//     {
//       if (channel->HasElement("input_index"))
//       {
//         int index = channel->Get<int>("input_index");
//         if (index < n_out_max)
//         {
//           input_offset_[index] = channel->Get<double>("input_offset");
//           input_scaling_[index] = channel->Get<double>("input_scaling");
//           zero_position_disarmed_[index] = channel->Get<double>("zero_position_disarmed");
//           zero_position_armed_[index] = channel->Get<double>("zero_position_armed");
//           if (channel->HasElement("joint_control_type"))
//           {
//             joint_control_type_[index] = channel->Get<std::string>("joint_control_type");
//           }
//           else
//           {
//             gzwarn << "joint_control_type[" << index << "] not specified, using velocity.\n";
//             joint_control_type_[index] = "velocity";
//           }

//           // start gz transport node handle
//           if (joint_control_type_[index] == "position_gztopic")
//           {
//             // setup publisher handle to topic
//             if (channel->HasElement("gztopic"))
//               gztopic_[index] = "~/" + model_->GetName() + channel->Get<std::string>("gztopic");
//             else
//               gztopic_[index] = "control_position_gztopic_" + std::to_string(index);
// #if GAZEBO_MAJOR_VERSION > 7 || (GAZEBO_MAJOR_VERSION == 7 && GAZEBO_MINOR_VERSION >= 4)
//             /// only gazebo 7.4 and above support Any
//             joint_control_pub_[index] = node_handle_->Advertise<gazebo::msgs::Any>(
//                 gztopic_[index]);
// #else
//             joint_control_pub_[index] = node_handle_->Advertise<gazebo::msgs::GzString>(
//                 gztopic_[index]);
// #endif
//           }

//           if (channel->HasElement("joint_name"))
//           {
//             std::string joint_name = channel->Get<std::string>("joint_name");
//             joints_[index] = model_->GetJoint(joint_name);
//           }

//           // setup joint control pid to control joint
//           if (channel->HasElement("joint_control_pid"))
//           {
//             sdf::ElementPtr pid = channel->GetElement("joint_control_pid");
//             double p = 0;
//             if (pid->HasElement("p"))
//               p = pid->Get<double>("p");
//             double i = 0;
//             if (pid->HasElement("i"))
//               i = pid->Get<double>("i");
//             double d = 0;
//             if (pid->HasElement("d"))
//               d = pid->Get<double>("d");
//             double iMax = 0;
//             if (pid->HasElement("iMax"))
//               iMax = pid->Get<double>("iMax");
//             double iMin = 0;
//             if (pid->HasElement("iMin"))
//               iMin = pid->Get<double>("iMin");
//             double cmdMax = 0;
//             if (pid->HasElement("cmdMax"))
//               cmdMax = pid->Get<double>("cmdMax");
//             double cmdMin = 0;
//             if (pid->HasElement("cmdMin"))
//               cmdMin = pid->Get<double>("cmdMin");
//             if (pid->HasElement("errMax")) {
//               joint_max_errors_[index] = pid->Get<double>("errMax");
//             }
//             pids_[index].Init(p, i, d, iMax, iMin, cmdMax, cmdMin);
//           }
//         }
//         else
//         {
//           gzerr << "input_index[" << index << "] out of range, not parsing.\n";
//         }
//       }
//       else
//       {
//         gzerr << "no input_index, not parsing.\n";
//         break;
//       }
//       channel = channel->GetNextElement("channel");
//     }
//   }

  if(_sdf->HasElement("hil_mode"))
  {
    hil_mode_ = _sdf->Get<bool>("hil_mode");
    mavlink_interface_->SetHILMode(hil_mode_);
  }

  if(_sdf->HasElement("hil_state_level"))
  {
    hil_state_level_ = _sdf->Get<bool>("hil_state_level");
    mavlink_interface_->SetHILStateLevel(hil_state_level_);
  }

  bool serial_enabled=false;
  if(_sdf->HasElement("serialEnabled"))
  {
    serial_enabled = _sdf->Get<bool>("serialEnabled");
    mavlink_interface_->SetSerialEnabled(serial_enabled);
  }

  bool use_tcp = false;
  if (!serial_enabled && _sdf->HasElement("use_tcp"))
  {
    use_tcp = _sdf->Get<bool>("use_tcp");
    mavlink_interface_->SetUseTcp(use_tcp);
  }
  ignmsg << "Connecting to PX4 SITL using " << (serial_enabled ? "serial" : (use_tcp ? "TCP" : "UDP")) << "\n";

  if (!hil_mode_ && _sdf->HasElement("enable_lockstep"))
  {
    enable_lockstep_ = _sdf->Get<bool>("enable_lockstep");
    mavlink_interface_->SetEnableLockstep(enable_lockstep_);
  }
  ignmsg << "Lockstep is " << (enable_lockstep_ ? "enabled" : "disabled") << "\n";


  // When running in lockstep, we can run the simulation slower or faster than
  // realtime. The speed can be set using the env variable PX4_SIM_SPEED_FACTOR.
  if (enable_lockstep_)
  {
    const char *speed_factor_str = std::getenv("PX4_SIM_SPEED_FACTOR");
    if (speed_factor_str)
    {
      speed_factor_ = std::atof(speed_factor_str);
      if (!std::isfinite(speed_factor_) || speed_factor_ <= 0.0)
      {
        ignerr << "Invalid speed factor '" << speed_factor_str << "', aborting\n";
        abort();
      }
    }
    ignmsg << "Speed factor set to: " << speed_factor_ << "\n";

    // boost::any param;
// #if GAZEBO_MAJOR_VERSION >= 8
//     physics::PresetManagerPtr presetManager = world_->PresetMgr();
// #else
//     physics::PresetManagerPtr presetManager = world_->GetPresetManager();
// #endif
//     presetManager->CurrentProfile("default_physics");

//     // We currently need to have the real_time_update_rate at a multiple of 250 Hz for lockstep.
//     // Also, the max_step_size needs to match this (e.g. 0.004 s at 250 Hz or 0.002 s at 500 Hz).
//     // Therefore we check these params and abort if they won't work.

//     presetManager->GetCurrentProfileParam("real_time_update_rate", param);
//     double real_time_update_rate = boost::any_cast<double>(param);
//     const int real_time_update_rate_int = static_cast<int>(real_time_update_rate + 0.5);

//     if (real_time_update_rate_int % 250 != 0)
//     {
//       gzerr << "real_time_update_rate is " << real_time_update_rate_int
//             << " but needs to be multiple of 250 Hz, absorting.\n";
//       abort();
//     }

//     presetManager->GetCurrentProfileParam("max_step_size", param);
//     const double max_step_size = boost::any_cast<double>(param);
//     if (1.0 / real_time_update_rate != max_step_size)
//     {
//       gzerr << "max_step_size of " << max_step_size
//             << " s does not match real_time_update_rate of "
//             << real_time_update_rate << ", aborting.\n";
//       abort();
//     }

//     update_skip_factor_ = real_time_update_rate_int / 250;

//     // Adapt the real_time_update_rate according to the speed
//     // that we ask for in the env variable.
//     real_time_update_rate *= speed_factor_;
//     presetManager->SetCurrentProfileParam("real_time_update_rate", real_time_update_rate);
  }

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  // updateConnection_ = event::Events::ConnectWorldUpdateBegin(
  //     boost::bind(&GazeboMavlinkInterface::OnUpdate, this, _1));

  // // Listen to Ctrl+C / SIGINT.
  // sigIntConnection_ = event::Events::ConnectSigInt(
  //     boost::bind(&GazeboMavlinkInterface::onSigInt, this));

  // Subscribe to messages of other plugins.
  node.Subscribe("/world/quadcopter/model/X3/link/base_link/sensor/imu_sensor/imu", &GazeboMavlinkInterface::ImuCallback, this);
//   imu_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + imu_sub_topic_, &GazeboMavlinkInterface::ImuCallback, this);
//   opticalFlow_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + opticalFlow_sub_topic_, &GazeboMavlinkInterface::OpticalFlowCallback, this);
//   irlock_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + irlock_sub_topic_, &GazeboMavlinkInterface::IRLockCallback, this);
//   gps_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + gps_sub_topic_, &GazeboMavlinkInterface::GpsCallback, this);
//   groundtruth_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + groundtruth_sub_topic_, &GazeboMavlinkInterface::GroundtruthCallback, this);
//   vision_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + vision_sub_topic_, &GazeboMavlinkInterface::VisionCallback, this);
//   mag_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + mag_sub_topic_, &GazeboMavlinkInterface::MagnetometerCallback, this);
//   baro_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + baro_sub_topic_, &GazeboMavlinkInterface::BarometerCallback, this);

//   // Get the model links
//   auto links = model_->GetLinks();

//   // Create subscriptions to the distance sensors
//   CreateSensorSubscription(&GazeboMavlinkInterface::LidarCallback, this, links);
//   CreateSensorSubscription(&GazeboMavlinkInterface::SonarCallback, this, links);

//   // Publish gazebo's motor_speed message
//   motor_velocity_reference_pub_ = node_handle_->Advertise<mav_msgs::msgs::CommandMotorSpeed>("~/" + model_->GetName() + motor_velocity_reference_pub_topic_, 1);

// #if GAZEBO_MAJOR_VERSION >= 9
//   last_time_ = world_->SimTime();
//   last_imu_time_ = world_->SimTime();
//   gravity_W_ = world_->Gravity();
// #else
//   last_time_ = world_->GetSimTime();
//   last_imu_time_ = world_->GetSimTime();
//   gravity_W_ = ignitionFromGazeboMath(world_->GetPhysicsEngine()->GetGravity());
// #endif

  // This doesn't seem to be used anywhere but we leave it here
  // for potential compatibility
  if (_sdf->HasElement("imu_rate")) {
    imu_update_interval_ = 1 / _sdf->Get<int>("imu_rate");
  }

  if (_sdf->HasElement("mavlink_addr")) {
    std::string mavlink_addr_str = _sdf->Get<std::string>("mavlink_addr");
    if (mavlink_addr_str != "INADDR_ANY") {
      mavlink_interface_->SetMavlinkAddr(mavlink_addr_str);
    }
  }

// #if GAZEBO_MAJOR_VERSION >= 9
//   auto worldName = world_->Name();
// #else
//   auto worldName = world_->GetName();
// #endif

  int mavlink_udp_port;
  if (_sdf->HasElement("mavlink_udp_port")) {
    mavlink_udp_port = _sdf->Get<int>("mavlink_udp_port");
    //TODO: Set Malvink UDP port
  }
  // model_param(worldName, model_->GetName(), "mavlink_udp_port", mavlink_udp_port);
  mavlink_interface_->SetMavlinkUdpPort(mavlink_udp_port);

  int mavlink_tcp_port;
  if (_sdf->HasElement("mavlink_tcp_port")) {
    mavlink_tcp_port = _sdf->Get<int>("mavlink_tcp_port");
  }
  // model_param(worldName, model_->GetName(), "mavlink_tcp_port", mavlink_tcp_port);
  mavlink_interface_->SetMavlinkTcpPort(mavlink_tcp_port);

  if (_sdf->HasElement("qgc_addr")) {
    std::string qgc_addr = _sdf->Get<std::string>("qgc_addr");
    if (qgc_addr != "INADDR_ANY") {
      mavlink_interface_->SetQgcAddr(qgc_addr);
    }
  }
  if (_sdf->HasElement("qgc_udp_port")) {
    int qgc_udp_port = _sdf->Get<int>("qgc_udp_port");
    mavlink_interface_->SetQgcUdpPort(qgc_udp_port);
  }

  if (_sdf->HasElement("sdk_addr")) {
    std::string sdk_addr = _sdf->Get<std::string>("sdk_addr");
    if (sdk_addr != "INADDR_ANY") {
      mavlink_interface_->SetSdkAddr(sdk_addr);
    }
  }
  if (_sdf->HasElement("sdk_udp_port")) {
    int sdk_udp_port = _sdf->Get<int>("sdk_udp_port");
    mavlink_interface_->SetSdkUdpPort(sdk_udp_port);
  }

  if (serial_enabled_) {
    // Set up serial interface
    if(_sdf->HasElement("serialDevice"))
    {
      std::string device = _sdf->Get<std::string>("serialDevice");
    }

    if (_sdf->HasElement("baudRate")) {
      int baudrate = _sdf->Get<int>("baudRate");
    }
  }

  if(_sdf->HasElement("send_vision_estimation"))
  {
    send_vision_estimation_ = _sdf->Get<bool>("send_vision_estimation");
  }

  if(_sdf->HasElement("send_odometry"))
  {
    send_odometry_ = _sdf->Get<bool>("send_odometry");
  }

  mavlink_status_t* chan_state = mavlink_get_channel_status(MAVLINK_COMM_0);

  // set the Mavlink protocol version to use on the link
  if (protocol_version_ == 2.0) {
    chan_state->flags &= ~(MAVLINK_STATUS_FLAG_OUT_MAVLINK1);
    ignmsg << "Using MAVLink protocol v2.0\n";
  }
  else if (protocol_version_ == 1.0) {
    chan_state->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    ignmsg << "Using MAVLink protocol v1.0\n";
  }
  else {
    ignerr << "Unkown protocol version! Using v" << protocol_version_ << "by default \n";
  }
  mavlink_interface_->Load();
}

void GazeboMavlinkInterface::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
  ignition::gazebo::EntityComponentManager &_ecm) {
  std::unique_lock<std::mutex> lock(last_imu_message_mutex_);

  if (previous_imu_seq_ > 0) {
    // while (previous_imu_seq_ == last_imu_message_.seq() && IsRunning()) {
      last_imu_message_cond_.wait_for(lock, std::chrono::milliseconds(10));
    // }
  }

//   previous_imu_seq_ = last_imu_message_.seq();

  // Always run at 250 Hz. At 500 Hz, the skip factor should be 2, at 1000 Hz 4.
  if (!(previous_imu_seq_ % update_skip_factor_ == 0)) {
    return;
  }

// #if GAZEBO_MAJOR_VERSION >= 9
//   common::Time current_time = world_->SimTime();
// #else
//   common::Time current_time = world_->GetSimTime();
// #endif
//   double dt = (current_time - last_time_).Double();
  double dt;

  bool close_conn_ = false;

  if (hil_mode_) {
    mavlink_interface_->pollFromQgcAndSdk();
  } else {
    mavlink_interface_->pollForMAVLinkMessages();
  }

  // Always send Gyro and Accel data at full rate (= sim update rate)
  SendSensorMessages();

  // Send groudntruth at full rate
  SendGroundTruth();

  if (close_conn_) { // close connection if required
    mavlink_interface_->close();
  }

  handle_control(dt);

  if (received_first_actuator_) {
    // mav_msgs::msgs::CommandMotorSpeed turning_velocities_msg;

    for (int i = 0; i < input_reference_.size(); i++) {
      if (last_actuator_time_ == 0 || dt > 0.2) {
        // turning_velocities_msg.add_motor_speed(0);
      } else {
        // turning_velocities_msg.add_motor_speed(input_reference_[i]);
      }
    }
    // TODO Add timestamp and Header
    // turning_velocities_msg->header.stamp.sec = current_time.sec;
    // turning_velocities_msg->header.stamp.nsec = current_time.nsec;

    // motor_velocity_reference_pub_->Publish(turning_velocities_msg);
  }

}

void GazeboMavlinkInterface::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm) {
}

void GazeboMavlinkInterface::ImuCallback(const ignition::msgs::IMU &_msg) {
  std::unique_lock<std::mutex> lock(last_imu_message_mutex_);

  // const int64_t diff = imu_message->seq() - last_imu_message_.seq();
  // if (diff != 1 && imu_message->seq() != 0)
  // {
  //   gzerr << "Skipped " << (diff - 1) << " IMU samples (presumably CPU usage is too high)\n";
  // }

  last_imu_message_ = _msg;
  lock.unlock();
  last_imu_message_cond_.notify_one();
}

void GazeboMavlinkInterface::SendSensorMessages() {
  ignition::math::Quaterniond q_gr = ignition::math::Quaterniond(
    last_imu_message_.orientation().w(),
    last_imu_message_.orientation().x(),
    last_imu_message_.orientation().y(),
    last_imu_message_.orientation().z());

  bool should_send_imu = false;
  if (!enable_lockstep_) {
//     ignition::common::Time current_time = world_->SimTime();
       ignition::common::Time current_time; //TODO: How do you get current time?

    double dt = (current_time - last_imu_time_).Double();

    if (imu_update_interval_!=0 && dt >= imu_update_interval_) {
      should_send_imu = true;
      last_imu_time_ = current_time;
    }
  }

  mavlink_hil_sensor_t sensor_msg;
//   sensor_msg.time_usec = world_->SimTime().Double() * 1e6;

  // send always accel and gyro data (not dependent of the bitmask)
  // required so to keep the timestamps on sync and the lockstep can
  // work properly
  ignition::math::Vector3d accel_b = q_br.RotateVector(ignition::math::Vector3d(
    last_imu_message_.linear_acceleration().x(),
    last_imu_message_.linear_acceleration().y(),
    last_imu_message_.linear_acceleration().z()));

  ignition::math::Vector3d gyro_b = q_br.RotateVector(ignition::math::Vector3d(
    last_imu_message_.angular_velocity().x(),
    last_imu_message_.angular_velocity().y(),
    last_imu_message_.angular_velocity().z()));

  sensor_msg.xacc = accel_b.X();
  sensor_msg.yacc = accel_b.Y();
  sensor_msg.zacc = accel_b.Z();
  sensor_msg.xgyro = gyro_b.X();
  sensor_msg.ygyro = gyro_b.Y();
  sensor_msg.zgyro = gyro_b.Z();

  // sensor_msg.fields_updated = SensorSource::ACCEL | SensorSource::GYRO;

  // send only mag data
  if (mag_updated_) {
    ignition::math::Quaterniond q_gb = q_gr*q_br.Inverse();
    ignition::math::Quaterniond q_nb = q_ng*q_gb;

    ignition::math::Vector3d mag_b = q_nb.RotateVectorReverse(mag_n_);

    sensor_msg.xmag = mag_b.X();
    sensor_msg.ymag = mag_b.Y();
    sensor_msg.zmag = mag_b.Z();
    // sensor_msg.fields_updated = sensor_msg.fields_updated | SensorSource::MAG;

    mag_updated_ = false;
  }

  if (!hil_mode_ || (hil_mode_ && !hil_state_level_)) {
    mavlink_message_t msg;
    mavlink_msg_hil_sensor_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &sensor_msg);
    mavlink_interface_->send_mavlink_message(&msg);
  }
}

void GazeboMavlinkInterface::SendGroundTruth()
{
  // ground truth
  ignition::math::Quaterniond q_gr = ignition::math::Quaterniond(
    last_imu_message_.orientation().w(),
    last_imu_message_.orientation().x(),
    last_imu_message_.orientation().y(),
    last_imu_message_.orientation().z());

  ignition::math::Quaterniond q_gb = q_gr*q_br.Inverse();
  ignition::math::Quaterniond q_nb = q_ng*q_gb;

  // ignition::math::Vector3d vel_b = q_br.RotateVector(model_->RelativeLinearVel());
  // ignition::math::Vector3d vel_n = q_ng.RotateVector(model_->WorldLinearVel());
  // ignition::math::Vector3d omega_nb_b = q_br.RotateVector(model_->RelativeAngularVel());
  ignition::math::Vector3d vel_b;
  ignition::math::Vector3d vel_n;
  ignition::math::Vector3d omega_nb_b;

  // ignition::math::Vector3d accel_true_b = q_br.RotateVector(model_->RelativeLinearAccel());
  ignition::math::Vector3d accel_true_b; //TODO: Get model pointer

  // send ground truth
  mavlink_hil_state_quaternion_t hil_state_quat;
// #if GAZEBO_MAJOR_VERSION >= 9
//   hil_state_quat.time_usec = world_->SimTime().Double() * 1e6;
// #else
//   hil_state_quat.time_usec = world_->GetSimTime().Double() * 1e6;
// #endif
  hil_state_quat.attitude_quaternion[0] = q_nb.W();
  hil_state_quat.attitude_quaternion[1] = q_nb.X();
  hil_state_quat.attitude_quaternion[2] = q_nb.Y();
  hil_state_quat.attitude_quaternion[3] = q_nb.Z();

  hil_state_quat.rollspeed = omega_nb_b.X();
  hil_state_quat.pitchspeed = omega_nb_b.Y();
  hil_state_quat.yawspeed = omega_nb_b.Z();

  hil_state_quat.lat = groundtruth_lat_rad * 180 / M_PI * 1e7;
  hil_state_quat.lon = groundtruth_lon_rad * 180 / M_PI * 1e7;
  hil_state_quat.alt = groundtruth_altitude * 1000;

  hil_state_quat.vx = vel_n.X() * 100;
  hil_state_quat.vy = vel_n.Y() * 100;
  hil_state_quat.vz = vel_n.Z() * 100;

  // assumed indicated airspeed due to flow aligned with pitot (body x)
  hil_state_quat.ind_airspeed = vel_b.X();

// #if GAZEBO_MAJOR_VERSION >= 9
//   hil_state_quat.true_airspeed = model_->WorldLinearVel().Length() * 100;  //no wind simulated
// #else
//   hil_state_quat.true_airspeed = model_->GetWorldLinearVel().GetLength() * 100;  //no wind simulated
// #endif

  hil_state_quat.xacc = accel_true_b.X() * 1000;
  hil_state_quat.yacc = accel_true_b.Y() * 1000;
  hil_state_quat.zacc = accel_true_b.Z() * 1000;

  if (!hil_mode_ || (hil_mode_ && hil_state_level_)) {
    mavlink_message_t msg;
    mavlink_msg_hil_state_quaternion_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &hil_state_quat);
    mavlink_interface_->send_mavlink_message(&msg);
  }
}

void GazeboMavlinkInterface::handle_control(double _dt)
{
  // set joint positions
  for (int i = 0; i < input_reference_.size(); i++) {
    // if (joints_[i]) {
    //   double target = input_reference_[i];
    //   if (joint_control_type_[i] == "velocity")
    //   {
//         double current = joints_[i]->GetVelocity(0);
//         double err = current - target;
//         double force = pids_[i].Update(err, _dt);
//         joints_[i]->SetForce(0, force);
      // }
      // else if (joint_control_type_[i] == "position")
      // {

// #if GAZEBO_MAJOR_VERSION >= 9
//         double current = joints_[i]->Position(0);
// #else
//         double current = joints_[i]->GetAngle(0).Radian();
// #endif

//         double err = current - target;
//         err = std::max(std::min(err, 0.2), -0.2);
//         double force = pids_[i].Update(err, _dt);
//         joints_[i]->SetForce(0, force);
//       }
//       else if (joint_control_type_[i] == "position_gztopic")
//       {
//      #if GAZEBO_MAJOR_VERSION > 7 || (GAZEBO_MAJOR_VERSION == 7 && GAZEBO_MINOR_VERSION >= 4)
//         /// only gazebo 7.4 and above support Any
//         gazebo::msgs::Any m;
//         m.set_type(gazebo::msgs::Any_ValueType_DOUBLE);
//         m.set_double_value(target);
//      #else
//         std::stringstream ss;
//         gazebo::msgs::GzString m;
//         ss << target;
//         m.set_data(ss.str());
//      #endif
//         joint_control_pub_[i]->Publish(m);
//       }
//       else if (joint_control_type_[i] == "position_kinematic")
//       {
//         /// really not ideal if your drone is moving at all,
//         /// mixing kinematic updates with dynamics calculation is
//         /// non-physical.
//      #if GAZEBO_MAJOR_VERSION >= 6
//         joints_[i]->SetPosition(0, input_reference_[i]);
//      #else
//         joints_[i]->SetAngle(0, input_reference_[i]);
//      #endif
//       }
//       else
//       {
//         gzerr << "joint_control_type[" << joint_control_type_[i] << "] undefined.\n";
//       }
    // }
  }
}

bool GazeboMavlinkInterface::IsRunning()
{
// #if GAZEBO_MAJOR_VERSION >= 8
//     return world_->Running();
// #else
//     return world_->GetRunning();
// #endif
  return true; //TODO;
}
