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
    baro_updated_(false),
    diff_press_updated_(false),
    groundtruth_lat_rad(0.0),
    groundtruth_lon_rad(0.0),
    groundtruth_altitude(0.0),
    mavlink_udp_port_(kDefaultMavlinkUdpPort),
    mavlink_tcp_port_(kDefaultMavlinkTcpPort),
    simulator_socket_fd_(0),
    simulator_tcp_client_fd_(0),
    use_tcp_(false),
    qgc_udp_port_(kDefaultQGCUdpPort),
    sdk_udp_port_(kDefaultSDKUdpPort),
    remote_qgc_addr_ {},
    local_qgc_addr_ {},
    remote_sdk_addr_ {},
    local_sdk_addr_ {},
    qgc_socket_fd_(0),
    sdk_socket_fd_(0),
    serial_enabled_(false),
    tx_q {},
    rx_buf {},
    m_status {},
    m_buffer {},
    io_service(),
    serial_dev(io_service),
    device_(kDefaultDevice),
    baudrate_(kDefaultBaudRate),
    hil_mode_(false),
    hil_state_level_(false)
    {}

GazeboMavlinkInterface::~GazeboMavlinkInterface() {
}

void GazeboMavlinkInterface::Configure(const ignition::gazebo::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      ignition::gazebo::EntityComponentManager &_ecm,
      ignition::gazebo::EventManager &/*_eventMgr*/) {

//   model_ = _model;
//   world_ = model_->GetWorld();

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
//             if (joints_[index] == nullptr)
//             {
//               gzwarn << "joint [" << joint_name << "] not found for channel["
//                     << index << "] no joint control for this channel.\n";
//             }
//             else
//             {
//               gzdbg << "joint [" << joint_name << "] found for channel["
//                     << index << "] joint control active for this channel.\n";
//             }
//           }
//           else
//           {
//             gzdbg << "<joint_name> not found for channel[" << index
//                   << "] no joint control will be performed for this channel.\n";
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
  }

  if(_sdf->HasElement("hil_state_level"))
  {
    hil_state_level_ = _sdf->Get<bool>("hil_state_level");
  }

  if(_sdf->HasElement("serialEnabled"))
  {
    serial_enabled_ = _sdf->Get<bool>("serialEnabled");
  }

  if (!serial_enabled_ && _sdf->HasElement("use_tcp"))
  {
    use_tcp_ = _sdf->Get<bool>("use_tcp");
  }
  ignmsg << "Connecting to PX4 SITL using " << (serial_enabled_ ? "serial" : (use_tcp_ ? "TCP" : "UDP")) << "\n";

  if (!hil_mode_ && _sdf->HasElement("enable_lockstep"))
  {
    enable_lockstep_ = _sdf->Get<bool>("enable_lockstep");
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

  mavlink_addr_ = htonl(INADDR_ANY);
  if (_sdf->HasElement("mavlink_addr")) {
    std::string mavlink_addr_str = _sdf->Get<std::string>("mavlink_addr");
    if (mavlink_addr_str != "INADDR_ANY") {
      mavlink_addr_ = inet_addr(mavlink_addr_str.c_str());
      if (mavlink_addr_ == INADDR_NONE) {
        ignerr << "Invalid mavlink_addr: " << mavlink_addr_str << ", aborting\n";
        abort();
      }
    }
  }

// #if GAZEBO_MAJOR_VERSION >= 9
//   auto worldName = world_->Name();
// #else
//   auto worldName = world_->GetName();
// #endif

  if (_sdf->HasElement("mavlink_udp_port")) {
    mavlink_udp_port_ = _sdf->Get<int>("mavlink_udp_port");
  }
//   model_param(worldName, model_->GetName(), "mavlink_udp_port", mavlink_udp_port_);

  if (_sdf->HasElement("mavlink_tcp_port")) {
    mavlink_tcp_port_ = _sdf->Get<int>("mavlink_tcp_port");
  }
//   model_param(worldName, model_->GetName(), "mavlink_tcp_port", mavlink_tcp_port_);

  local_qgc_addr_.sin_port = 0;
  if (_sdf->HasElement("qgc_addr")) {
    std::string qgc_addr = _sdf->Get<std::string>("qgc_addr");
    if (qgc_addr != "INADDR_ANY") {
      local_qgc_addr_.sin_port = inet_addr(qgc_addr.c_str());
      if (local_qgc_addr_.sin_port == 0) {
        ignerr << "Invalid qgc_addr: " << qgc_addr << ", aborting\n";
        abort();
      }
    }
  }
  if (_sdf->HasElement("qgc_udp_port")) {
    qgc_udp_port_ = _sdf->Get<int>("qgc_udp_port");
  }

  local_sdk_addr_.sin_port = 0;
  if (_sdf->HasElement("sdk_addr")) {
    std::string sdk_addr = _sdf->Get<std::string>("sdk_addr");
    if (sdk_addr != "INADDR_ANY") {
      local_sdk_addr_.sin_port = inet_addr(sdk_addr.c_str());
      if (local_sdk_addr_.sin_port == 0) {
        ignerr << "Invalid sdk_addr: " << sdk_addr << ", aborting\n";
        abort();
      }
    }
  }
  if (_sdf->HasElement("sdk_udp_port")) {
    sdk_udp_port_ = _sdf->Get<int>("sdk_udp_port");
  }

  if (hil_mode_) {

    local_qgc_addr_.sin_family = AF_INET;
    local_qgc_addr_.sin_port = htons(0);
    local_qgc_addr_len_ = sizeof(local_qgc_addr_);

    remote_qgc_addr_.sin_family = AF_INET;
    remote_qgc_addr_.sin_port = htons(qgc_udp_port_);
    remote_qgc_addr_len_ = sizeof(remote_qgc_addr_);

    local_sdk_addr_.sin_family = AF_INET;
    local_sdk_addr_.sin_port = htons(0);
    local_sdk_addr_len_ = sizeof(local_sdk_addr_);

    remote_sdk_addr_.sin_family = AF_INET;
    remote_sdk_addr_.sin_port = htons(sdk_udp_port_);
    remote_sdk_addr_len_ = sizeof(remote_sdk_addr_);

    if ((qgc_socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
      ignerr << "Creating QGC UDP socket failed: " << strerror(errno) << ", aborting\n";
      abort();
    }

    if (bind(qgc_socket_fd_, (struct sockaddr *)&local_qgc_addr_, local_qgc_addr_len_) < 0) {
      ignerr << "QGC UDP bind failed: " << strerror(errno) << ", aborting\n";
      abort();
    }

    if ((sdk_socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
      ignerr << "Creating SDK UDP socket failed: " << strerror(errno) << ", aborting\n";
      abort();
    }

    if (bind(sdk_socket_fd_, (struct sockaddr *)&local_sdk_addr_, local_sdk_addr_len_) < 0) {
      ignerr << "SDK UDP bind failed: " << strerror(errno) << ", aborting\n";
      abort();
    }

  }

  if (serial_enabled_) {
    // Set up serial interface
    if(_sdf->HasElement("serialDevice"))
    {
      device_ = _sdf->Get<std::string>("serialDevice");
    }

    if (_sdf->HasElement("baudRate")) {
      baudrate_ = _sdf->Get<int>("baudRate");
    }
    io_service.post(std::bind(&GazeboMavlinkInterface::do_read, this));

    // run io_service for async io
    io_thread = std::thread([this] () {
      io_service.run();
    });
    open();

  } else {
    memset((char *)&remote_simulator_addr_, 0, sizeof(remote_simulator_addr_));
    remote_simulator_addr_.sin_family = AF_INET;
    remote_simulator_addr_len_ = sizeof(remote_simulator_addr_);

    memset((char *)&local_simulator_addr_, 0, sizeof(local_simulator_addr_));
    local_simulator_addr_.sin_family = AF_INET;
    local_simulator_addr_len_ = sizeof(local_simulator_addr_);

    if (use_tcp_) {

      local_simulator_addr_.sin_addr.s_addr = htonl(mavlink_addr_);
      local_simulator_addr_.sin_port = htons(mavlink_tcp_port_);

      if ((simulator_socket_fd_ = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        ignerr << "Creating TCP socket failed: " << strerror(errno) << ", aborting\n";
        abort();
      }

      int yes = 1;
      int result = setsockopt(simulator_socket_fd_, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
      if (result != 0) {
        ignerr << "setsockopt failed: " << strerror(errno) << ", aborting\n";
        abort();
      }

      struct linger nolinger {};
      nolinger.l_onoff = 1;
      nolinger.l_linger = 0;

      result = setsockopt(simulator_socket_fd_, SOL_SOCKET, SO_LINGER, &nolinger, sizeof(nolinger));
      if (result != 0) {
        ignerr << "setsockopt failed: " << strerror(errno) << ", aborting\n";
        abort();
      }

      // The socket reuse is necessary for reconnecting to the same address
      // if the socket does not close but gets stuck in TIME_WAIT. This can happen
      // if the server is suddenly closed, for example, if the robot is deleted in gazebo.
      int socket_reuse = 1;
      result = setsockopt(simulator_socket_fd_, SOL_SOCKET, SO_REUSEADDR, &socket_reuse, sizeof(socket_reuse));
      if (result != 0) {
        ignerr << "setsockopt failed: " << strerror(errno) << ", aborting\n";
        abort();
      }

      // Same as above but for a given port
      result = setsockopt(simulator_socket_fd_, SOL_SOCKET, SO_REUSEPORT, &socket_reuse, sizeof(socket_reuse));
      if (result != 0) {
        ignerr << "setsockopt failed: " << strerror(errno) << ", aborting\n";
        abort();
      }

      // set socket to non-blocking
      result = fcntl(simulator_socket_fd_, F_SETFL, O_NONBLOCK);
      if (result == -1) {
        ignerr << "setting socket to non-blocking failed: " << strerror(errno) << ", aborting\n";
        abort();
      }

      if (bind(simulator_socket_fd_, (struct sockaddr *)&local_simulator_addr_, local_simulator_addr_len_) < 0) {
        ignerr << "bind failed: " << strerror(errno) << ", aborting\n";
        abort();
      }

      errno = 0;
      if (listen(simulator_socket_fd_, 0) < 0) {
        ignerr << "listen failed: " << strerror(errno) << ", aborting\n";
        abort();
      }

      memset(fds_, 0, sizeof(fds_));
      fds_[LISTEN_FD].fd = simulator_socket_fd_;
      fds_[LISTEN_FD].events = POLLIN; // only listens for new connections on tcp

    } else {
      remote_simulator_addr_.sin_addr.s_addr = mavlink_addr_;
      remote_simulator_addr_.sin_port = htons(mavlink_udp_port_);

      local_simulator_addr_.sin_addr.s_addr = htonl(INADDR_ANY);
      local_simulator_addr_.sin_port = htons(0);

      if ((simulator_socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        ignerr << "Creating UDP socket failed: " << strerror(errno) << ", aborting\n";
        abort();
      }

      // set socket to non-blocking
      int result = fcntl(simulator_socket_fd_, F_SETFL, O_NONBLOCK);
      if (result == -1) {
        ignerr << "setting socket to non-blocking failed: " << strerror(errno) << ", aborting\n";
        abort();
      }

      if (bind(simulator_socket_fd_, (struct sockaddr *)&local_simulator_addr_, local_simulator_addr_len_) < 0) {
        ignerr << "bind failed: " << strerror(errno) << ", aborting\n";
        abort();
      }

      memset(fds_, 0, sizeof(fds_));
      fds_[CONNECTION_FD].fd = simulator_socket_fd_;
      fds_[CONNECTION_FD].events = POLLIN | POLLOUT; // read/write
    }
  }

  if(_sdf->HasElement("vehicle_is_tailsitter"))
  {
    vehicle_is_tailsitter_ = _sdf->Get<bool>("vehicle_is_tailsitter");
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

  standard_normal_distribution_ = std::normal_distribution<float>(0.0f, 1.0f);

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

  double dt = std::chrono::duration<double>(_info.dt).count();

  close_conn_ = false;
  if (hil_mode_) {
    pollFromQgcAndSdk();
  } else {
    pollForMAVLinkMessages();
  }

  // Always send Gyro and Accel data at full rate (= sim update rate)
  SendSensorMessages();

  // Send groudntruth at full rate
  SendGroundTruth();

  if (close_conn_) { // close connection if required
    close();
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

void GazeboMavlinkInterface::forward_mavlink_message(const mavlink_message_t *message)
{
  if (gotSigInt_) {
    return;
  }

  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  int packetlen = mavlink_msg_to_send_buffer(buffer, message);
  ssize_t len;
  if (qgc_socket_fd_ > 0) {
    len = sendto(qgc_socket_fd_, buffer, packetlen, 0, (struct sockaddr *)&remote_qgc_addr_, remote_qgc_addr_len_);

    if (len <= 0)
    {
      ignerr << "Failed sending mavlink message to QGC: " << strerror(errno) << "\n";
    }
  }

  if (sdk_socket_fd_ > 0) {
    len = sendto(sdk_socket_fd_, buffer, packetlen, 0, (struct sockaddr *)&remote_sdk_addr_, remote_sdk_addr_len_);
    if (len <= 0)
    {
      ignerr << "Failed sending mavlink message to SDK: " << strerror(errno) << "\n";
    }
  }
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

  // send only baro data
  if (baro_updated_) {
    sensor_msg.temperature = temperature_;
    sensor_msg.abs_pressure = abs_pressure_;
    sensor_msg.pressure_alt = pressure_alt_;
    // sensor_msg.fields_updated = sensor_msg.fields_updated | SensorSource::BARO;

    baro_updated_ = false;
  }

  // send only diff pressure data
  if (diff_press_updated_) {
    const float temperature_msl = 288.0f; // temperature at MSL (Kelvin)
    float temperature_local = sensor_msg.temperature + 273.0f;
    const float density_ratio = powf((temperature_msl/temperature_local) , 4.256f);
    float rho = 1.225f / density_ratio;

    // Let's use a rough guess of 0.01 hPa as the standard devitiation which roughly yields
    // about +/- 1 m/s noise.
    const float diff_pressure_stddev = 0.01f;
    const float diff_pressure_noise = standard_normal_distribution_(random_generator_) * diff_pressure_stddev;

// #if GAZEBO_MAJOR_VERSION >= 9
    ignition::math::Vector3d vel_b; //TODO: How do you get body velocity?
//     ignition::math::Vector3d vel_b = q_br.RotateVector(model_->RelativeLinearVel());
// #else
//     ignition::math::Vector3d vel_b = q_br.RotateVector(ignitionFromGazeboMath(model_->GetRelativeLinearVel()));
// #endif

    // calculate differential pressure in hPa
    // if vehicle is a tailsitter the airspeed axis is different (z points from nose to tail)
    if (vehicle_is_tailsitter_) {
      sensor_msg.diff_pressure = 0.005f * rho * vel_b.Z() * vel_b.Z() + diff_pressure_noise;
    } else {
      sensor_msg.diff_pressure = 0.005f * rho * vel_b.X() * vel_b.X() + diff_pressure_noise;
    }
    // sensor_msg.fields_updated = sensor_msg.fields_updated | SensorSource::DIFF_PRESS;

    diff_press_updated_ = false;
  }

  if (!hil_mode_ || (hil_mode_ && !hil_state_level_)) {
    mavlink_message_t msg;
    mavlink_msg_hil_sensor_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &sensor_msg);
    send_mavlink_message(&msg);
  }
}

void GazeboMavlinkInterface::send_mavlink_message(const mavlink_message_t *message)
{
  assert(message != nullptr);

  if (gotSigInt_ || close_conn_) {
    return;
  }

  if (serial_enabled_) {

    if (!is_open()) {
      ignerr << "Serial port closed! \n";
      return;
    }

    {
      std::lock_guard<std::recursive_mutex> lock(mutex);

      if (tx_q.size() >= MAX_TXQ_SIZE) {
        ignwarn << "Tx queue overflow\n";
      }
      tx_q.emplace_back(message);
    }
    io_service.post(std::bind(&GazeboMavlinkInterface::do_write, this, true));

  } else {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    int packetlen = mavlink_msg_to_send_buffer(buffer, message);

    if (fds_[CONNECTION_FD].fd > 0) {
      int timeout_ms = (received_first_actuator_ && enable_lockstep_) ? 1000 : 0;
      int ret = ::poll(&fds_[0], N_FDS, timeout_ms);

      if (ret < 0) {
        ignerr << "poll error: " << strerror(errno) << "\n";
        return;
      }

      if (ret == 0 && timeout_ms > 0) {
        ignerr << "poll timeout\n";
        return;
      }

      if (!(fds_[CONNECTION_FD].revents & POLLOUT)) {
        ignerr << "invalid events at fd:" << fds_[CONNECTION_FD].revents << "\n";
        return;
      }

      size_t len;
      if (use_tcp_) {
        len = send(fds_[CONNECTION_FD].fd, buffer, packetlen, 0);
      } else {
        len = sendto(fds_[CONNECTION_FD].fd, buffer, packetlen, 0, (struct sockaddr *)&remote_simulator_addr_, remote_simulator_addr_len_);
      }
      if (len < 0) {
        ignerr << "Failed sending mavlink message: " << strerror(errno) << "\n";
        if (errno == ECONNRESET || errno == EPIPE) {
          if (use_tcp_) { // udp socket remains alive
            ignerr << "Closing connection." << "\n";
            close_conn_ = true;
          }
        }
      }
    }
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
    send_mavlink_message(&msg);
  }
}

void GazeboMavlinkInterface::pollForMAVLinkMessages()
{
  if (gotSigInt_) {
    return;
  }

  bool received_actuator = false;

  do {
    int timeout_ms = (received_first_actuator_ && enable_lockstep_) ? 1000 : 0;
    int ret = ::poll(&fds_[0], N_FDS, timeout_ms);

    if (ret < 0) {
      ignerr << "poll error: " << strerror(errno) << "\n";
      return;
    }

    if (ret == 0 && timeout_ms > 0) {
      ignerr << "poll timeout\n";
      return;
    }

    for (int i = 0; i < N_FDS; i++) {
      if(fds_[i].revents == 0) {
        continue;
      }

      if (!(fds_[i].revents & POLLIN)) {
        continue;
      }

      if (i == LISTEN_FD) { // if event is raised on the listening socket
        acceptConnections();
      } else { // if event is raised on connection socket
        int ret = recvfrom(fds_[i].fd, _buf, sizeof(_buf), 0, (struct sockaddr *)&remote_simulator_addr_, &remote_simulator_addr_len_);
        if (ret < 0) {
          // all data is read if EWOULDBLOCK is raised
          if (errno != EWOULDBLOCK) { // disconnected from client
            ignerr << "recvfrom error: " << strerror(errno) << "\n";
          }
          continue;
        }

        // client closed the connection orderly, only makes sense on tcp
        if (use_tcp_ && ret == 0) {
          ignerr << "Connection closed by client." << "\n";
          close_conn_ = true;
          continue;
        }

        // data received
        int len = ret;
        mavlink_message_t msg;
        mavlink_status_t status;
        for (unsigned i = 0; i < len; ++i) {
          if (mavlink_parse_char(MAVLINK_COMM_0, _buf[i], &msg, &status)) {
            if (hil_mode_) {
              send_mavlink_message(&msg);
            }
            handle_message(&msg, received_actuator);
          }
        }
      }
    }
  } while (!close_conn_ && received_first_actuator_ && !received_actuator && enable_lockstep_ && IsRunning() && !gotSigInt_);
}

void GazeboMavlinkInterface::acceptConnections()
{
  if (fds_[CONNECTION_FD].fd > 0) {
    return;
  }

  // accepting incoming connections on listen fd
  int ret =
    accept(fds_[LISTEN_FD].fd, (struct sockaddr *)&remote_simulator_addr_, &remote_simulator_addr_len_);

  if (ret < 0) {
    if (errno != EWOULDBLOCK) {
      ignerr << "accept error: " << strerror(errno) << "\n";
    }
    return;
  }

  // assign socket to connection descriptor on success
  fds_[CONNECTION_FD].fd = ret; // socket is replaced with latest connection
  fds_[CONNECTION_FD].events = POLLIN | POLLOUT; // read/write
}

void GazeboMavlinkInterface::pollFromQgcAndSdk()
{
  struct pollfd fds[2] = {};
  fds[0].fd = qgc_socket_fd_;
  fds[0].events = POLLIN;
  fds[1].fd = sdk_socket_fd_;
  fds[1].events = POLLIN;

  const int timeout_ms = 0;

  int ret = ::poll(&fds[0], 2, timeout_ms);

  if (ret < 0) {
    ignerr << "poll error: " << strerror(errno) << "\n";
    return;
  }

  if (fds[0].revents & POLLIN) {
    int len = recvfrom(qgc_socket_fd_, _buf, sizeof(_buf), 0, (struct sockaddr *)&remote_qgc_addr_, &remote_qgc_addr_len_);

    if (len > 0) {
      mavlink_message_t msg;
      mavlink_status_t status;
      for (unsigned i = 0; i < len; ++i) {
        if (mavlink_parse_char(MAVLINK_COMM_1, _buf[i], &msg, &status)) {
          // forward message from QGC to serial
          send_mavlink_message(&msg);
        }
      }
    }
  }

  if (fds[1].revents & POLLIN) {
    int len = recvfrom(sdk_socket_fd_, _buf, sizeof(_buf), 0, (struct sockaddr *)&remote_sdk_addr_, &remote_sdk_addr_len_);

    if (len > 0) {
      mavlink_message_t msg;
      mavlink_status_t status;
      for (unsigned i = 0; i < len; ++i) {
        if (mavlink_parse_char(MAVLINK_COMM_2, _buf[i], &msg, &status)) {
          // forward message from SDK to serial
          send_mavlink_message(&msg);
        }
      }
    }
  }
}

void GazeboMavlinkInterface::handle_message(mavlink_message_t *msg, bool &received_actuator)
{
  switch (msg->msgid) {
  case MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS:
    mavlink_hil_actuator_controls_t controls;
    mavlink_msg_hil_actuator_controls_decode(msg, &controls);

    bool armed = (controls.mode & MAV_MODE_FLAG_SAFETY_ARMED);

// #if GAZEBO_MAJOR_VERSION >= 9
//     last_actuator_time_ = world_->SimTime();
// #else
//     last_actuator_time_ = world_->GetSimTime();
// #endif

    for (unsigned i = 0; i < n_out_max; i++) {
      input_index_[i] = i;
    }

    // set rotor speeds, controller targets
    input_reference_.resize(n_out_max);
    for (int i = 0; i < input_reference_.size(); i++) {
      if (armed) {
        input_reference_[i] = (controls.controls[input_index_[i]] + input_offset_[i])
            * input_scaling_[i] + zero_position_armed_[i];
      } else {
        input_reference_[i] = zero_position_disarmed_[i];
      }
    }

    received_actuator = true;
    received_first_actuator_ = true;
    break;
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

void GazeboMavlinkInterface::open() {
  try{
    serial_dev.open(device_);
    serial_dev.set_option(boost::asio::serial_port_base::baud_rate(baudrate_));
    serial_dev.set_option(boost::asio::serial_port_base::character_size(8));
    serial_dev.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_dev.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    serial_dev.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    igndbg << "Opened serial device " << device_ << "\n";
  }
  catch (boost::system::system_error &err) {
    ignerr <<"Error opening serial device: " << err.what() << "\n";
  }
}

void GazeboMavlinkInterface::close()
{
  if(serial_enabled_) {
    ::close(qgc_socket_fd_);
    ::close(sdk_socket_fd_);

    std::lock_guard<std::recursive_mutex> lock(mutex);
    if (!is_open())
      return;

    io_service.stop();
    serial_dev.close();

    if (io_thread.joinable())
      io_thread.join();

  } else {

    ::close(fds_[CONNECTION_FD].fd);
    fds_[CONNECTION_FD] = { 0, 0, 0 };
    fds_[CONNECTION_FD].fd = -1;

    received_first_actuator_ = false;

  }
}

void GazeboMavlinkInterface::do_read(void)
{
  serial_dev.async_read_some(boost::asio::buffer(rx_buf), boost::bind(
      &GazeboMavlinkInterface::parse_buffer, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred
      )
  );
}

// Based on MAVConnInterface::parse_buffer in MAVROS
void GazeboMavlinkInterface::parse_buffer(const boost::system::error_code& err, std::size_t bytes_t){
  mavlink_status_t status;
  mavlink_message_t message;
  uint8_t *buf = this->rx_buf.data();

  assert(rx_buf.size() >= bytes_t);

  for(; bytes_t > 0; bytes_t--)
  {
    auto c = *buf++;

    auto msg_received = static_cast<Framing>(mavlink_frame_char_buffer(&m_buffer, &m_status, c, &message, &status));
    if (msg_received == Framing::bad_crc || msg_received == Framing::bad_signature) {
      _mav_parse_error(&m_status);
      m_status.msg_received = MAVLINK_FRAMING_INCOMPLETE;
      m_status.parse_state = MAVLINK_PARSE_STATE_IDLE;
      if (c == MAVLINK_STX) {
        m_status.parse_state = MAVLINK_PARSE_STATE_GOT_STX;
        m_buffer.len = 0;
        mavlink_start_checksum(&m_buffer);
      }
    }

    if (msg_received != Framing::incomplete) {
      if (hil_mode_) {
        forward_mavlink_message(&message);
      }
      bool not_used;
      handle_message(&message, not_used);
    }
  }
  do_read();
}

void GazeboMavlinkInterface::do_write(bool check_tx_state){
  if (check_tx_state && tx_in_progress)
    return;

  std::lock_guard<std::recursive_mutex> lock(mutex);
  if (tx_q.empty())
    return;

  tx_in_progress = true;
  auto &buf_ref = tx_q.front();

  serial_dev.async_write_some(
    boost::asio::buffer(buf_ref.dpos(), buf_ref.nbytes()), [this, &buf_ref] (boost::system::error_code error,   size_t bytes_transferred)
    {
      assert(bytes_transferred <= buf_ref.len);
      if(error) {
        ignerr << "Serial error: " << error.message() << "\n";
      return;
      }

    std::lock_guard<std::recursive_mutex> lock(mutex);

    if (tx_q.empty()) {
      tx_in_progress = false;
      return;
    }

    buf_ref.pos += bytes_transferred;
    if (buf_ref.nbytes() == 0) {
      tx_q.pop_front();
    }

    if (!tx_q.empty()) {
      do_write(false);
    }
    else {
      tx_in_progress = false;
    }
  });
}