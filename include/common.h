#ifndef SITL_GAZEBO_COMMON_H_
#define SITL_GAZEBO_COMMON_H_
/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
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


#include <tinyxml.h>
#include <typeinfo>
#include <Eigen/Dense>
#include <ignition/math.hh>
#include <sdf/sdf.hh>
#include <ignition/common4/ignition/common.hh>

namespace gazebo {

/// Returns scalar value constrained by (min_val, max_val)
template<typename Scalar>
static inline constexpr const Scalar &constrain(const Scalar &val, const Scalar &min_val, const Scalar &max_val) {
    return (val < min_val) ? min_val : ((val > max_val) ? max_val : val);
}

/**
 * \brief Obtains a parameter from sdf.
 * \param[in] sdf Pointer to the sdf object.
 * \param[in] name Name of the parameter.
 * \param[out] param Param Variable to write the parameter to.
 * \param[in] default_value Default value, if the parameter not available.
 * \param[in] verbose If true, ignerror if the parameter is not available.
 */
template<class T>
bool getSdfParam(const std::shared_ptr<const sdf::Element> sdf, const std::string& name, T& param, const T& default_value, const bool& verbose =
                     false) {
  if (sdf->HasElement(name)) {
    param = sdf->Get<T>(name);
    return true;
  }
  else {
    param = default_value;
    if (verbose)
      ignerr << "[rotors_gazebo_plugins] Please specify a value for parameter \"" << name << "\".\n";
  }
  return false;
}

/**
 * \brief Get a math::Angle as an angle from [0, 360)
 */
inline double GetDegrees360(const ignition::math::Angle& angle) {
  double degrees = angle.Degree();
  while (degrees < 0.) degrees += 360.0;
  while (degrees >= 360.0) degrees -= 360.0;
  return degrees;
}


}  // namespace gazebo

template <typename T>
class FirstOrderFilter {
/*
This class can be used to apply a first order filter on a signal.
It allows different acceleration and deceleration time constants.

Short reveiw of discrete time implementation of firest order system:
Laplace:
    X(s)/U(s) = 1/(tau*s + 1)
continous time system:
    dx(t) = (-1/tau)*x(t) + (1/tau)*u(t)
discretized system (ZoH):
    x(k+1) = exp(samplingTime*(-1/tau))*x(k) + (1 - exp(samplingTime*(-1/tau))) * u(k)
*/

  public:
    FirstOrderFilter(double timeConstantUp, double timeConstantDown, T initialState):
      timeConstantUp_(timeConstantUp),
      timeConstantDown_(timeConstantDown),
      previousState_(initialState) {}

    T updateFilter(T inputState, double samplingTime) {
      /*
      This method will apply a first order filter on the inputState.
      */
      T outputState;
      if(inputState > previousState_){
        // Calcuate the outputState if accelerating.
        double alphaUp = exp(- samplingTime / timeConstantUp_);
        // x(k+1) = Ad*x(k) + Bd*u(k)
        outputState = alphaUp * previousState_ + (1 - alphaUp) * inputState;

      }else{
        // Calculate the outputState if decelerating.
        double alphaDown = exp(- samplingTime / timeConstantDown_);
        outputState = alphaDown * previousState_ + (1 - alphaDown) * inputState;
      }
      previousState_ = outputState;
      return outputState;

    }
    ~FirstOrderFilter() {}

  protected:
    double timeConstantUp_;
    double timeConstantDown_;
    T previousState_;
};

/// Computes a quaternion from the 3-element small angle approximation theta.
template<class Derived>
Eigen::Quaternion<typename Derived::Scalar> QuaternionFromSmallAngle(const Eigen::MatrixBase<Derived> & theta) {
  typedef typename Derived::Scalar Scalar;
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
  const Scalar q_squared = theta.squaredNorm() / 4.0;

  if (q_squared < 1) {
    return Eigen::Quaternion<Scalar>(sqrt(1 - q_squared), theta[0] * 0.5, theta[1] * 0.5, theta[2] * 0.5);
  }
  else {
    const Scalar w = 1.0 / sqrt(1 + q_squared);
    const Scalar f = w * 0.5;
    return Eigen::Quaternion<Scalar>(w, theta[0] * f, theta[1] * f, theta[2] * f);
  }
}

template<class In, class Out>
void copyPosition(const In& in, Out* out) {
  out->x = in.x;
  out->y = in.y;
  out->z = in.z;
}

/**
 * @note Frames of reference:
 * g - gazebo (ENU), east, north, up
 * r - rotors imu frame (FLU), forward, left, up
 * b - px4 (FRD) forward, right down
 * n - px4 (NED) north, east, down
 */

/**
 * @brief Quaternion for rotation between ENU and NED frames
 *
 * NED to ENU: +PI/2 rotation about Z (Down) followed by a +PI rotation around X (old North/new East)
 * ENU to NED: +PI/2 rotation about Z (Up) followed by a +PI rotation about X (old East/new North)
 * This rotation is symmetric, so q_ENU_to_NED == q_NED_to_ENU.
 */
static const auto q_ENU_to_NED = ignition::math::Quaterniond(0, 0.70711, 0.70711, 0);

/**
 * @brief Quaternion for rotation between body FLU and body FRD frames
 *
 * +PI rotation around X (Forward) axis rotates from Forward, Right, Down (aircraft)
 * to Forward, Left, Up (base_link) frames and vice-versa.
 * This rotation is symmetric, so q_FLU_to_FRD == q_FRD_to_FLU.
 */
static const auto q_FLU_to_FRD = ignition::math::Quaterniond(0, 1, 0, 0);

// sensor X-axis unit vector in `base_link` frame
static const ignition::math::Vector3d kDownwardRotation = ignition::math::Vector3d(0, 0, -1);
static const ignition::math::Vector3d kUpwardRotation = ignition::math::Vector3d(0, 0, 1);
static const ignition::math::Vector3d kBackwardRotation = ignition::math::Vector3d(-1, 0, 0);
static const ignition::math::Vector3d kForwardRotation = ignition::math::Vector3d(1, 0, 0);
static const ignition::math::Vector3d kLeftRotation = ignition::math::Vector3d(0, 1, 0);
static const ignition::math::Vector3d kRightRotation = ignition::math::Vector3d(0, -1, 0);

// Zurich Irchel Park
static constexpr const double kDefaultHomeLatitude = 47.397742 * M_PI / 180.0;   // rad
static constexpr const double kDefaultHomeLongitude = 8.545594 * M_PI / 180.0;   // rad
static constexpr const double kDefaultHomeAltitude = 488.0;                      // meters

// Earth radius
static constexpr const double earth_radius = 6353000.0;      // meters

/**
 * @brief Get latitude and longitude coordinates from local position
 * @param[in] pos position in the local frame
 * @return std::pair of Latitude and Longitude
 */
inline std::pair<double, double> reproject(ignition::math::Vector3d& pos,
                                    double& lat_home,
                                    double& lon_home,
                                    double& alt_home)
{
  // reproject local position to gps coordinates
  const double x_rad = pos.Y() / earth_radius;    // north
  const double y_rad = pos.X() / earth_radius;    // east
  const double c = sqrt(x_rad * x_rad + y_rad * y_rad);
  const double sin_c = sin(c);
  const double cos_c = cos(c);

  double lat_rad, lon_rad;

  if (c != 0.0) {
    lat_rad = asin(cos_c * sin(lat_home) + (x_rad * sin_c * cos(lat_home)) / c);
    lon_rad = (lon_home + atan2(y_rad * sin_c, c * cos(lat_home) * cos_c - x_rad * sin(lat_home) * sin_c));
  } else {
    lat_rad = lat_home;
    lon_rad = lon_home;
  }

  return std::make_pair (lat_rad, lon_rad);
}

#endif  // SITL_GAZEBO_COMMON_H_
