/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the PAL Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Author: Luca Marchionni
 * Author: Bence Magyar
 * Author: Enrique Fernández
 * Author: Paul Mathieu
 * Author: Masaru Morita
 */

#pragma once

#include <math.h>
#include <rclcpp/rclcpp.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <boost/function.hpp>

namespace ackermann_steering_controller
{
  namespace bacc = boost::accumulators;

  /**
   * \brief The Odometry class handles odometry readings
   * (2D pose and velocity with related timestamp)
   */
  class Odometry
  {
  public:

    /// Integration function, used to integrate the odometry:
    typedef boost::function<void(double, double)> IntegrationFunction;

    /**
     * \brief Constructor
     * Timestamp will get the current time value
     * Value will be set to zero
     * \param velocity_rolling_window_size Rolling window size used to compute the velocity mean
     */
    /// TODO : const double&
    Odometry(size_t velocity_rolling_window_size = 10);

    /**
     * \brief Initialize the odometry
     * \param time Current time
     */
    void init(const rclcpp::Time &time);

    /**
     * \brief Updates the odometry class with latest wheels position
     * \param rear_wheel_pos  Rear wheel position [rad]
     * \param front_steer_pos Front Steer position [rad]
     * \param time      Current time
     * \return true if the odometry is actually updated
     */
    /// TODO : const double&
    bool update(double rear_wheel_pos, double front_steer_pos, const rclcpp::Time &time);

    // TODO
    bool updateWithHeading(const double& rear_wheel_pos, const double& heading_angle, const rclcpp::Time &time);
    
    bool updateWithHeading(int64_t& encoder_pos, const double& heading_angle, const rclcpp::Time &time);

    /**
     * \brief Updates the odometry class with latest velocity command
     * \param linear  Linear velocity [m/s]
     * \param angular Angular velocity [rad/s]
     * \param time    Current time
     */
    /// TODO : const double& 
    void updateOpenLoop(double linear, double angular, const rclcpp::Time &time);

    /**
     * \brief heading getter
     * \return heading [rad]
     */
    double getHeading() const
    {
      return heading_;
    }

    /**
     * \brief x position getter
     * \return x position [m]
     */
    double getX() const
    {
      return x_;
    }

    /**
     * \brief y position getter
     * \return y position [m]
     */
    double getY() const
    {
      return y_;
    }

    /**
     * \brief linear velocity getter
     * \return linear velocity [m/s]
     */
    double getLinear() const
    {
      return linear_;
    }

    /**
     * \brief angular velocity getter
     * \return angular velocity [rad/s]
     */
    double getAngular() const
    {
      return angular_;
    }

    /**
     * @brief Sets the wheel parameters: radius, separation, encoder resolution
     * 
     * @param wheel_separation_h Seperation between left and right wheels [m]
     * @param wheel_radius       Wheel radius [m]
     * @param encoder_resolution encoder value for one wheel cycle [uint]
     */
    void setWheelParams(const double& wheel_separation_h, const double& wheel_radius, const uint& encoder_resolution);

    /**
     * \brief Velocity rolling window size setter
     * \param velocity_rolling_window_size Velocity rolling window size
     */
    /// TODO: void setVelocityRollingWindowSize(const size_t& velocity_rolling_window_size);
    void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);

  private:

    /// Rolling mean accumulator and window:
    typedef bacc::accumulator_set<double, bacc::stats<bacc::tag::rolling_mean> > RollingMeanAcc;
    typedef bacc::tag::rolling_window RollingWindow;

    /**
     * \brief Integrates the velocities (linear and angular) using 2nd order Runge-Kutta
     * \param linear  Linear  velocity   [m] (linear  displacement, i.e. m/s * dt) computed by encoders
     * \param angular Angular velocity [rad] (angular displacement, i.e. m/s * dt) computed by encoders
     */
    /// TODO : const double& 
    void integrateRungeKutta2(double linear, double angular);

    /**
     * \brief Integrates the velocities (linear and angular) using exact method
     * \param linear  Linear  velocity   [m] (linear  displacement, i.e. m/s * dt) computed by encoders
     * \param angular Angular velocity [rad] (angular displacement, i.e. m/s * dt) computed by encoders
     */
    /// TODO : const double& 
    void integrateExact(double linear, double angular);

    bool integrateExactwithHeading(const double& linear, const double& heading_angle, double& angular, const rclcpp::Time &time);

    /**
     *  \brief Reset linear and angular accumulators
     */
    void resetAccumulators();

    /// Current timestamp:
    rclcpp::Time timestamp_;

    /// Current pose:
    double x_;        //   [m]
    double y_;        //   [m]
    double heading_;  // [rad]

    /// Current velocity:
    double linear_;  //   [m/s]
    double angular_; // [rad/s]

    /// Wheel kinematic parameters [m]:
    double wheel_separation_h_;
    double wheel_radius_;

    /// how many encoders for 1 circle
    uint encoder_resolution_;

    /// Previous wheel position/state [rad]:
    double rear_wheel_old_pos_;
    uint64_t encoder_old_pos_;

    double heading_angle_old_;

    /// Rolling mean accumulators for the linar and angular velocities:
    size_t velocity_rolling_window_size_;
    RollingMeanAcc linear_acc_;
    RollingMeanAcc angular_acc_;

    /// Integration funcion, used to integrate the odometry:
    IntegrationFunction integrate_fun_;
  };
}
