/**
 * This file is part of Resc-Pilot.
 *
 * Copyright 2025 Zhaohong Liu, IRMV Lab, Shanghai Jiao Tong University, <https://www.sjtu.edu.cn/>
 * Developed by Zhaohong Liu <zhliu25 at outlook dot com>, <jaimefriedhelmzhao at gmail dot com>
 * for more information see <https://github.com/JaimeParker/resc-pilot>.
 * If you use this code, please cite the respective publications as
 * listed on the above website.
 *
 * Resc-Pilot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Resc-Pilot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Resc-Pilot. If not, see <http://www.gnu.org/licenses/>.
 * 
 * Created by Zhaohong Liu on 24-9-10.
*/

#ifndef RESCLIB_QUADROTORDYNAMICS_H
#define RESCLIB_QUADROTORDYNAMICS_H

#include <random>

#include "RateControl.h"
#include "ControlAllocator.h"
#include "Rotation.h"
#include "RK4.h"
#include "drone/DroneBase.h"
#include "drone/Iris.h"
#include "drone/IMP250.h"

using State = Eigen::Matrix<double, 12, 1>;
using RP = RescParams;
//using State = Eigen::Matrix<double, 12, 1, Eigen::RowMajor>;

class QuadrotorDynamics {
private:
    /* controller */
    RateControl::Ptr mc_rate_ctrl_ptr_;
    std::unique_ptr<ControlAllocator> control_allocator_;
    bool sys_ctrl_alloc_ = false;

    /* data */
    // we can't get motor thrust dot currently, so we use a function to get thrust based on time
    Eigen::Vector4d motor_thrusts_init_;  // motor thrusts at the beginning of the time step
    Eigen::Vector4d motor_thrusts_;  // motor thrusts at current time and will be used as final motor thrusts
    Eigen::Vector4d motor_thrusts_req_;  // motor thrusts required by the rate controller
    double randomized_mass_;

    /* drone parameters */
    Eigen::Matrix3d inertia_ = Eigen::Matrix3d::Identity();
    Eigen::Matrix4d alloc_mat_;
    std::shared_ptr<DroneBase> drone_;

    /* rl params */
    double domain_randomization_fac_ = 0.0;
public:
    void setDomainRandomizationFac(double fac) { domain_randomization_fac_ = fac; }
    void initDrone(const std::string& drone_name);
    void setSysCtrlAlloc(bool sys_ctrl_alloc) { sys_ctrl_alloc_ = sys_ctrl_alloc; }
    void resetCtrlAllocRange (float & ca_min, float & ca_max) { drone_->resetCARange(ca_min, ca_max); }

    /**
     * @brief Run the control and dynamic kinematic updating using RK4.
     * @param state Eigen::Matrix<double, 12, 1>, including position, velocity, attitude, and body rate
     * @param body_rate_setpoint Required body rate by RL action
     * @param motor_thrusts_cur Current motor thrusts
     * @param thrust_req
     * @return next_state Eigen::Matrix<double, 12, 1>, including position, velocity, attitude, and body rate
     */
    State run(const State &state,
              const Eigen::Vector3d &body_rate_setpoint,
              const Eigen::Vector4d &motor_thrusts_cur,
              const double &thrust_req);

    /**
     * @brief Reset the yaw angle to the range (-PI, PI]
     * @param state
     * @param next_state
     */
    static void resetYaw(const State &state, State &next_state);

    /**
     * @brief Calculate the motor real thrusts after t seconds from the initial state
     * @param motor_thrusts
     * @param motor_thrusts_req
     * @param t
     * @return real motor thrusts using a motor model
     */
    Eigen::Vector4d runMotor(const Eigen::Vector4d &motor_thrusts,
                             Eigen::Vector4d &motor_thrusts_req,
                             double t);

    static Eigen::Vector3d getAttitudeDot(const Eigen::Vector3d & attitude,
                                          const Eigen::Vector3d & body_rate);

    Eigen::Vector3d getBodyRateDot(Eigen::Vector3d & body_rate, Eigen::Vector3d & torque);

    [[nodiscard]] Eigen::Vector3d getTorque(const Eigen::Vector4d & motor_thrusts) const;

    static Eigen::Vector3d getAccel(const Eigen::Vector3d & attitude,
                                    double collective_thrust,
                                    double mass,
                                    const Eigen::Vector3d& external_force);

    State getStateDerivative(double t, const State &state);

    [[nodiscard]] Eigen::Vector4d getFinalMotorThrusts() const;

    void resetDomainRandomization();

    /**
     * @brief Update the drone's state purely by kinematic method,
     * assuming body rate and thrust can change immediately.
     * @param pos Eigen::Vector3d
     * @param vel Eigen::Vector3d
     * @param acc Eigen::Vector3d
     * @param att Eigen::Vector3d
     * @param action Eigen::Vector4d, including body rate and thrust
     * @return Eigen::Matrix<double, 12, 1>, including pos, vel, accel and attitude updated
     */
    static State runKinematicUpdate(const Eigen::Vector3d & pos, const Eigen::Vector3d & vel,
                                    const Eigen::Vector3d & acc, const Eigen::Vector3d & att,
                                    const Eigen::Vector4d & action);

    double rescaledThrust2Thrust(double & rescaled_thrust);
    void resetDronePwmRange(double & pwm_min, double & pwm_max);
};


#endif //RESCLIB_QUADROTORDYNAMICS_H
