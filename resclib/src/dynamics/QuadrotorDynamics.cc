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

#include "QuadrotorDynamics.h"

void QuadrotorDynamics::initDrone(const std::string& drone_name) {
    if (drone_name == "iris") {
        drone_ = std::make_shared<Iris>();
    } else if (drone_name == "imp250") {
        drone_ = std::make_shared<IMP250>();
    } else {
        throw std::runtime_error("Unknown drone name: " + drone_name);
    }

    mc_rate_ctrl_ptr_ = std::make_unique<RateControl>(drone_);
    randomized_mass_ = drone_->mass_;
    inertia_(0, 0) = drone_->ixx_;
    inertia_(1, 1) = drone_->iyy_;
    inertia_(2, 2) = drone_->izz_;
    alloc_mat_ = mc_rate_ctrl_ptr_->getAllocMat();

    control_allocator_ = std::make_unique<ControlAllocator>(drone_);
}

State QuadrotorDynamics::run(const State &state,
                             const Eigen::Vector3d &body_rate_setpoint,
                             const Eigen::Vector4d &motor_thrusts_cur,
                             const double &thrust_req) {
    const auto body_rate = state.segment(9, 3);
    motor_thrusts_init_ = motor_thrusts_cur;

    auto torque_sp = mc_rate_ctrl_ptr_->update(body_rate, body_rate_setpoint, thrust_req);

    if (!sys_ctrl_alloc_) {
        // using mixer
        motor_thrusts_req_ = mc_rate_ctrl_ptr_->mixer(torque_sp, thrust_req);
    } else {
        // using control allocator
        control_allocator_->setControlSetpoint(torque_sp, thrust_req);
        control_allocator_->pseudoInverseAllocate();
        motor_thrusts_req_ = control_allocator_->getMotorThrusts();
    }

    State next_state = RK4::rk4(
        [this](const double t, const State &state_c) { return this->getStateDerivative(t, state_c); },
        state, 0.0, RP::RL_DT, RP::SIM_DT);

    resetYaw(state, next_state);

    return next_state;
}

void QuadrotorDynamics::resetYaw(const State &state, State &next_state) {
    // Assume the yaw is in range (-PI, PI]
    const double yaw = state[8];
    const double yaw_next = next_state[8];
    constexpr double TWO_PI = 2 * M_PI;

    if (yaw <= M_PI && M_PI < yaw_next) {
        next_state[8] -= TWO_PI;
    } else if (yaw_next < -M_PI && -M_PI <= yaw) {
        next_state[8] += TWO_PI;
    }
}

Eigen::Vector4d QuadrotorDynamics::runMotor(const Eigen::Vector4d &motor_thrusts,
                                            Eigen::Vector4d &motor_thrusts_req,
                                            const double t) {
    // We simply model the motor as a first-order system
    // 当sim dt足够小或电机时间常数比较大时可以仔细设计该函数，现暂时直接认为阶跃
    if (t >= drone_->rotor_rise_time_ || RP::SIM_DT >= drone_->rotor_rise_time_) {
        // if the simulation dt is larger than the rise time, we directly return the required thrusts
        return motor_thrusts_req;
    }

    return motor_thrusts + (motor_thrusts_req - motor_thrusts) * RP::RL_DT / RP::ROTOR_RISE_TIME;
}

Eigen::Vector3d QuadrotorDynamics::getAttitudeDot(const Eigen::Vector3d &attitude,
                                                  const Eigen::Vector3d &body_rate) {
    return Rotation::rotB2A(attitude) * body_rate;
}

Eigen::Vector3d QuadrotorDynamics::getBodyRateDot(Eigen::Vector3d &body_rate, Eigen::Vector3d &torque) {
    Eigen::Vector3d body_rate_dot = Eigen::Vector3d::Zero();

    Eigen::Vector3d inertia_diag = inertia_.diagonal();
    const double ixx = inertia_diag[0];
    const double iyy = inertia_diag[1];
    const double izz = inertia_diag[2];

    body_rate_dot[0] = (torque[0] + (iyy - izz) * body_rate[1] * body_rate[2]) / ixx;
    body_rate_dot[1] = (torque[1] + (izz - ixx) * body_rate[0] * body_rate[2]) / iyy;
    body_rate_dot[2] = (torque[2] + (ixx - iyy) * body_rate[0] * body_rate[1]) / izz;

    return body_rate_dot;
}

Eigen::Vector3d QuadrotorDynamics::getTorque(const Eigen::Vector4d &motor_thrusts) const {
    const auto ctrl_unit = alloc_mat_ * motor_thrusts;
    Eigen::Vector3d torque_body(ctrl_unit[1], ctrl_unit[2], ctrl_unit[3]);

    return torque_body;
}

Eigen::Vector3d QuadrotorDynamics::getAccel(const Eigen::Vector3d &attitude,
                                            const double collective_thrust, const double mass,
                                            const Eigen::Vector3d& external_force = Eigen::Vector3d::Zero()) {
    const Eigen::Vector3d gravity(0, 0, -RP::G);
    const Eigen::Vector3d thrust_w = Rotation::rotB2ody2World(attitude) * Eigen::Vector3d(0, 0, collective_thrust);
    const auto joint_external_force = thrust_w + external_force + gravity * mass;

    return joint_external_force / mass;
}

State QuadrotorDynamics::getStateDerivative(double t, const State &state) {
    const auto vel = state.segment(3, 3);
    const auto att = state.segment(6, 3);
    Eigen::Vector3d body_rate = state.segment(9, 3);

    motor_thrusts_ = runMotor(motor_thrusts_init_, motor_thrusts_req_, t);
    const double collective_thrust = motor_thrusts_.sum();

    const auto accel = getAccel(att, collective_thrust, randomized_mass_);

    auto torque = getTorque(motor_thrusts_);
    const auto body_rate_dot = getBodyRateDot(body_rate, torque);

    const auto attitude_dot = getAttitudeDot(att, body_rate);

    State state_dot;
    state_dot.segment<3>(0) = vel;
    state_dot.segment<3>(3) = accel;
    state_dot.segment<3>(6) = attitude_dot;
    state_dot.segment<3>(9) = body_rate_dot;

    return state_dot;
}

Eigen::Vector4d QuadrotorDynamics::getFinalMotorThrusts() const {
    return motor_thrusts_;
}

void QuadrotorDynamics::resetDomainRandomization() {
    if (domain_randomization_fac_ < 1e-3) {
        return;
    }
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(-domain_randomization_fac_, domain_randomization_fac_);

    std::array<double, 4> randomization{};
    for (auto &value : randomization) {
        value = dis(gen);
    }

    randomized_mass_= drone_->mass_ * (1.0 + randomization[0]);
    inertia_(0, 0) = drone_->ixx_ * (1.0 + randomization[1]);
    inertia_(1, 1) = drone_->iyy_ * (1.0 + randomization[2]);
    inertia_(2, 2) = drone_->izz_ * (1.0 + randomization[3]);
}

State QuadrotorDynamics::runKinematicUpdate(const Eigen::Vector3d &pos, const Eigen::Vector3d &vel,
                                            const Eigen::Vector3d &acc, const Eigen::Vector3d &att,
                                            const Eigen::Vector4d &action) {
    // action to body rate and thrust_accel
    double acc_thrust = (action[3] + 1) * 2 * (RP::CT_G_MAX - RP::CT_G_MIN) + RP::CT_G_MIN;
    Eigen::Vector3d body_rate = action.head<3>().
            cwiseProduct(Eigen::Vector3d(RP::MAX_ROLL_RATE, RP::MAX_PITCH_RATE, RP::MAX_YAW_RATE));

    State next_state = State();

    double delta_t = RP::RL_DT;

    Eigen::Matrix3d rot_mat_b2a = Rotation::rotB2A(att);
    Eigen::Vector3d attitude_euler_dot = rot_mat_b2a * body_rate;

    Eigen::Vector3d g(0, 0, -RP::G);
    Eigen::Vector3d a_thrust_b(0, 0, acc_thrust);

    Eigen::Vector3d attitude_euler = att;

    Eigen::Matrix<double, 5, 3> trans_accel;
    trans_accel.row(0) = acc;
    double dt_div_4 = delta_t / 4.0;
    for (int i = 0; i < 4; ++i) {
        attitude_euler += attitude_euler_dot * dt_div_4;
        Eigen::Matrix3d rot_mat_b2w = Rotation::rotB2ody2World(attitude_euler);
        trans_accel.row(i + 1) = g + rot_mat_b2w * a_thrust_b;
    }

    // Simpson's rule, get vel
    double h = delta_t / 2.0;
    Eigen::Vector3d d_vel_mid = h / 6.0 * (trans_accel.row(0) + 4.0 * trans_accel.row(1) + trans_accel.row(2));
    Eigen::Vector3d trans_vel_mid = vel + d_vel_mid;
    Eigen::Vector3d d_vel_final = h / 6.0 * (trans_accel.row(2) + 4.0 * trans_accel.row(3) + trans_accel.row(4));
    Eigen::Vector3d trans_vel_final = trans_vel_mid + d_vel_final;

    Eigen::Vector3d d_pos = delta_t / 6.0 * (vel + 4.0 * trans_vel_mid + trans_vel_final);
    Eigen::Vector3d pos_next = d_pos + pos;

    // Revise yaw
    double yaw0 = att[2];
    double yaw1 = attitude_euler[2];
    if (yaw0 <= M_PI && M_PI < yaw1) {
        attitude_euler[2] -= 2 * M_PI;
    }
    if (yaw1 < -M_PI && -M_PI <= yaw0) {
        attitude_euler[2] += 2 * M_PI;
    }

    next_state.segment(0, 3) = pos_next;
    next_state.segment(3, 3) = trans_vel_final;
    next_state.segment(6, 3) = trans_accel.row(4);
    next_state.segment(9, 3) = attitude_euler;

    return next_state;
}

double QuadrotorDynamics::rescaledThrust2Thrust(double &rescaled_thrust) {
    double pwm = drone_->mavrosAttTarThrust2Pwm(rescaled_thrust);
    return drone_->pwm2Thrust(pwm);
}

void QuadrotorDynamics::resetDronePwmRange(double &pwm_min, double &pwm_max) {
    drone_->resetPwmRange(pwm_min, pwm_max);
}
