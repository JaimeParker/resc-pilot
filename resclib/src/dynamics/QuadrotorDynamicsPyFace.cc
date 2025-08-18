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

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include "QuadrotorDynamics.h"

PYBIND11_MODULE(quadrotor_dynamics_cpp, m) {
    pybind11::class_<QuadrotorDynamics>(m, "QuadrotorDynamics")
        .def(pybind11::init<>())
        .def("initDrone", &QuadrotorDynamics::initDrone,
             pybind11::arg("drone_name"))
        .def("run", &QuadrotorDynamics::run,
             pybind11::arg("state"),
             pybind11::arg("body_rate_setpoint"),
             pybind11::arg("motor_thrusts"),
             pybind11::arg("thrust_req"))
        .def("getFinalMotorThrusts", &QuadrotorDynamics::getFinalMotorThrusts)
        .def("resetDomainRandomization", &QuadrotorDynamics::resetDomainRandomization)
        .def("setDomainRandomizationFac", &QuadrotorDynamics::setDomainRandomizationFac,
             pybind11::arg("fac"))
        .def("setSysCtrlAlloc", &QuadrotorDynamics::setSysCtrlAlloc,
             pybind11::arg("sys_ctrl_alloc"))
        .def("resetCtrlAllocRange", &QuadrotorDynamics::resetCtrlAllocRange,
             pybind11::arg("ca_min"),
             pybind11::arg("ca_max"))
        .def_static("runKinematicUpdate", &QuadrotorDynamics::runKinematicUpdate,
             pybind11::arg("pos"),
             pybind11::arg("vel"),
             pybind11::arg("acc"),
             pybind11::arg("att"),
             pybind11::arg("action"));
}