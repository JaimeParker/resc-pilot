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
 * Created by Zhaohong Liu on 24-9-17.
*/

#include <pybind11/pybind11.h>

#include "Params.h"

PYBIND11_MODULE(params_cpp, m) {
    pybind11::class_<RescParams>(m, "RescParams")
        .def_readonly_static("G", &RescParams::G)
        .def_readonly_static("RL_DT", &RescParams::RL_DT)
        .def_readonly_static("SIM_DT", &RescParams::SIM_DT)
        .def_readonly_static("CT_G_MIN", &RescParams::CT_G_MIN)
        .def_readonly_static("CT_G_MAX", &RescParams::CT_G_MAX)
        .def_readonly_static("ROTOR_INPUT_SCALING", &RescParams::ROTOR_INPUT_SCALING)
        .def_readonly_static("ROTOR_POSITION_ARMED", &RescParams::ROTOR_POSITION_ARMED)
        .def_readonly_static("ROTOR_VEL_SLOWDOWN_SIM", &RescParams::ROTOR_VEL_SLOWDOWN_SIM)
        .def_readonly_static("ROTOR_RISE_TIME", &RescParams::ROTOR_RISE_TIME);
}
