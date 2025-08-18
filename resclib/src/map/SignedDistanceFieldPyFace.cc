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
 * Created by Zhaohong Liu on 24-9-5.
*/

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>  // include this header to use Eigen matrix as argument

#include "SignedDistanceField.h"

PYBIND11_MODULE(sdf_cpp, m) {
    pybind11::class_<SignedDistanceField>(m, "SignedDistanceField")
        .def(pybind11::init<>())
        .def("resetMapSize", &SignedDistanceField::resetMapSize,
             pybind11::arg("map_size"))
        .def("rayCastingEdge", &SignedDistanceField::rayCastingEdge,
             pybind11::arg("pos"),
             pybind11::arg("ctrl_p"),
             pybind11::arg("sdf_map"),
             pybind11::arg("preset_radius") = -1);
}
