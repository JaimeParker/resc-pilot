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

#include <random>
#include <iostream>
#include <Eigen/Eigen>

#include "SignedDistanceField.h"

int main() {
//    SignedDistanceField sdf;
//
//    Eigen::Matrix<double, 120, 120, Eigen::RowMajor> sdf_map;
//    std::random_device rd;
//    std::mt19937 gen(rd());
//    std::uniform_real_distribution<> dis(0.0, 2.0);
//
//    for (int i = 0; i < sdf_map.rows(); ++i) {
//        for (int j = 0; j < sdf_map.cols(); ++j) {
//            sdf_map(i, j) = dis(gen);
//        }
//    }
//
    Eigen::Vector3d pos(1.0, 1.0, 1.0);
    Eigen::Vector3d ctrl_p(2.0, 2.0, 1.0);
//
//    auto edge_info = sdf.rayCastingEdge(pos, ctrl_p, sdf_map);
//    std::cout << "Edge info: " << edge_info << std::endl;
//
//    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> row_major_matrix;
//    Eigen::Matrix<double, 3, 3, Eigen::ColMajor> col_major_matrix;
//
//    row_major_matrix << 1, 2, 3,
//                        4, 5, 6,
//                        7, 8, 9;
//
//    col_major_matrix << 1, 4, 7,
//                        2, 5, 8,
//                        3, 6, 9;
//
//    std::cout << "Row-major matrix:\n" << row_major_matrix << std::endl;
//    std::cout << "Column-major matrix:\n" << col_major_matrix << std::endl;
//    sdf.~SignedDistanceField();
//    std::cout << "sdf is destructed." << std::endl;

    SignedDistanceField sdf2;
    Eigen::Matrix<double, 7, 7, Eigen::RowMajor> sdf_map2;
    double map_size = 0.7;
    sdf2.resetMapSize(map_size);
    sdf_map2 << 1, 1, 1, 1, 1, 1, 1,
                1, 1, 1, 1, 1, 1, 1,
                1, 1, 0, 0, 1, 1, 1,
                1, 1, 1, 0, 1, 1, 1,
                1, 1, 1, 1, 0, 1, 1,
                1, 1, 1, 1, 0, 1, 1,
                1, 1, 1, 1, 1, 1, 1;
    pos = Eigen::Vector3d(0.15, 0.15, 1.0);
    ctrl_p = Eigen::Vector3d(0.55, 0.55, 1.0);
    auto edge_info = sdf2.rayCastingEdge(pos, ctrl_p, sdf_map2);
    std::cout << "Edge info: " << edge_info << std::endl;

    return 0;
}