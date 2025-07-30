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
 * Created by Zhaohong Liu on 24-12-6.
*/

#include <iostream>
#include "K_GPEP.h"

int main() {
    K_GPEP k_gpep;
    Matrix sdf_map(7, 7);
    double map_size = 0.7;

    sdf_map <<
        1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1,
        1, 1, 0, 1, 1, 1, 1,
        1, 1, 1, 0, 1, 1, 1,
        1, 1, 1, 1, 0, 1, 1,
        1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1;

    k_gpep.set2DESDFMap(sdf_map, map_size, map_size);

    auto test_pos = k_gpep.voxel2Pos(0, 0);
    std::cout << "Test pos: " << test_pos.transpose() << std::endl;

    auto pos = Eigen::Vector3d(0.15, 0.15, 1.0);
    auto ctrl_p = Eigen::Vector3d(0.55, 0.55, 1.0);

    auto edge_info = k_gpep.guidedPseudoRaycast(pos, ctrl_p);
    std::cout << "Edge info: " << edge_info.transpose() << std::endl;

    for (float y = 0.65; y > 0;) {
        for (float x = 0.05; x < 0.7;) {
            auto pos_f = Eigen::Vector2f(x, y);
            auto sdf = k_gpep.getDistanceSDF(pos_f);
            std::cout << sdf << ", ";
            x += 0.1;
        }
        y -= 0.1;
        std::cout << std::endl;
    }

    return 0;
}
