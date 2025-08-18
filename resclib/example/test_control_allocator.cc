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
 * Created by Zhaohong Liu on 24-11-19.
*/

#include "drone/DroneBase.h"
#include "Convertor.h"
#include "ControlAllocator.h"
#include "drone/Iris.h"

int main() {
    std::shared_ptr<DroneBase> drone = std::make_shared<Iris>();
    ControlAllocator control_allocator(drone);

    const auto& effectiveness = control_allocator.getEffectiveness();
    std::cout << "effectiveness: " << std::endl;
    Convertor::printMatrix(effectiveness);

    const auto& mix = control_allocator.getMix();
    std::cout << "mix: " << std::endl;
    Convertor::printMatrix(mix);

    // example of how to use control allocator
    auto torque_flu_sp = Eigen::Vector3d(0.2, 0.0, 0.0);
    double thrust_flu_sp = 0.5 * 6.925;

    control_allocator.setControlSetpoint(torque_flu_sp, thrust_flu_sp);
    control_allocator.pseudoInverseAllocate();
    auto motor_thrusts = control_allocator.getMotorThrusts();

    std::cout << "Motor thrusts: " << motor_thrusts.transpose() << std::endl;

    return 0;
}
