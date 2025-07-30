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
 * Created by Zhaohong Liu on 24-9-3.
*/

#include <iostream>
#include <vector>
#include <RK4.h>

std::vector<double> example_f([[maybe_unused]] double t, const std::vector<double>& y) {
    std::vector<double> dydt(y.size());
    // Example differential equation: dy/dt = -y (exponential decay)
    for (size_t i = 0; i < y.size(); ++i) {
        dydt[i] = -y[i];
    }
    return dydt;
}

int main() {
    std::vector<double> y0 = {1.0};  // Initial condition y(t0) = 1
    double t0 = 0.0;
    double tf = 2.0;
    double h = 0.1;

    std::vector<double> result = RK4::rk4(example_f, y0, t0, tf, h);

    std::cout << "Result: ";
    for (const double& value : result) {
        std::cout << value << " ";
    }
    std::cout << std::endl;

    return 0;
}