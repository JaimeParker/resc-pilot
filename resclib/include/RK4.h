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
 * Created by Zhaohong Liu on 24-10-14.
*/

#ifndef RESCLIB_RK4_H
#define RESCLIB_RK4_H

#include <functional>
#include <vector>
#include <Eigen/Eigen>

using State = Eigen::Matrix<double, 12, 1>;

class RK4 {
public:
    template <typename EigenVecMat>
    static EigenVecMat rk4(
        const std::function<EigenVecMat(double, const EigenVecMat&)>& f,
        const EigenVecMat& y0, double t0, double tf, double h);

    static std::vector<double> rk4(
        const std::function<std::vector<double>(double, const std::vector<double>&)>& f,
        const std::vector<double>& y0, double t0, double tf, double h);

    static State rk4(
        const std::function<State(double, const State&)>& f,
        const State& y0, double t0, double tf, double h);
};

#endif //RESCLIB_RK4_H
