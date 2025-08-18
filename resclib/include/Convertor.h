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
 * Created by Zhaohong Liu on 24-10-15.
*/

#ifndef RESCLIB_CONVERTOR_H
#define RESCLIB_CONVERTOR_H

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <iomanip>

class Convertor {
public:
    static void q2EulerAngle(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw);

    static void q2EulerAngle(const Eigen::Quaterniond& q, Eigen::Vector3d& euler);

    static void euler2Quaternion(Eigen::Quaterniond& q, const Eigen::Vector3d& euler);

    static Eigen::Quaterniond euler2Quaternion(const Eigen::Vector3d& euler);

    /**
     * Fast pseudoinverse based on full rank cholesky factorisation
     * Courrieu, P. (2008). Fast Computation of Moore-Penrose Inverse Matrices, 8(2), 25–29.
     * http://arxiv.org/abs/0804.4809
     */
    template <typename T, int Rows, int Cols>
    static bool getInv(const Eigen::Matrix<T, Rows, Cols> &G, Eigen::Matrix<T, Cols, Rows> &res) {
        // ref PX4-Autopilot src/lib/matrix, gen by chatgpt-4o
        if constexpr (Rows >= Cols) {
            Eigen::Matrix<T, Cols, Cols> A = G.transpose() * G;  // Square matrix
            Eigen::LDLT<Eigen::Matrix<T, Cols, Cols>> ldlt(A);   // LDLT decomposition for stability
            if (ldlt.info() != Eigen::Success) {
                return false;  // Matrix is singular
            }
            res = ldlt.solve(G.transpose());
        } else {
            Eigen::Matrix<T, Rows, Rows> A = G * G.transpose();  // Square matrix
            Eigen::LDLT<Eigen::Matrix<T, Rows, Rows>> ldlt(A);   // LDLT decomposition for stability
            if (ldlt.info() != Eigen::Success) {
                return false;  // Matrix is singular
            }
            res = G.transpose() * ldlt.solve(Eigen::Matrix<T, Rows, Rows>::Identity(A.rows(), A.cols()));
        }
        return true;
    }

    template <typename T, int Rows, int Cols>
    static void printMatrix(const Eigen::Matrix<T, Rows, Cols> &G) {
        std::cout << std::fixed << std::setprecision(4);

        for (int row = 0; row < G.rows(); ++row) {
            std::cout << "| ";
            for (int col = 0; col < G.cols(); ++col) {
                std::cout << std::setw(8) << G(row, col) << " ";
            }
            std::cout << "|\n";
        }

        std::cout << std::endl;
    }
};


#endif //RESCLIB_CONVERTOR_H
