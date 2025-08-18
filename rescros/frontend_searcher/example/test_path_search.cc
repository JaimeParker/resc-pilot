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
 * Created by Zhaohong Liu on 24-10-19.
*/

#include <ros/ros.h>
#include <iostream>

#include "frontend_searcher/PathSearch.h"
#include "map_utils/MapBridge.h"
#include "rviz_utils/PathVisualization.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "path_search_test_node");
    ros::NodeHandle nh("~");
    auto rate = ros::Rate(10);

//    auto map_ptr = std::make_shared<MapBridge>(nh);
//    map_ptr->init();
//    map_ptr->setNumObstaclesManually(100);
//    map_ptr->setRandomObstacles();
//
//    PathVisualization path_vis(nh, "world");
//    path_vis.init();
//
//    PathSearch::Ptr in_kino_searcher;
//    in_kino_searcher = std::make_shared<PathSearch>();
//    in_kino_searcher->setMapBridge(map_ptr);
//    Eigen::Vector3d start_pos(-5, -5, 0);
//    Eigen::Vector3d end_pos(5, 5, 0);
//    if (in_kino_searcher->visibilitySearch(start_pos, end_pos)) {
//        auto path = in_kino_searcher->getDiscretePath();
//        for (const auto& p : path) {
//            std::cout << p.transpose() << std::endl;
//        }
//        path_vis.setPath(path);
//        while (ros::ok()) {
//            map_ptr->publishPCL();
//            path_vis.publish();
//
//            ros::spinOnce();
//            rate.sleep();
//        }
//    }
//    std::cout << "No path found or ros shutdown." << std::endl;
//
//    return 0;

    // for path search debug
    try {
        // Loop for testing visibility search
        for (int i = 0; i < 1000; ++i) {
            try {
                std::cout << "Iteration " << i + 1 << ": Initializing map and testing visibilitySearch..." << std::endl;

                // Initialize map and obstacles
                auto map_ptr = std::make_shared<MapBridge>(nh);
                map_ptr->init();
                map_ptr->setNumObstaclesManually(200);  // Set 100 obstacles
                map_ptr->setRandomObstacles();         // Randomize obstacles

                // Create path searcher
                PathSearch::Ptr in_kino_searcher = std::make_shared<PathSearch>();
                in_kino_searcher->setMapBridge(map_ptr);

                // Test parameters
                Eigen::Vector3d start_pos(-18, -8, 0);
                Eigen::Vector3d end_pos(12, 4, 0);

                // Perform visibility search
                if (in_kino_searcher->visibilitySearch(start_pos, end_pos)) {
                    auto path = in_kino_searcher->getDiscretePath();

                    // Print path if found
                    std::cout << "Path found:" << std::endl;
                    for (const auto &p : path) {
                        std::cout << p.transpose() << std::endl;
                    }
                } else {
                    std::cout << "No path found." << std::endl;
                }

            } catch (const std::exception &e) {
                // Catch and report exception for each iteration
                std::cerr << "Exception occurred in iteration " << i + 1 << ": " << e.what() << std::endl;
                return -1;
            } catch (...) {
                // Catch any unknown errors
                std::cerr << "Unknown error occurred in iteration " << i + 1 << "." << std::endl;
                return -1;
            }
        }

        std::cout << "Test completed successfully for 1000 iterations." << std::endl;
    } catch (const std::exception &e) {
        // Catch initialization-related exceptions
        std::cerr << "Initialization error: " << e.what() << std::endl;
        return -1;
    } catch (...) {
        // Catch unknown errors during initialization
        std::cerr << "Unknown error during initialization." << std::endl;
        return -1;
    }

    return 0;
}