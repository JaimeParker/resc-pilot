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
 * Created by Zhaohong Liu on 24-9-18.
*/

#ifndef RVIZ_UTILS_VISUALIZATIONCOLOR_H
#define RVIZ_UTILS_VISUALIZATIONCOLOR_H

class VisualizationColor {
public:
    // RGB values
    float r, g, b;

    VisualizationColor(float red, float green, float blue) : r(red), g(green), b(blue) {}

    // predefined colors
    static const VisualizationColor RED;
    static const VisualizationColor GREEN;
    static const VisualizationColor BLUE;
    static const VisualizationColor YELLOW;
    static const VisualizationColor CYAN;
    static const VisualizationColor MAGENTA;
    static const VisualizationColor WHITE;
    static const VisualizationColor BLACK;
    static const VisualizationColor PURPLE;
    static const VisualizationColor DARKTEAL;
    static const VisualizationColor ORANGE;
};

#endif //RVIZ_UTILS_VISUALIZATIONCOLOR_H
