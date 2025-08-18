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
 * Created by Zhaohong Liu on 24-5-9.
*/

#include <frontend_searcher/fake_search.h>

using WptPair = std::pair<Eigen::Vector3d, Eigen::Vector3d>;

int main() {
    FakeSearch fake_search;
//    fake_search.init(nh);

    WptPair wpt_pair0 = fake_search.doCircleLoop(0);
    WptPair wpt_pair1 = fake_search.doCircleLoop(1);
    WptPair wpt_pair2 = fake_search.doCircleLoop(2);
    WptPair wpt_pair3 = fake_search.doCircleLoop(3);

    std::cout << "wpt_pair0: " << wpt_pair0.first << " " << wpt_pair0.second << std::endl;
    std::cout << "wpt_pair1: " << wpt_pair1.first << " " << wpt_pair1.second << std::endl;
    std::cout << "wpt_pair2: " << wpt_pair2.first << " " << wpt_pair2.second << std::endl;
    std::cout << "wpt_pair3: " << wpt_pair3.first << " " << wpt_pair3.second << std::endl;

    fake_search.checkRectLoopWpt();
    Eigen::Vector3d pos(0, 0, 1);
    fake_search.getRectLoopWpt(pos);
    pos[1] = 1;
    fake_search.getRectLoopWpt(pos);

    return 0;
}