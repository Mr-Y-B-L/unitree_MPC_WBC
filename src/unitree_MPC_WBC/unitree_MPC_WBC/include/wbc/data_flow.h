// The MIT License

// Copyright (c) 2022
// Robot Motion and Vision Laboratory at East China Normal University
// Contact: tophill.robotics@gmail.com

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#ifndef DATAFLOW_H
#define DATAFLOW_H

#include"utils/qr_cpptypes.h"
class qrWbcCtrlData {

public:

    /**
     * @brief Desired body position in world frame.
     */
    wbc::Vec3<float> pBody_des;

    /**
     * @brief Desired body velocity in world frame.
     */
    wbc::Vec3<float> vBody_des;

    /**
     * @brief Desired body acceleration in world frame.
     */
    wbc::Vec3<float> aBody_des;

    /**
     * @brief Desired body roll pitch yaw.
     */
    wbc::Vec3<float> pBody_RPY_des;

    /**
     * @brief Desired body angular velocity in world frame.
     */
    wbc::Vec3<float> vBody_Ori_des;

    /**
     * @brief Desired foothold position in world frame.
     */
    wbc::Vec3<float> pFoot_des[4];

    /**
     * @brief Desired foothold velocity in world frame.
     */
    wbc::Vec3<float> vFoot_des[4];

    /**
     * @brief Desired foothold acceleration in world frame.
     */
    wbc::Vec3<float> aFoot_des[4];

    /**
     * @brief Desired foothold force in world frame.
     */
    wbc::Vec3<float> Fr_des[4];

    /**
     * @brief Current contact state of 4 foothold.
     */
    wbc::Vec4<int> contact_state;

    /**
     * @brief Whether to conduct WBC.
     * If MPC and WBC are conducted in one iteration, this iteration will consume so much time,
     * so if MPC is conducted in this iteration, WBC will be conducted and vice versa.
     */
    bool allowAfterMPC = true;

};
#endif