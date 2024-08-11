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

#include "wbc/task_set/qr_task_body_orientation.hpp"


template<typename T>
qrTaskBodyOrientation<T>::qrTaskBodyOrientation(const FloatingBaseModel<T> *fb_model):
    qrTask<T>(3), fbModel(fb_model)
{
    TK::Jt = wbc::DMat<T>::Zero(TK::dimTask, this->dimConfig);
    TK::Jt.block(0, 0, 3, 3).setIdentity();
    TK::JtDotQdot = wbc::DVec<T>::Zero(TK::dimTask);

    errScale = wbc::DVec<T>::Constant(TK::dimTask, 1.);
    Kp = wbc::DVec<T>::Constant(TK::dimTask, 50.);
    Kd = wbc::DVec<T>::Constant(TK::dimTask, 1.0);
}


template<typename T>
bool qrTaskBodyOrientation<T>::UpdateCommand(const void *des_pos, const wbc::DVec<T> &des_vel, const wbc::DVec<T> &des_acc)
{
    wbc::Quat<T> *ori_cmd = (wbc::Quat<T> *)des_pos;
    wbc::Quat<T> link_ori = (fbModel->_state.bodyOrientation);

    /* Get the inverse of the quaternion. */
    wbc::Quat<T> link_ori_inv;
    link_ori_inv[0] = link_ori[0];
    link_ori_inv[1] = -link_ori[1];
    link_ori_inv[2] = -link_ori[2];
    link_ori_inv[3] = -link_ori[3];

    /* Compute the error of quaternion. */
    wbc::Quat<T> ori_err = robotics::math::quatProduct(*ori_cmd, link_ori_inv);
    if (ori_err[0] < 0.) {
        ori_err *= (-1.);
    }

    /* Convert the error from quaternion to SO3. */
    wbc::Vec3<T> ori_err_so3;
    robotics::math::quaternionToso3(ori_err, ori_err_so3);

    wbc::SVec<T> curr_vel = fbModel->_state.bodyVelocity;/* Angular and linear velocity in base frame. */

    wbc::Mat3<T> Rot = robotics::math::quaternionToRotationMatrix(link_ori); /* Rot transform world fram to base frame. */
    wbc::Vec3<T> vel_err = Rot.transpose() * (TK::desiredVel - curr_vel.head(3));

    /* PD control to get acceleration command. */
    for (int i = 0; i < 3; ++i) {
        TK::posErr[i] = errScale[i] * ori_err_so3[i];
        TK::desiredVel[i] = des_vel[i];
        TK::desiredAcc[i] = des_acc[i];

        TK::xddotCmd[i] = Kp[i] * ori_err_so3[i] + Kd[i] * vel_err[i] + TK::desiredAcc[i];
        TK::xddotCmd[i] = std::min(std::max(TK::xddotCmd[i], (T)-10), (T)10);
    }
    return true;
}


template<typename T>
bool qrTaskBodyOrientation<T>::UpdateTaskJacobian()
{
    wbc::Quat<T> quat = fbModel->_state.bodyOrientation;
    wbc::Mat3<T> Rot = robotics::math::quaternionToRotationMatrix(quat);
    TK::Jt.block(0, 0, 3, 3) = Rot.transpose();
    return true;
}


template<typename T>
bool qrTaskBodyOrientation<T>::UpdateTaskJDotQdot()
{
    return true;
}


template class qrTaskBodyOrientation<float>;
