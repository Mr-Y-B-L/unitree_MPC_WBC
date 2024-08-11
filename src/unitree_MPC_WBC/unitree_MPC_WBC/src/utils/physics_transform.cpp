#include "utils/physics_transform.h"


namespace Quadruped {

wbc::Mat3<float> transformInertia(wbc::Mat3<float> inertia, float mass, wbc::Vec3<float> p, wbc::Mat3<float> R)
{
    wbc::Mat3<float> newInertia = wbc::Mat3<float>::Identity();
    wbc::Mat3<float> pxp = p*p.transpose();
    newInertia = inertia + mass*(p.dot(p)*wbc::Mat3<float>::Identity()-pxp);
    return newInertia;
}

} // namespace Quadruped
