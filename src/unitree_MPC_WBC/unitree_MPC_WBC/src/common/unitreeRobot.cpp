/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "common/unitreeRobot.h"
#include <iostream>

Vec3 QuadrupedRobot::getX(LowlevelState &state){
    return getFootPosition(state, 0, FrameType::BODY);
}

Vec34 QuadrupedRobot::getVecXP(LowlevelState &state){
    Vec3 x = getX(state);
    Vec34 vecXP, qLegs;
    qLegs = state.getQ();

    for(int i(0); i < 4; ++i){
        vecXP.col(i) = _Legs[i]->calcPEe2B(qLegs.col(i)) - x;
    }
    return vecXP;
}
// Inverse Kinematics
Vec12 QuadrupedRobot::getQ(const Vec34 &vecP, FrameType frame){
    Vec12 q;
    for(int i(0); i < 4; ++i){
        q.segment(3*i, 3) = _Legs[i]->calcQ(vecP.col(i), frame);
    }
    return q;
}

Vec12 QuadrupedRobot::getQd(const Vec34 &pos, const Vec34 &vel, FrameType frame){
    Vec12 qd;
    for(int i(0); i < 4; ++i){
        qd.segment(3*i, 3) = _Legs[i]->calcQd(pos.col(i), vel.col(i), frame);
    }
    return qd;
}

Vec12 QuadrupedRobot::getTau(const Vec12 &q, const Vec34 feetForce){
    Vec12 tau;
    for(int i(0); i < 4; ++i){
        tau.segment(3*i, 3) = _Legs[i]->calcTau(q.segment(3*i, 3), feetForce.col(i));
    }
    return tau;
}

// Forward Kinematics
Vec3 QuadrupedRobot::getFootPosition(LowlevelState &state, int id, FrameType frame){
    Vec34 qLegs= state.getQ();

    if(frame == FrameType::BODY){
        return _Legs[id]->calcPEe2B(qLegs.col(id));
    }else if(frame == FrameType::HIP){
        return _Legs[id]->calcPEe2H(qLegs.col(id));
    }else{
        std::cout << "[ERROR] The frame of function: getFootPosition can only be BODY or HIP." << std::endl;
        exit(-1);
    }
}

// Forward derivative Kinematics
Vec3 QuadrupedRobot::getFootVelocity(LowlevelState &state, int id){
    Vec34 qLegs = state.getQ();
    Vec34 qdLegs= state.getQd();
    return _Legs[id]->calcVEe(qLegs.col(id), qdLegs.col(id));
}

// Forward Kinematics
Vec34 QuadrupedRobot::getFeet2BPositions(LowlevelState &state, FrameType frame){
    Vec34 feetPos;
    if(frame == FrameType::GLOBAL){
        for(int i(0); i<4; ++i){
            feetPos.col(i) = getFootPosition(state, i, FrameType::BODY);
        }
        feetPos = state.getRotMat() * feetPos;
    }
    else if((frame == FrameType::BODY) || (frame == FrameType::HIP)){
        for(int i(0); i<4; ++i){
            feetPos.col(i) = getFootPosition(state, i, frame);
        }
    }
    else{
        std::cout << "[ERROR] Frame error of function getFeet2BPositions" << std::endl;
        exit(-1);
    }
    return feetPos;
}

Vec34 QuadrupedRobot::getFeet2BVelocities(LowlevelState &state, FrameType frame){
    Vec34 feetVel;
    for(int i(0); i<4; ++i){
        feetVel.col(i) = getFootVelocity(state, i);
    }

    if(frame == FrameType::GLOBAL){
        Vec34 feetPos = getFeet2BPositions(state, FrameType::BODY);
        feetVel += skew(state.getGyro()) * feetPos;
        return state.getRotMat() * feetVel;
    }
    else if((frame == FrameType::BODY) || (frame == FrameType::HIP)){
        return feetVel;
    }
    else{
        std::cout << "[ERROR] Frame error of function getFeet2BVelocities" << std::endl;
        exit(-1);
    }   
}

Mat3 QuadrupedRobot::getJaco(LowlevelState &state, int legID){
    return _Legs[legID]->calcJaco(state.getQ().col(legID));
}

A1Robot::A1Robot(){
    _Legs[0] = new A1Leg(0, Vec3( 0.1805, -0.047, 0));
    _Legs[1] = new A1Leg(1, Vec3( 0.1805,  0.047, 0));
    _Legs[2] = new A1Leg(2, Vec3(-0.1805, -0.047, 0));
    _Legs[3] = new A1Leg(3, Vec3(-0.1805,  0.047, 0));

    _feetPosNormalStand <<  0.1805,  0.1805, -0.1805, -0.1805, 
                           -0.1308,  0.1308, -0.1308,  0.1308,
                           -0.3180, -0.3180, -0.3180, -0.3180;

    _robVelLimitX << -0.4, 0.4;
    _robVelLimitY << -0.3, 0.3;
    //_robVelLimitYaw << -0.5, 0.5;
    _robVelLimitYaw << -0.3, 0.3;
#ifdef COMPILE_WITH_REAL_ROBOT
    _mass = 12.5;
    _pcb << 0.01, 0.0, 0.0;
    _Ib = Vec3(0.132, 0.3475, 0.3775).asDiagonal();
#endif  // COMPILE_WITH_REAL_ROBOT

#ifdef COMPILE_WITH_SIMULATION
    _mass = 13.4;
    _pcb << 0.0, 0.0, 0.0;
    _Ib = Vec3(0.132, 0.3475, 0.3775).asDiagonal();
#endif  // COMPILE_WITH_SIMULATION
}

Go1Robot::Go1Robot(){
    _Legs[0] = new Go1Leg(0, Vec3( 0.1881, -0.04675, 0));
    _Legs[1] = new Go1Leg(1, Vec3( 0.1881,  0.04675, 0));
    _Legs[2] = new Go1Leg(2, Vec3(-0.1881, -0.04675, 0));
    _Legs[3] = new Go1Leg(3, Vec3(-0.1881,  0.04675, 0));

    _feetPosNormalStand <<  0.1881,  0.1881, -0.1881, -0.1881,
                           -0.1300,  0.1300, -0.1300,  0.1300,
                           -0.3200, -0.3200, -0.3200, -0.3200;

    _robVelLimitX << -0.4, 0.4;
    _robVelLimitY << -0.3, 0.3;
    //_robVelLimitYaw << -0.5, 0.5;
    _robVelLimitYaw << -0.3, 0.3;

#ifdef COMPILE_WITH_REAL_ROBOT
    _mass = 10.5;
    _pcb << 0.04, 0.0, 0.0;
    _Ib = Vec3(0.0792, 0.2085, 0.2265).asDiagonal();
#endif  // COMPILE_WITH_REAL_ROBOT

#ifdef COMPILE_WITH_SIMULATION
    _mass = 12.0;
    _pcb << 0.0, 0.0, 0.0;
    _Ib = Vec3(0.0792, 0.2085, 0.2265).asDiagonal();
#endif  // COMPILE_WITH_SIMULATION
}

// bool Go1Robot:: BuildDynamicModel()
// {
//     // we assume the cheetah's body (not including rotors) can be modeled as a
//     // uniformly distributed box.
//     std::vector<float> bodySize = {0.3762, 0.0935, 0.114}; // Length, Width, Height
//     Eigen::Matrix<float,3, 1> bodyDims(bodySize[0], bodySize[1], bodySize[2]);
//     float hipLength=0.085;
//     float upperLegLength=0.213;
//     float lowerLegLength=0.213;
//     /*
//     abad_location: [0.1881, 0.04675, 0]
//     hip_l: 0.085
//     upper_l: 0.213
//     lower_l: 0.213
//     */
//     // locations
//     Eigen::Matrix<float,3, 1> _abadRotorLocation = {0.14f, 0.04675f, 0.f};//？？？
//     //Eigen::Matrix<float,3, 1> _abadLocation = {0.1805f, 0.047f, 0.f};
//     Eigen::Matrix<float,3, 1> _abadLocation = {0.1881f, 0.04675f, 0.f};
//     Eigen::Matrix<float,3, 1> _hipLocation = Eigen::Matrix<float,3, 1>(0, hipLength, 0);
//     Eigen::Matrix<float,3, 1> _hipRotorLocation = Eigen::Matrix<float,3, 1>(0, 0.04, 0);
//     Eigen::Matrix<float,3, 1> _kneeLocation = Eigen::Matrix<float,3, 1>(0, 0, -upperLegLength);
//     Eigen::Matrix<float,3, 1> _kneeRotorLocation = Eigen::Matrix<float,3, 1>(0, 0, 0);

//     float scale_ = 1e-2;
//     // rotor inertia if the rotor is oriented so it spins around the z-axis
//     Eigen::Matrix<float,3, 3> rotorRotationalInertiaZ;
//     rotorRotationalInertiaZ << 33, 0, 0,
//                                0, 33, 0,
//                                0, 0, 63;
//     rotorRotationalInertiaZ.setIdentity();
//     rotorRotationalInertiaZ = scale_*1e-6 * rotorRotationalInertiaZ;

//     Eigen::Matrix<float,3, 3> RY = coordinateRotation<float>(CoordinateAxis::Y, M_PI / 2);
//     Eigen::Matrix<float,3, 3> RX = coordinateRotation<float>(CoordinateAxis::X, M_PI / 2);
//     Eigen::Matrix<float,3, 3> rotorRotationalInertiaX = RY * rotorRotationalInertiaZ * RY.transpose();
//     Eigen::Matrix<float,3, 3> rotorRotationalInertiaY = RX * rotorRotationalInertiaZ * RX.transpose();

//     // spatial inertias
//     Eigen::Matrix<float,3, 3> abadRotationalInertia;
//     // abadRotationalInertia << 469.2, -9.4, -0.342,
//     //                          -9.4, 807.5, -0.466,
//     //                          -0.342, -0.466,  552.9;
//     abadRotationalInertia << 305.3, -7.788, 0.220,
//                              -7.788, 590.9, -0.0172,
//                              0.220, -0.0172,  396.6;
//     abadRotationalInertia.setIdentity();
//     abadRotationalInertia = abadRotationalInertia * 1e-6;
//     // Eigen::Matrix<float,3, 1> abadCOM(0, 0.036, 0);  // mini-cheetah
//     //Eigen::Matrix<float,3, 1> abadCOM(-0.0033, 0, 0);//A1
//     Eigen::Matrix<float,3, 1> abadCOM(-0.00541, 0, 0);//go1
//     SpatialInertia<float> abadInertia(0.510299, abadCOM, abadRotationalInertia);

//     Eigen::Matrix<float,3, 3> hipRotationalInertia;
//     // hipRotationalInertia << 5529, 4.825, 343.9,
//     //                         4.825, 5139.3, 22.4,
//     //                         343.9, 22.4, 1367.8;
//     hipRotationalInertia << 5395.9, 0.1028, 337.5,
//                             0.1028, 5142.5, 5.82,
//                             337.5, 5.82, 1024.8;
//      hipRotationalInertia.setIdentity();
//     hipRotationalInertia = hipRotationalInertia * 1e-6;
//     // Eigen::Matrix<float,3, 1> hipCOM(0, 0.016, -0.02);
//     //Eigen::Matrix<float,3, 1> hipCOM(-0.003237, -0.022327, -0.027326); // left, for right filp y-axis value.
//     Eigen::Matrix<float,3, 1> hipCOM(-0.003468, -0.018947, -0.032736);//go1
//     SpatialInertia<float> hipInertia(0.8989, hipCOM, hipRotationalInertia);
//     std::cout << "hipInertia -----" <<std::endl;
//     std::cout << hipInertia.getInertiaTensor() << std::endl;
//     std::cout << hipInertia.flipAlongAxis(CoordinateAxis::Y).getInertiaTensor() << std::endl;
//     std::cout << "----- hipInertia " <<std::endl;
    
    
//     Eigen::Matrix<float,3, 3> kneeRotationalInertia, kneeRotationalInertiaRotated;
//     // kneeRotationalInertiaRotated << 2998, 0,   -141.2,
//     //                                 0,    3014, 0,
//     //                                 -141.2, 0,   32.4;
//     kneeRotationalInertiaRotated << 3607.6, 1.495,   -132.78,
//                                     1.495,    3626.78, 28.64,
//                                     -132.78, 28.64,   35.15;
//     // kneeRotationalInertiaRotated.setIdentity();
//     kneeRotationalInertiaRotated = kneeRotationalInertiaRotated * 1e-6;
//     kneeRotationalInertia = kneeRotationalInertiaRotated;//RY * kneeRotationalInertiaRotated * RY.transpose();
//     // Eigen::Matrix<float,3, 1> kneeCOM(0, 0, -0.061);
//     //Eigen::Matrix<float,3, 1> kneeCOM(0.006435, 0, -0.107);
//     Eigen::Matrix<float,3, 1> kneeCOM(0.006286, 0.001307, -0.122269);
//     SpatialInertia<float> kneeInertia(0.158, kneeCOM, kneeRotationalInertia);

//     Eigen::Matrix<float,3, 1> rotorCOM(0, 0, 0);
//     float rotorMass = 1e-8; //0.055
//     SpatialInertia<float> rotorInertiaX(rotorMass, rotorCOM, rotorRotationalInertiaX);
//     SpatialInertia<float> rotorInertiaY(rotorMass, rotorCOM, rotorRotationalInertiaY);

//     Eigen::Matrix<float,3, 3> bodyRotationalInertia;
//     // bodyRotationalInertia << 15853, 0, 0,
//     //                          0, 37799, 0,
//     //                          0, 0, 45654;
//     bodyRotationalInertia << 16130.7, 593.2, 7.32,
//                              593.2, 36507.8, 20.97,
//                              7.32, 20.97, 44693.9;   
//     bodyRotationalInertia = bodyRotationalInertia * 1e-6;
//     //Eigen::Matrix<float,3, 1> bodyCOM(0, 0, 0);
//     Eigen::Matrix<float,3, 1> bodyCOM(0.011611, 0.004437, 0.000108);
//     // Eigen::Matrix<float,3, 1> bodyCOM(0, 0.004, -0.0005);
//     SpatialInertia<float> bodyInertia(4.8, bodyCOM, bodyRotationalInertia);
    
//     model.addBase(bodyInertia);
    
//     // add contact for the cheetah's body
//     model.addGroundContactBoxPoints(5, bodyDims);

//     const int baseID = 5;
//     int bodyID = baseID;
//     float sideSign = -1;

//     Eigen::Matrix<float,3, 3> I_3 = Eigen::Matrix<float,3, 3>::Identity();
    
//     auto& abadRotorInertia = rotorInertiaX;
//     float abadGearRatio = 1; // 6
//     auto& hipRotorInertia = rotorInertiaY;
//     float hipGearRatio = 1; // 6
//     auto& kneeRotorInertia = rotorInertiaY;
//     float kneeGearRatio = 1; // 9.33
//     float kneeLinkY_offset = 0.004;
    
//     // loop over 4 legs
//     for (int legID = 0; legID < 4; legID++) {
//         // Ab/Ad joint
//         //  int addBody(const SpatialInertia<T>& inertia, const SpatialInertia<T>&
//         //  rotorInertia, T gearRatio,
//         //              int parent, JointType jointType, CoordinateAxis jointAxis,
//         //              const Mat6<T>& Xtree, const Mat6<T>& Xrot);
//         bodyID++;
//         Eigen::Matrix<float,6,6> xtreeAbad = spatial::createSXform(I_3, WithLegSigns(_abadLocation, legID));//
//         Eigen::Matrix<float,6,6> xtreeAbadRotor = spatial::createSXform(I_3, WithLegSigns(_abadRotorLocation, legID));//
//         if (sideSign < 0) {
//             model.addBody(abadInertia.flipAlongAxis(CoordinateAxis::Y),
//                           abadRotorInertia.flipAlongAxis(CoordinateAxis::Y),
//                           abadGearRatio, baseID, JointType::Revolute,
//                           CoordinateAxis::X, xtreeAbad, xtreeAbadRotor);
//         } else {
//             model.addBody(abadInertia, abadRotorInertia, abadGearRatio, baseID,
//                           JointType::Revolute, CoordinateAxis::X, xtreeAbad,
//                           xtreeAbadRotor);
//         }

//         // Hip Joint
//         bodyID++;
//         Eigen::Matrix<float,6,6> xtreeHip =
//             spatial::createSXform(I_3, //coordinateRotation(CoordinateAxis::Z, float(M_PI)),
//                         WithLegSigns(_hipLocation, legID)); // 0, hipLength=0.085, 0
//         Eigen::Matrix<float,6,6> xtreeHipRotor =
//             spatial::createSXform(coordinateRotation(CoordinateAxis::Z, float(M_PI)),
//                         WithLegSigns(_hipRotorLocation, legID));
//         if (sideSign < 0) {
//             model.addBody(hipInertia.flipAlongAxis(CoordinateAxis::Y),
//                           hipRotorInertia.flipAlongAxis(CoordinateAxis::Y),
//                           hipGearRatio, bodyID - 1, JointType::Revolute,
//                           CoordinateAxis::Y, xtreeHip, xtreeHipRotor);
//         } else {
//             model.addBody(hipInertia, hipRotorInertia, hipGearRatio, bodyID - 1,
//                           JointType::Revolute, CoordinateAxis::Y, xtreeHip,
//                           xtreeHipRotor);
//         }

//         // add knee ground contact point
//         model.addGroundContactPoint(bodyID, Eigen::Matrix<float,3, 1>(0, 0, -upperLegLength));

//         // Knee Joint
//         bodyID++;
//         Eigen::Matrix<float,6,6> xtreeKnee = spatial::createSXform(I_3, _kneeLocation);
//         Eigen::Matrix<float,6,6> xtreeKneeRotor = spatial::createSXform(I_3, _kneeRotorLocation);
//         if (sideSign < 0) {
//             model.addBody(kneeInertia, //.flipAlongAxis(CoordinateAxis::Y),
//                           kneeRotorInertia.flipAlongAxis(CoordinateAxis::Y),
//                           kneeGearRatio, bodyID - 1, JointType::Revolute,
//                           CoordinateAxis::Y, xtreeKnee, xtreeKneeRotor);

//             model.addGroundContactPoint(bodyID, Eigen::Matrix<float,3, 1>(0, kneeLinkY_offset, -lowerLegLength), true);
//         } else {
//             model.addBody(kneeInertia, kneeRotorInertia, kneeGearRatio, bodyID - 1,
//                           JointType::Revolute, CoordinateAxis::Y, xtreeKnee,
//                           xtreeKneeRotor);

//             model.addGroundContactPoint(bodyID, Eigen::Matrix<float,3, 1>(0, -kneeLinkY_offset, -lowerLegLength), true);
//         }

//         // add foot
//         //model.addGroundContactPoint(bodyID, Vec3<T>(0, 0, -_kneeLinkLength), true);

//         sideSign *= -1;
//     }
    
//     Eigen::Matrix<float,3, 1> g(0, 0, -9.81);
//     model.setGravity(g);
//     std::cout << "ok " <<std::endl;
//     // bool test_fb = false;
//     // if (test_fb) {
//     //     FBModelState<float> fb;
//     //     // for (size_t i(0); i < 3; ++i) {
//     //     // _state.bodyVelocity[i] = omegaBody[i]; // in body frame
//     //     // _state.bodyVelocity[i + 3] = vBody[i];

//     //     //     for (size_t leg(0); leg < 4; ++leg) {
//     //     //         _state.q[3 * leg + i] = q[3 * leg + i];
//     //     //         _state.qd[3 * leg + i] = dq[3 * leg + i];
//     //     //         _full_config[3 * leg + i + 6] = _state.q[3 * leg + i];
//     //     //     }
//     //     // }
//     //     printf("339\n");
//     //     fb.bodyOrientation << 1,0,0,0;//0.896127, 0.365452,0.246447,-0.0516205;
//     //     // fb.bodyPosition << 0.00437649, 0.000217693, 0.285963;
//     //     fb.bodyVelocity <<  3,3,3, 0.2, 0.1, 0.1;
//     //     fb.bodyPosition.setZero();
//     //     printf("343\n");
//     //     fb.q.resize(12,1);
//     //     fb.q.setZero();
//     //     fb.q << 0.2, 0, -0.2,
//     //             0.2, 0, -0.2, // 0, 0.7, 0,
//     //             0, 0.3, 0.5, // 0, 0.8, 0
//     //             0, 0.3, 0.5,
//     //     fb.qd.resize(12, 1);
//     //     fb.qd.setZero();
//     //     // fb.qd << 0.2, 0, 0,
//     //     //             0.1, 0, 0.,
//     //     //             0, -0.3, 0.6,
//     //     //             0, 0.3, 1;
                    
//     //     printf("346\n");
//     //     model.setState(fb);
//     //     printf("348\n");
//     //     model.forwardKinematics();
//     //     model.massMatrix();
//     //     Eigen::MatrixXf A;
//     //     Eigen::Matrix<float,18,1> dq;
//     //     dq << fb.bodyVelocity, fb.qd;
//     //     // model.generalizedGravityForce();
//     //     // model.generalizedCoriolisForce();
//     //     A = model.getMassMatrix();
//     //     for (int i=0; i <18 ; ++i) {
//     //         for (int j=0; j<18; ++j) {
//     //             if (A(i,j)<1e-6) A(i,j) = 0;
//     //         }
//     //     }
//     //     std::cout << "A = \n" << A << std::endl;
//     //     float energy = 0.5*dq.dot(A*dq);
//     //     std::cout << "energy = " << energy << std::endl;
        
//     //     // model.getPosition(8);
//     //     printf("351\n");
//     //     throw std::domain_error("finished!!!!");
//     // }
//     std::cout << "ok " <<std::endl;
//     return true;
    
// }
bool Go1Robot:: BuildDynamicModel()
{
    // we assume the cheetah's body (not including rotors) can be modeled as a
    // uniformly distributed box.
    std::vector<float> bodySize = {0.3762, 0.0935, 0.114};
    wbc::Vec3<float> bodyDims(bodySize[0], bodySize[1], bodySize[2]);
    
    // locations
    wbc::Vec3<float> _abadRotorLocation = {0.11215f, 0.04675f, 0.f};//？？？
    //wbc::Vec3<float> _abadLocation = {0.1805f, 0.047f, 0.f};
    wbc::Vec3<float> _abadLocation = {0.1881f, 0.04675f, 0.f};
    wbc::Vec3<float> _hipLocation = wbc::Vec3<float>(0, 0.08, 0);
    wbc::Vec3<float> _hipRotorLocation = wbc::Vec3<float>(0, -0.00015, 0);
    wbc::Vec3<float> _kneeLocation = wbc::Vec3<float>(0, 0, -0.213);
    wbc::Vec3<float> _kneeRotorLocation = wbc::Vec3<float>(0, -0.03235, 0);

    float scale_ = 1e-2;
    // rotor inertia if the rotor is oriented so it spins around the z-axis
    /*电机转子*/
 
    //rotor
    wbc::Mat3<float> hiprotorRotationalInertiaZ;
    wbc::Mat3<float> thighrotorRotationalInertiaZ;
    wbc::Mat3<float> calfrotorRotationalInertiaZ;
    hiprotorRotationalInertiaZ<<0.000111842,0.0,0.0,
                                0.0,0.000059647,0.0,
                                0.0,0.0,0.000059647;
    thighrotorRotationalInertiaZ<<  0.000059647,0.0,0.0,
                                    0.0,0.000111842,0.0,
                                    0.0,0.0,0.000059647;
    calfrotorRotationalInertiaZ<<0.000059647,0.0,0.0,
                                 0.0,0.000111842,0.0,
                                 0.0,0.0,0.000059647;
    wbc::Mat3<float> RY = coordinateRotation<float>(CoordinateAxis::Y, M_PI / 2);
    wbc::Mat3<float> RX = coordinateRotation<float>(CoordinateAxis::X, M_PI / 2);  
    wbc::Mat3<float> hiprotorRotationalInertiaX=RY * hiprotorRotationalInertiaZ * RY.transpose();
    wbc::Mat3<float> thighrotorRotationalInertiaY=RX * thighrotorRotationalInertiaZ * RX.transpose();
    wbc::Mat3<float> calfrotorRotationalInertiaY=RX * calfrotorRotationalInertiaZ * RX.transpose();
    float rotorMass = 0.089; //0.055
    wbc::Vec3<float> rotorCOM(0, 0, 0);
    SpatialInertia<float> hiprotorInertiaX(rotorMass, rotorCOM, hiprotorRotationalInertiaX);
    SpatialInertia<float> thighrotorInertiaY(rotorMass, rotorCOM, thighrotorRotationalInertiaY);
    SpatialInertia<float> calfrotorInertiaY(rotorMass, rotorCOM, calfrotorRotationalInertiaY);
    
    // spatial inertias
    // hip link
 
    wbc::Mat3<float> abadRotationalInertia;
    // abadRotationalInertia << 469.2, -9.4, -0.342,
    //                          -9.4, 807.5, -0.466,
    //                          -0.342, -0.466,  552.9;
    abadRotationalInertia << 0.000334008405, -0.000010826066, 0.000001290732,
                             -0.000010826066, 0.000619101213, 0.000001643194,
                             0.000001290732, 0.000001643194,  0.00040057614;
    // abadRotationalInertia.setIdentity();
    // abadRotationalInertia = abadRotationalInertia * 1e-6;
    // wbc::Vec3<float> abadCOM(0, 0.036, 0);  // mini-cheetah
    //wbc::Vec3<float> abadCOM(-0.0033, 0, 0);//A1
    wbc::Vec3<float> abadCOM(-0.005657, -0.008752, -0.000102);//go1
    SpatialInertia<float> abadInertia(0.591, abadCOM, abadRotationalInertia);
    
    //thigh link
    
    //thigh link
    wbc::Mat3<float> hipRotationalInertia;
    // hipRotationalInertia << 5529, 4.825, 343.9,
    //                         4.825, 5139.3, 22.4,
    //                         343.9, 22.4, 1367.8;
    hipRotationalInertia << 0.004431760472, 0.000057496807, -0.000218457134,
                            0.000057496807, 0.004485671726, 0.000572001265,
                            -0.000218457134, 0.000572001265, 0.000740309489;
    // hipRotationalInertia.setIdentity();
    // hipRotationalInertia = hipRotationalInertia * 1e-6;
    // wbc::Vec3<float> hipCOM(0, 0.016, -0.02);
    //wbc::Vec3<float> hipCOM(-0.003237, -0.022327, -0.027326); // left, for right filp y-axis value.
    wbc::Vec3<float> hipCOM(-0.003342, -0.018054, -0.033451);//go1
    SpatialInertia<float> hipInertia(0.92, hipCOM, hipRotationalInertia);
    std::cout << "hipInertia -----" <<std::endl;
    std::cout << hipInertia.getInertiaTensor() << std::endl;
    std::cout << hipInertia.flipAlongAxis(CoordinateAxis::Y).getInertiaTensor() << std::endl;
    std::cout << "----- hipInertia " <<std::endl;

    
    // calf link
   
// calf link
    wbc::Mat3<float> kneeRotationalInertia, kneeRotationalInertiaRotated;
    // kneeRotationalInertiaRotated << 2998, 0,   -141.2,
    //                                 0,    3014, 0,
    //                                 -141.2, 0,   32.4;
    kneeRotationalInertiaRotated << 0.001088793059, -0.000000255679, 0.000007117814,
                                    -0.000000255679,    0.001100428748, 0.000002077264,
                                    0.000007117814, 0.000002077264,   0.000024787446;
    // kneeRotationalInertiaRotated.setIdentity();
    // kneeRotationalInertiaRotated = kneeRotationalInertiaRotated * 1e-6;
    kneeRotationalInertia = kneeRotationalInertiaRotated;//RY * kneeRotationalInertiaRotated * RY.transpose();
    // wbc::Vec3<float> kneeCOM(0, 0, -0.061);
    //wbc::Vec3<float> kneeCOM(0.006435, 0, -0.107);
    wbc::Vec3<float> kneeCOM(0.006197, 0.001408, -0.116695);
    SpatialInertia<float> kneeInertia(0.135862, kneeCOM, kneeRotationalInertia);
    
    // wbc::Vec3<float> rotorCOM(0, 0, 0);
  
    //body link
    wbc::Mat3<float> bodyRotationalInertia;
    // bodyRotationalInertia << 15853, 0, 0,
    //                          0, 37799, 0,
    //                          0, 0, 45654;
    bodyRotationalInertia << 0.0168128557, -0.0002296769, -0.0002945293,
                             -0.0002296769, 0.063009565, -0.0000418731,
                             -0.0002945293, -0.0000418731, 0.0716547275;   
    // bodyRotationalInertia = bodyRotationalInertia * 1e-6;
    //wbc::Vec3<float> bodyCOM(0, 0, 0);
    wbc::Vec3<float> bodyCOM(0.0223, 0.002, -0.0005);
    // wbc::Vec3<float> bodyCOM(0, 0.004, -0.0005);
    SpatialInertia<float> bodyInertia(5.204, bodyCOM, bodyRotationalInertia);
    
    model.addBase(bodyInertia);
    // add contact for the cheetah's body
    model.addGroundContactBoxPoints(5, bodyDims);

    const int baseID = 5;
    int bodyID = baseID;
    float sideSign = -1;

    wbc::Mat3<float> I_3 = wbc::Mat3<float>::Identity();

    auto& abadRotorInertia = hiprotorInertiaX;
    float abadGearRatio = 1; // 6
    auto& hipRotorInertia = thighrotorInertiaY;
    float hipGearRatio = 1; // 6
    auto& kneeRotorInertia = calfrotorInertiaY;
    float kneeGearRatio = 1; // 9.33
    float kneeLinkY_offset = 0.004;

    // loop over 4 legs
    for (int legID = 0; legID < 4; legID++) {
        // Ab/Ad joint
        //  int addBody(const SpatialInertia<T>& inertia, const SpatialInertia<T>&
        //  rotorInertia, T gearRatio,
        //              int parent, JointType jointType, CoordinateAxis jointAxis,
        //              const wbc::Mat6<T>& Xtree, const wbc::Mat6<T>& Xrot);
        bodyID++;
        wbc::Mat6<float> xtreeAbad = createSXform(I_3, WithLegSigns(_abadLocation, legID));//
        wbc::Mat6<float> xtreeAbadRotor = createSXform(I_3, WithLegSigns(_abadRotorLocation, legID));//
        if (sideSign < 0) {
            model.addBody(abadInertia.flipAlongAxis(CoordinateAxis::Y),
                          abadRotorInertia.flipAlongAxis(CoordinateAxis::Y),
                          abadGearRatio, baseID, JointType::Revolute,
                          CoordinateAxis::X, xtreeAbad, xtreeAbadRotor);
        } else {
            model.addBody(abadInertia, abadRotorInertia, abadGearRatio, baseID,
                          JointType::Revolute, CoordinateAxis::X, xtreeAbad,
                          xtreeAbadRotor);
        }

        // Hip Joint
        bodyID++;
        wbc::Mat6<float> xtreeHip =
            createSXform(I_3, //coordinateRotation(CoordinateAxis::Z, float(M_PI)),
                        WithLegSigns(_hipLocation, legID)); // 0, hipLength=0.085, 0
        wbc::Mat6<float> xtreeHipRotor =
            createSXform(coordinateRotation(CoordinateAxis::Z, float(M_PI)),
                        WithLegSigns(_hipRotorLocation, legID));
        if (sideSign < 0) {
            model.addBody(hipInertia.flipAlongAxis(CoordinateAxis::Y),
                          hipRotorInertia.flipAlongAxis(CoordinateAxis::Y),
                          hipGearRatio, bodyID - 1, JointType::Revolute,
                          CoordinateAxis::Y, xtreeHip, xtreeHipRotor);
        } else {
            model.addBody(hipInertia, hipRotorInertia, hipGearRatio, bodyID - 1,
                          JointType::Revolute, CoordinateAxis::Y, xtreeHip,
                          xtreeHipRotor);
        }

        // add knee ground contact point
        model.addGroundContactPoint(bodyID, wbc::Vec3<float>(0, 0, -0.213));

        // Knee Joint
        bodyID++;
        wbc::Mat6<float> xtreeKnee = createSXform(I_3, _kneeLocation);
        wbc::Mat6<float> xtreeKneeRotor = createSXform(I_3, _kneeRotorLocation);
        if (sideSign < 0) {
            model.addBody(kneeInertia, //.flipAlongAxis(CoordinateAxis::Y),
                          kneeRotorInertia.flipAlongAxis(CoordinateAxis::Y),
                          kneeGearRatio, bodyID - 1, JointType::Revolute,
                          CoordinateAxis::Y, xtreeKnee, xtreeKneeRotor);

            model.addGroundContactPoint(bodyID, wbc::Vec3<float>(0, kneeLinkY_offset, -0.213), true);
        } else {
            model.addBody(kneeInertia, kneeRotorInertia, kneeGearRatio, bodyID - 1,
                          JointType::Revolute, CoordinateAxis::Y, xtreeKnee,
                          xtreeKneeRotor);

            model.addGroundContactPoint(bodyID, wbc::Vec3<float>(0, -kneeLinkY_offset, -0.213), true);
        }

        // add foot
        //model.addGroundContactPoint(bodyID, wbc::Vec3<T>(0, 0, -_kneeLinkLength), true);

        sideSign *= -1;
    }

    wbc::Vec3<float> g(0, 0, -9.81);
    model.setGravity(g);

    bool test_fb = false;
    if (test_fb) {
        FBModelState<float> fb;
        // for (size_t i(0); i < 3; ++i) {
        // _state.bodyVelocity[i] = omegaBody[i]; // in body frame
        // _state.bodyVelocity[i + 3] = vBody[i];

        //     for (size_t leg(0); leg < 4; ++leg) {
        //         _state.q[3 * leg + i] = q[3 * leg + i];
        //         _state.qd[3 * leg + i] = dq[3 * leg + i];
        //         _full_config[3 * leg + i + 6] = _state.q[3 * leg + i];
        //     }
        // }
        printf("339\n");
        fb.bodyOrientation << 1,0,0,0;//0.896127, 0.365452,0.246447,-0.0516205;
        // fb.bodyPosition << 0.00437649, 0.000217693, 0.285963;
        fb.bodyVelocity <<  3,3,3, 0.2, 0.1, 0.1;
        fb.bodyPosition.setZero();
        printf("343\n");
        fb.q.resize(12,1);
        fb.q.setZero();
        fb.q << 0.2, 0, -0.2,
                0.2, 0, -0.2, // 0, 0.7, 0,
                0, 0.3, 0.5, // 0, 0.8, 0
                0, 0.3, 0.5,
        fb.qd.resize(12, 1);
        fb.qd.setZero();
        // fb.qd << 0.2, 0, 0,
        //             0.1, 0, 0.,
        //             0, -0.3, 0.6,
        //             0, 0.3, 1;
                    
        printf("346\n");
        model.setState(fb);
        printf("348\n");
        model.forwardKinematics();
        model.massMatrix();
        Eigen::MatrixXf A;
        Eigen::Matrix<float,18,1> dq;
        dq << fb.bodyVelocity, fb.qd;
        // model.generalizedGravityForce();
        // model.generalizedCoriolisForce();
        A = model.getMassMatrix();
        for (int i=0; i <18 ; ++i) {
            for (int j=0; j<18; ++j) {
                if (A(i,j)<1e-6) A(i,j) = 0;
            }
        }
        std::cout << "A = \n" << A << std::endl;
        float energy = 0.5*dq.dot(A*dq);
        std::cout << "energy = " << energy << std::endl;
        
        // model.getPosition(8);
        printf("351\n");
        throw std::domain_error("finished!!!!");
    }
    return true;
}