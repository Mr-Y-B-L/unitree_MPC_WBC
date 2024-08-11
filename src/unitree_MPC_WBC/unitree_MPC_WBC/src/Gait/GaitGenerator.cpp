/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "Gait/GaitGenerator.h"

GaitGenerator::GaitGenerator(CtrlComponents *ctrlComp)
              : _waveG(ctrlComp->waveGen), _est(ctrlComp->estimator), 
                _phase(ctrlComp->phase), _contact(ctrlComp->contact), 
                _robModel(ctrlComp->robotModel), _state(ctrlComp->lowState){
    _feetCal = new FeetEndCal(ctrlComp);
    _firstRun = true;
}

GaitGenerator::~GaitGenerator(){
}

void GaitGenerator::setGait(Vec2 vxyGoalGlobal, float dYawGoal, float gaitHeight){
    _vxyGoal = vxyGoalGlobal;
    _dYawGoal = dYawGoal;
    _gaitHeight = gaitHeight;
}

void GaitGenerator::restart(){
    _firstRun = true;
    _vxyGoal.setZero();
}

void GaitGenerator::run(Vec34 &feetPos, Vec34 &feetVel,Vec34 &feetAcc){
    if(_firstRun){
        _startP = _est->getFeetPos();
        _firstRun = false;
    }

    for(int i(0); i<4; ++i){
        if((*_contact)(i) == 1){
            if((*_phase)(i) < 0.5){
                _startP.col(i) = _est->getFootPos(i);
            }
            feetPos.col(i) = _startP.col(i);
            feetVel.col(i).setZero();
            feetAcc.col(i).setZero();
        }
        else{
            _endP.col(i) = _feetCal->calFootPos(i, _vxyGoal, _dYawGoal, (*_phase)(i));

            feetPos.col(i) = getFootPos(i);
            feetVel.col(i) = getFootVel(i);
            feetAcc.col(i) = getFootAcc(i);
        }
    }
    _pastP = feetPos;
    _phasePast = *_phase;
}

Vec3 GaitGenerator::getFootPos(int i){
    Vec3 footPos;

    footPos(0) = cycloidXYPosition(_startP.col(i)(0), _endP.col(i)(0), (*_phase)(i));
    footPos(1) = cycloidXYPosition(_startP.col(i)(1), _endP.col(i)(1), (*_phase)(i));
    footPos(2) =  cycloidZPosition(_startP.col(i)(2), _gaitHeight, (*_phase)(i));
    
    return footPos;
}

Vec3 GaitGenerator::getFootVel(int i){
    Vec3 footVel;

    footVel(0) = cycloidXYVelocity(_startP.col(i)(0), _endP.col(i)(0), (*_phase)(i));
    footVel(1) = cycloidXYVelocity(_startP.col(i)(1), _endP.col(i)(1), (*_phase)(i));
    footVel(2) =  cycloidZVelocity(_gaitHeight, (*_phase)(i));

    return footVel;
}

float GaitGenerator::cycloidXYPosition(float start, float end, float phase){
    float phasePI = 2 * M_PI * phase;
    return (end - start)*(phasePI - sin(phasePI))/(2*M_PI) + start;
}

float GaitGenerator::cycloidXYVelocity(float start, float end, float phase){
    float phasePI = 2 * M_PI * phase;
    return (end - start)*(1 - cos(phasePI)) / _waveG->getTswing();
}

float GaitGenerator::cycloidZPosition(float start, float h, float phase){
    float phasePI = 2 * M_PI * phase;
    return h*(1 - cos(phasePI))/2 + start;
}

float GaitGenerator::cycloidZVelocity(float h, float phase){
    float phasePI = 2 * M_PI * phase;
    return h*M_PI * sin(phasePI) / _waveG->getTswing();
}
//WBC
float GaitGenerator::cycloidXYAcc(float start, float end, float phase){
    float phasePI = 2 * M_PI * phase;
    return ((end - start)*2 * M_PI/(_waveG->getTswing()*_waveG->getTswing()))*sin(phasePI);
}
float GaitGenerator::cycloidZAcc(float h, float phase){
    float phasePI = 2 * M_PI * phase;
    return (h*M_PI*M_PI*2/(_waveG->getTswing()*_waveG->getTswing()))*cos(phasePI);
}

Vec3 GaitGenerator::getFootAcc(int i){
    Vec3 footAcc;

    footAcc(0) = cycloidXYAcc(_startP.col(i)(0), _endP.col(i)(0), (*_phase)(i));
    footAcc(1) = cycloidXYAcc(_startP.col(i)(1), _endP.col(i)(1), (*_phase)(i));
    footAcc(2) =  cycloidZAcc(_gaitHeight, (*_phase)(i));

    return footAcc;
}
//只适用于troting 
Eigen::Matrix<double, 4, 1> GaitGenerator::getContactState(){
    Eigen::Matrix<double, 4, 1> progress=(*_phase)-Vec4(0, 0.5, 0.5, 0);
    for(int i = 0; i < 4; i++)
  {
    if(progress[i] < 0) progress[i] += 1.; 
    if(progress[i] > 0.5) //相位大于支撑结束相位，非支撑状态
    {
      progress[i] = 0.;
    }
    else
    {
      progress[i] = progress[i] / 0.5; //相位在支撑相中的百分比
    }
  }
  return progress;
}