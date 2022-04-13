#pragma once

#include <Kin/kin.h>
#include <Kin/frame.h>
#include <FlatVision/helpers.h>
#include <math.h>

inline uintA franka_getJointIndices(const rai::Configuration& C, char L_or_R){
  CHECK(C._state_indexedJoints_areGood , "need to ensure_q (indexed joints) before!");
  StringA jointNames;
  for(uint i=1;i<=7;i++){
    jointNames.append(STRING(L_or_R <<"_panda_joint" <<i));
  }
  FrameL joints = C.getFrames(jointNames);
  uintA qIndices(7);
  for(uint i=0;i<joints.N;i++) qIndices(i) = joints(i)->joint->qIndex;
  return qIndices;
}


inline byteA franka_getFrameMaskMap(const rai::Configuration& K){
  byteA frameMaskMap(K.frames.N); //map each frame in the image to a mask byte (here just 0 or 0xff)
  frameMaskMap.setZero();
  for(rai::Frame *f:K.frames){
    if(f->shape){
      if(f->getUpwardLink()->name.startsWith("l_")){
        frameMaskMap(f->ID)=PL_robot;
      }
      if(f->getUpwardLink()->name.startsWith("r_")){
        frameMaskMap(f->ID)=PL_robot+1;
      }
      if(f->getUpwardLink()->name.startsWith("perc_")){
        int id;
        f->getUpwardLink()->name >>"perc_" >>id;
        frameMaskMap(f->ID)=PixelLabel(PL_robot+id);
      }
//      cout <<f->ID <<' ' <<f->name <<' ' <<frameMaskMap(f->ID) <<endl;
    }
  }
  return frameMaskMap;
}

inline void franka_setFrameMaskMapLabels(rai::Configuration& K){
  for(rai::Frame *f:K.frames){
    if(f->shape){
      if(f->getUpwardLink()->name.startsWith("l_")){
        f->ats->getNew<int>("label") = PL_robot;
      }
      if(f->getUpwardLink()->name.startsWith("r_")){
        f->ats->getNew<int>("label") = PL_robot+1;
      }
    }
  }
}

inline void sigmoidal_friction(arr & torque, const arr & v, const arr & param) {
    for (uint i=0; i<torque.N; i++){
        double speed = v(i);
        double phi1j = param(i, 0);
        double phi2j = param(i, 1);
        double phi3j = param(i, 2);
        torque(i) = phi1j / (1 + exp(-phi2j * (speed + phi3j))) - phi1j / (1+exp(-phi2j*phi3j));
    }
}

inline void update_buffer(arr & NMF_buffer, const arr & q_real, const arr & qDot_real, const arr & last_fric){
    // NMF_buffer = (62,21)
    NMF_buffer.shift(-21, false);
    NMF_buffer(61, {0, 6}) = q_real;
    NMF_buffer(61, {7, 13}) = qDot_real;
    NMF_buffer(61, {14, 20}) = last_fric;
}

inline void nonmarkovian_friction(arr & fric, arr & NMF_buffer, const arr & NMF_beta){
    NMF_buffer.reshape(1, -1);
    arr one = {1.};
    arr feature = (one, NMF_buffer);
    feature.reshape(1,1303);
    fric = feature * NMF_beta;
    fric.reshape(7);
}

inline void decoupling_friction(arr & fric, const arr & beta, const arr & q, const arr & qD){
//beta (7,103)
    arr feature_i = zeros(103);
    for(uint i=0; i<7; i++){
        // joint
        feature_i(0) = 1.;
        feature_i(1) = q(i);
        feature_i(2) = qD(i);
        for(uint j=1; j<26; j++){
            feature_i(3+4*(j-1)) = sin(j*q(i));
            feature_i(4+4*(j-1)) = sin(j*qD(i));
            feature_i(5+4*(j-1)) = cos(j*q(i));
            feature_i(6+4*(j-1)) = cos(j*qD(i));
        }

        arr beta_i = beta(i, {0,-1});
        beta_i.reshape(1, 103);
        feature_i.reshape(103, 1);
        fric(i) = (beta_i * feature_i)(0,0);
        feature_i = zeros(103);
    }
}

inline void ReLU(arr &x){
    uint l = x.N;
    for(uint i = 0; i<l; i++){
        if(x(i, 0) < 0.){
            x(i, 0) = 0.;
        }
    }
}

inline void net_forward(arr &res, const arr &x, const arr &be0, const arr &bi0, const arr &be1, const arr &bi1, const arr &be2, const arr &bi2){
    //x vector
    res = be0 * x;
    res = res + bi0;
    ReLU(res);
    res = be1 * res;
    res = res + bi1;
    ReLU(res);
    res = be2 * res;
    res = res + bi2;
}






