#include "franka.h"

#include <franka/model.h>
#include <franka/robot.h>
#include <franka/exception.h>
#include "help.h"

void naturalGains(double& Kp, double& Kd, double decayTime, double dampingRatio);

const char *frankaIpAddresses[2] = {"172.16.0.2", "172.17.0.2"};

FrankaThread::~FrankaThread(){
  stop = true;
  threadClose();
}

long c = 0;

void FrankaThread::init(uint _robotID, const uintA& _qIndices) {
  robotID=_robotID;
  qIndices=_qIndices;

  CHECK_EQ(qIndices.N, 7, "");
  qIndices_max = qIndices.max();

  //-- basic Kp Kd settings for reference control mode
  Kp_freq = rai::getParameter<arr>("Franka/Kp_freq", ARR(20., 20., 20., 20., 10., 15., 10.)); //18., 18., 18., 13., 8., 8., 6.));
  Kd_ratio = rai::getParameter<arr>("Franka/Kd_ratio", ARR(.6, .6, .4, .4, .1, .5, .1)); //.8, .8, .7, .7, .1, .1, .1));
  Ki_ratio = rai::getParameter<arr>("Franka/Ki_ratio", ARR(.6, .6, .6, .6, .3, .3, .3));
  friction = rai::getParameter<arr>("Franka/friction", zeros(7));
  fric_type = rai::getParameter<int>("Franka/fric_type", 0);
  use_residual = rai::getParameter<int>("Franka/use_residual", 0);
  LOG(0) << "FRANKA: Kp_freq:" << Kp_freq << " Kd_ratio:" << Kd_ratio <<" friction:" <<friction << " Ki_ratio" << Ki_ratio << " fric_type:" << fric_type;

  /* hand tuning result of friction calib:
     Franka/friction: [0.8, 1.0, 0.8, 1.0, 0.9, 0.5, 0.4]
     Franka/Kd_ratio: [0.6, 0.6, 0.3, 0.3, 0.3, 0.3, 0.4]
  */

  //-- choose robot/ipAddress
  CHECK_LE(robotID, 1, "");
  ipAddress = frankaIpAddresses[robotID];

  //-- start thread and wait for first state signal
  threadStep();  //this is not looping! The step method passes a callback to robot.control, which is blocking! (that's why we use a thread) until stop becomes true
  while(requiresInitialization) rai::wait(.01);
}

void FrankaThread::step(){
  // connect to robot
  franka::Robot robot(ipAddress);

  // load the kinematics and dynamics model
  franka::Model model = robot.loadModel();

  arr lastTorque = zeros(7);
  arr qDotFilter = zeros(7);
  double qDotFilterAlpha = .7;

  // set collision behavior
  robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                             {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                             {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                             {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

  // sigmoid friction parameters
  arr sig_fric_param = {4.398, 2.090, 0.003, 2.333, 6.143, -0.035, 2.824, 3.759, -0.022,
                        1.350, 45.370, -0.002, 2.534, 3.107, -0.018, 1.265, 3.264, -0.018, 0.690, 2.283, 0.003};
  // accumulative tracking error
  arr acc_error = zeros(7);

  sig_fric_param.reshape(7, 3);
  // nonmarkovian friction model
  arr NMF_beta;
  arr NMF_buffer = zeros(62, 21);
  if (fric_type==2){
      FILE("Param_NonMarkovianRR.txt") >> NMF_beta;
      NMF_beta.reshape(1303, 7);
  }

  arr beta_decoupling ;
  if (fric_type==3){
      FILE("weights/Tri_feature_decoupling.txt") >> beta_decoupling; // (7, 103)
  }

  arr beta0, bias0, beta1, bias1, beta2, bias2;
  arr last3_beta0, last3_bias0, last3_beta1, last3_bias1, last3_beta2, last3_bias2;
  if (use_residual == 1){
      FILE("weights/net.0.weight.txt") >> beta0;
      FILE("weights/net.0.bias.txt") >> bias0;
      FILE("weights/net.2.weight.txt") >> beta1;
      FILE("weights/net.2.bias.txt") >> bias1;
      FILE("weights/net.4.weight.txt") >> beta2;
      FILE("weights/net.4.bias.txt") >> bias2;

      FILE("weights/last3_net.0.weight.txt") >> last3_beta0;
      FILE("weights/last3_net.0.bias.txt") >> last3_bias0;
      FILE("weights/last3_net.2.weight.txt") >> last3_beta1;
      FILE("weights/last3_net.2.bias.txt") >> last3_bias1;
      FILE("weights/last3_net.4.weight.txt") >> last3_beta2;
      FILE("weights/last3_net.4.bias.txt") >> last3_bias2;
  }

  arr last_command = zeros(7);
  arr last_velocity = zeros(7);

  //-- initialize state and ctrl with first state
  {
    franka::RobotState initial_state = robot.readOnce();
    arr q_real, qDot_real;
    q_real.setCarray(initial_state.q.begin(), initial_state.q.size());
    qDot_real.setCarray(initial_state.dq.begin(), initial_state.dq.size());

    auto stateSet = state.set();
    auto cmdSet = cmd.set();

    //ensure state variables have sufficient size
    while(stateSet->q.N<=qIndices_max) stateSet->q.append(0.);
    while(stateSet->qDot.N<=qIndices_max) stateSet->qDot.append(0.);
    while(stateSet->tauExternal.N<=qIndices_max) stateSet->tauExternal.append(0.);

    for(uint i=0; i<7; i++){
      stateSet->q.elem(qIndices(i)) = q_real(i);
      stateSet->qDot.elem(qIndices(i)) = qDot_real(i);
      stateSet->tauExternal.elem(qIndices(i)) = 0.;
    }
  }


  //-- define the callback for the torque control loop
  std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
      torque_control_callback = [&](const franka::RobotState& robot_state,
                                    franka::Duration /*duration*/) -> franka::Torques {

    steps++;

//    if(stop) return franka::MotionFinished(franka::Torques( std::array<double, 7>{0., 0., 0., 0., 0., 0., 0.}));

    //-- get current state from libfranka
    arr q_real(robot_state.q.begin(), robot_state.q.size(), false);
    arr qDot_real(robot_state.dq.begin(), robot_state.dq.size(), false);
    arr torquesExternal_real(robot_state.tau_ext_hat_filtered.begin(), robot_state.tau_ext_hat_filtered.size(), false);
    arr torques_real(robot_state.tau_J.begin(), robot_state.tau_J.size(), false);

    qDotFilter = qDotFilterAlpha * qDotFilter + (1.-qDotFilterAlpha) * qDot_real;
    qDot_real = qDotFilter;

    //-- get real time
    ctrlTime += .001; //HARD CODED: 1kHz
    //ctrlTime = rai::realTime();

    //-- publish state
    arr state_q_real, state_qDot_real;
    {
      auto stateSet = state.set();
      stateSet->time = ctrlTime;
      for(uint i=0;i<7;i++){
        stateSet->q.elem(qIndices(i)) = q_real.elem(i);
        stateSet->qDot.elem(qIndices(i)) = qDot_real.elem(i);
        stateSet->tauExternal.elem(qIndices(i)) = torquesExternal_real.elem(i);
      }
      state_q_real = stateSet->q;
      state_qDot_real = stateSet->qDot;
    }

    //-- get current ctrl command
    arr q_ref, qDot_ref, qDDot_ref, Kp_ref, Kd_ref, P_compliance; // TODO Kp, Kd and also read out the correct indices

    // define the friction torques
    arr fric = zeros(7);
    // define the residual torques, the first 4 and last 3 joints are treated seperatly
    arr residual_dyna_4 = zeros(4,1);
    arr residual_dyna_3 = zeros(3,1);
    arr residual_dyna = zeros(7);

    rai::ControlType controlType;
    {
      auto cmdGet = cmd.get();

      controlType = cmdGet->controlType;

      //get commanded reference from the reference callback (e.g., sampling a spline reference)
      arr cmd_q_ref, cmd_qDot_ref, cmd_qDDot_ref;
      if(cmdGet->ref){
        cmdGet->ref->getReference(cmd_q_ref, cmd_qDot_ref, cmd_qDDot_ref, state_q_real, state_qDot_real, ctrlTime);
        CHECK(!cmd_q_ref.N || cmd_q_ref.N > qIndices_max, "");
        CHECK(!cmd_qDot_ref.N || cmd_qDot_ref.N > qIndices_max, "");
        CHECK(!cmd_qDDot_ref.N || cmd_qDDot_ref.N > qIndices_max, "");
      }

      //pick qIndices for this particular robot
      if(cmd_q_ref.N){
        q_ref.resize(7);
        for(uint i=0; i<7; i++) q_ref.elem(i) = cmd_q_ref.elem(qIndices(i));
      }
      if(cmd_qDot_ref.N){
        qDot_ref.resize(7);
        for(uint i=0; i<7; i++) qDot_ref.elem(i) = cmd_qDot_ref.elem(qIndices(i));
      }
      if(cmd_qDDot_ref.N){
        qDDot_ref.resize(7);
        for(uint i=0; i<7; i++) qDDot_ref.elem(i) = cmd_qDDot_ref.elem(qIndices(i));
      }
      if(cmdGet->Kp.d0 >= 7 && cmdGet->Kp.d1 >=7 && cmdGet->Kp.d0 == cmdGet->Kp.d1){
        Kp_ref.resize(7, 7);
        for(uint i=0; i<7; i++) for(uint j=0; j<7; j++) Kp_ref(i, j) = cmdGet->Kp(qIndices(i), qIndices(j));
      }
      if(cmdGet->Kd.d0 >= 7 && cmdGet->Kd.d1 >=7 && cmdGet->Kd.d0 == cmdGet->Kd.d1){
        for(uint i=0; i<7; i++) for(uint j=0; j<7; j++) Kd_ref(i, j) = cmdGet->Kd(qIndices(i), qIndices(j));
      }

      if(cmdGet->P_compliance.N) {
        HALT("NOT IMPLEMENTED YET (at least properly)")
        P_compliance = cmdGet->P_compliance;
      }

    }

    requiresInitialization=false;

    //-- cap the reference difference
    if(q_ref.N==7){
      double err = length(q_ref - q_real);
      if(err>.05){ //if(err>.02){ //stall!
        ctrlTime -= .001; //no progress in reference time!
        cout <<"STALLING - step:" <<steps <<endl;
      }
    }

    //-- grab dynamics
    arr M_org(model.mass(robot_state).begin(), 49, false);
    M_org.reshape(7,7);
    arr C_org(model.coriolis(robot_state).begin(), 7, false);
    arr G_org(model.gravity(robot_state).begin(), 7, false);

    //-- compute torques from control message depending on the control type
    arr u;

    if(controlType == rai::ControlType::configRefs) { //default: PD for given references
      //check for correct ctrl otherwise do something...
      if(q_ref.N!=7){
//        if(!(steps%10)){
//          cerr <<"FRANKA: inconsistent ctrl q_ref message - step: " <<steps <<endl;
//        }
//        return std::array<double, 7>({0., 0., 0., 0., 0., 0., 0.});
      }
      //check for correct compliance objective
      if(P_compliance.N){
        if(!(P_compliance.nd==2 && P_compliance.d0==7 && P_compliance.d1==7)){
          cerr << "FRANKA: inconsistent ctrl P_compliance message" << endl;
          P_compliance.clear();
        }
      }

      //-- compute desired torques
      arr Kp(7), Kd(7), Ki(7);
      CHECK_EQ(Kp_freq.N, 7,"");
      CHECK_EQ(Kd_ratio.N, 7,"");
      for(uint i=0;i<7;i++){
        double freq = Kp_freq.elem(i);
        Kp.elem(i) = freq*freq;
        Kd.elem(i) = 2.*Kd_ratio.elem(i)*freq;
        Ki.elem(i) = Ki_ratio.elem(i);
      }

      if(P_compliance.N){
        Kp = P_compliance * (Kp % P_compliance);
      }else{
        Kp = diag(Kp);
      }
      Ki = diag(Ki);

      //-- initialize zero torques
      u.resize(7).setZero();
      if(q_ref.N==7){
        acc_error += (q_ref - q_real);
      }



      //-- add feedback term
      if(q_ref.N==7){
        u += Kp * (q_ref - q_real);
      }
      if(qDot_ref.N==7){
        u += Kd % (qDot_ref - qDot_real);
      }
      if(q_ref.N==7){
        u += Ki * acc_error;
      }

      //-- add feedforward term
      if(qDDot_ref.N==7 && absMax(qDDot_ref)>0.){
        arr M = M_org; // + diag(ARR(0.4, 0.3, 0.3, 0.4, 0.4, 0.4, 0.2));
        u += M*qDDot_ref;
      }

      //-- add friction term
      if(friction.N==7 && qDot_ref.N==7){
        for(uint i=0;i<7;i++){
          if(qDot_ref.elem(i)>1e-4){ u.elem(i) += friction.elem(i); fric(i) = friction.elem(i);}
          if(qDot_ref.elem(i)<-1e-4){ u.elem(i) -= friction.elem(i); fric(i) = -friction.elem(i);}
        }
      }

      //-- add sigmoidal friction term
      if(qDot_ref.N == 7 && fric_type==1){
          sigmoidal_friction(fric, qDot_real, sig_fric_param);
          for(uint i=0; i<7; i++){
            u.elem(i) += fric(i);
          }
      }

      //-- add non-markovian friction term
      arr last_fric = last_command - torques_real;
      //update buffer
      update_buffer(NMF_buffer, q_real, qDot_real, last_fric);
      last_command = zeros(7);
      //
      arr NMF_buffer_temp = zeros(62, 21);
      std::copy(NMF_buffer.begin(), NMF_buffer.end(), NMF_buffer_temp.begin());
      if(q_real.N == 7 && qDot_real.N == 7 && fric_type==2){
          nonmarkovian_friction(fric, NMF_buffer_temp, NMF_beta);
      }

      //-- add markovian decoupling joint friction
      if(q_real.N == 7 && qDot_real.N == 7 && fric_type==3){
          decoupling_friction(fric, beta_decoupling, q_real, qDot_real);
          for(uint i=0; i<7; i++){
              u.elem(i) += fric(i);
          }
      }

      //-- add residual dynamics
      arr q_dq = (q_real, qDot_real);
      q_dq.reshape(14,1);
      arr ref_q_dq = zeros(14,1);
      arr e_ed = zeros(14,1);
      if (q_ref.N == 7){
          arr ref_q_dq = (q_ref, qDot_ref);
          ref_q_dq.reshape(14,1);
          e_ed = q_dq - ref_q_dq;
      }

      if(q_real.N == 7 && qDot_real.N == 7 && use_residual==1){
          net_forward(residual_dyna_4, q_dq, beta0, bias0, beta1, bias1, beta2, bias2);
          net_forward(residual_dyna_3, e_ed, last3_beta0, last3_bias0, last3_beta1, last3_bias1, last3_beta2, last3_bias2);
          arr temp_residual = (residual_dyna_4, residual_dyna_3);
          residual_dyna = temp_residual.reshape(-1);
          for(uint i=0; i<7; i++){
              u.elem(i) += residual_dyna(i);
          }
      }

      //-- project with compliance
      if(P_compliance.N) u = P_compliance * u;

    } else if(controlType == rai::ControlType::projectedAcc) { // projected Kp, Kd and u_b term for projected operational space control
      CHECK_EQ(Kp_ref.nd, 2, "")
      CHECK_EQ(Kp_ref.d0, 7, "")
      CHECK_EQ(Kp_ref.d1, 7, "")
      CHECK_EQ(Kd_ref.nd, 2, "")
      CHECK_EQ(Kd_ref.d0, 7, "")
      CHECK_EQ(Kd_ref.d1, 7, "")
      CHECK_EQ(qDDot_ref.N, 7, "")

      arr M;
      M.setCarray(model.mass(robot_state).begin(), 49);
      M.reshape(7,7);

#if 0
      c++;
      if(c%10 == 0) {
        cout << M << endl << endl;
      }
#endif


#if 0
      M(4,4) = 0.2;
      M(5,5) = 0.2;../04-trivialCtrl/retired.cpp
      M(6,6) = 0.1;
#else
      arr MDiag = diag(ARR(0.4, 0.3, 0.3, 0.4, 0.4, 0.4, 0.2));
      M = M + MDiag;
#endif

      Kp_ref = M*Kp_ref;
      Kd_ref = M*Kd_ref;
      qDDot_ref = M*qDDot_ref;

      u = qDDot_ref - Kp_ref*q_real - Kd_ref*qDot_real;

      //u(5) *= 2.0;

      //cout << u << endl;

      //u *= 0.0; // useful for testing new stuff without braking the robot
    }

    //-- filter torques
    if(u.N==lastTorque.N){
//      double alpha=.5;
//      u = alpha*u + (1.-alpha)*lastTorque;
      lastTorque = u;
    }

    //-- data log?
    if(writeData>0 && !(steps%1)){
      if(!dataFile.is_open()) dataFile.open(STRING("z.panda"<<robotID <<".dat"));
      dataFile <<ctrlTime <<' '; //single number
      q_real.writeRaw(dataFile); //7
      q_ref.writeRaw(dataFile); //7
      if(writeData>1){
        qDot_real.writeRaw(dataFile); //7
        qDot_ref.writeRaw(dataFile); //7
        u.writeRaw(dataFile); //7
        torques_real.writeRaw(dataFile); //7
        G_org.writeRaw(dataFile); //7-vector gravity
        C_org.writeRaw(dataFile); //7-vector coriolis
      }
      if(writeData>2){
        M_org.write(dataFile, " ", " ", "  "); //7x7 inertia matrix
        qDDot_ref.writeRaw(dataFile);
        fric.writeRaw(dataFile); // 7-firction vector
        residual_dyna.writeRaw(dataFile); // 7-residual vector
      }
      dataFile <<endl;
    }

    if (fric_type==2){
        last_command = u + G_org;

        u += fric;
    }

    //-- send torques
    std::array<double, 7> u_array = {0., 0., 0., 0., 0., 0., 0.};
    std::copy(u.begin(), u.end(), u_array.begin());
    if(stop) return franka::MotionFinished(franka::Torques(u_array));
    return franka::Torques(u_array);
  };

  // start real-time control loop
  //cout <<"HERE" <<endl;

  try {
    robot.control(torque_control_callback, true, 2000.);
  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
  }
  LOG(0) <<"EXIT FRANKA CONTROL LOOP";
}
