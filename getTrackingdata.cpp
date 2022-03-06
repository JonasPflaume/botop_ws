#include <KOMO/pathTools.h>

#include <BotOp/bot.h>
#include <BotOp/motionHelpers.h>
#include <math.h>
//===========================================================================
void collectTrackingData(){
    rai::Configuration C("../../rai-robotModels/scenarios/pandasTable.g");
    arr q0 = C.getJointState();
    arr start_point;
    FILE("q0.txt") >> start_point;
    q0({0,6}) = start_point;
    q0.reshape(1,14);
    arr times;
    FILE("t.txt") >> times;
    times += 1.5;
    times *= 2.;
    arr dq;
    FILE("dq.txt") >> dq;
    arr path = q0;

    for (uint i = 0; i<11000; i++){
        arr dq_i = dq(i, {0,-1}) * 0.001;
        arr dq_temp = zeros(14);
        dq_temp({0,6}) = dq_i;
        dq_temp.reshape(1,14);
        q0 += dq_temp;
        path.append(q0);
    }
    LOG(0) << path.dim() << times.dim();
    BotOp bot(C, rai::checkParameter<bool>("false")); //false
    bot.robotL->writeData=2;
    bot.move(path, times);
    while(bot.step(C)){}
    bot.robotL->writeData=0;
    bot.home(C);
    rai::wait();
}

void collectJointData(){
  rai::Configuration C("../../rai-robotModels/scenarios/pandasTable.g");
  arr qHome = C.getJointState();
  arr qLimits = C.getLimits();

  uint jointID = rai::getParameter<int>("joint", 6);
  CHECK_LE(jointID, qHome.N, "");


  BotOp bot(C, rai::checkParameter<bool>("real")); //false
  bot.home(C);

  double q0 = qHome(jointID);
  double lo = qLimits(jointID, 0)+.1;
  double up = qLimits(jointID, 1)-.1;
  if(jointID==1) up -= 1.2;

  uint k=30;
  arr path = ~qHome;
  arr times = ARR(0.);
  double q = lo;
  double v = 0.;
  double t = 10.;
  double vstep = .005;

  for(uint s=0;;s++){
    for(uint j=0;j<k;j++){
      q += .1*v;
      t += .1;
      if(q>up) break;
      path.append(qHome);  path(-1, jointID) = q;
      times.append(t);
    }
    if(q>up) break;
    v += vstep;
  }
  q = up;
  v = 0.;
  for(uint s=0;;s++){
    for(uint j=0;j<k;j++){
      q += .1*v;
      t += .1;
      if(q<lo) break;
      path.append(qHome);  path(-1, jointID) = q;
      times.append(t);
    }
    if(q<lo) break;
    v -= vstep;
  }

#if 0
  for(uint s=0;;s++){
    q += .01 * s;
    t += 1.;
    if(q>up) break;
    for(uint j=0;j<k;j++){
      path.append(qHome);
      path(-1, jointID) = q;
      times.append(t);
      t += .2;
    }
  }
  q = up;
  for(uint s=0;;s++){
    q -= .01 * s;
    t += 1.;
    if(q<lo) break;
    for(uint j=0;j<k;j++){
      path.append(qHome);
      path(-1, jointID) = q;
      times.append(t);
      t += .2;
    }
  }
#endif

  path.append(qHome);
  times.append(t+10.);
  times *= .5;

  rai::wait();

  bot.robotL->writeData=2;
  LOG(0) << path.dim() << times.dim();
  bot.move(path, times);
  while(bot.step(C)){}
  bot.robotL->writeData=0;


  rai::wait();
}

void testJacobian(){
    rai::Configuration C("../../rai-robotModels/scenarios/pandasTable.g");
    BotOp bot(C, rai::checkParameter<bool>("false")); //false
    rai::Frame *gripper = C["l_panda_joint8"];
    // - create trajectory
    arr qHome = C.getJointState();
    arr path = ~qHome;
    arr times = {0.};

    for (uint i=0; i<100000; ++i){
        double t = 0.001 * i;
        times.append(t);
        arr endVec = {0.5*sin(2*t*3.141592/5)*(2*3.141592/5), 0.7*cos(2*t*3.141592/5)*(2*3.141592/5), 0};
        arr J_pos;
        C.jacobian_pos(J_pos, gripper, 0);
        arr q_vec= ~endVec * J_pos;
        qHome += q_vec * 0.001;

        path.append(qHome);
    }
    times += 0.5;

    bot.robotL->writeData=2;
    bot.move(path, times);
    while(bot.step(C)){}
    bot.robotL->writeData=0;
    bot.home(C);
    rai::wait();
}

void postprocessData(){
    rai::Configuration C("../../rai-robotModels/scenarios/pandasTable.g");
    rai::Frame *gripper = C["l_panda_joint8"];

    arr X;
    FILE("z.panda0.dat") >> X;

    ofstream fil2("z.processed.dat");

    for(uint t=0;t<X.d0;t++){
        arr q = X.sub(t,t,1,14);
        q.reshape(-1);
        arr J;
        arr y;

        C.jacobian_pos(J, gripper, 0);
        //C.kinematicsPos(y, J, gripper);
        C.setJointState(q);
        C.watch();

        std::cout<<J.dim() <<std::endl;

        fil2 <<q <<' ' <<J <<endl;
    }

}
//===========================================================================
int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);
  collectTrackingData();
//  collectTrackingData();
  return 0;
}

