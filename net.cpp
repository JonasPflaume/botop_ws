void ReLU(arr &x){
    uint l = x.N;
    for(uint i = 0; i<l; i++){
        if(x(i, 0) < 0.){
            x(i, 0) = 0.;
        }
    }
}

void net_forward(arr &res, const arr &x, const arr &be0, const arr &bi0, const arr &be1, const arr &bi1, const arr &be2, const arr &bi2){
    //x vector
    res = be0 * x;
    res = res + bi0;
    ReLU(res);
    res = be1 * res;
    res = res + bi1;
    ReLU(res);
    res = be2 * res;
    res = res + bi2;
    cout << res;
}
int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);
  //C["l_panda_joint8"]->getPose(); ->getRotationMatrix()
  //C.kinematicsPos(output, NoArr, C["l_panda_joint8"], relVector)
//  postprocessData();
  arr beta0, bias0, beta1, bias1, beta2, bias2;
  FILE("Regression_weight/net.0.weight.txt") >> beta0;
  FILE("Regression_weight/net.0.bias.txt") >> bias0;
  FILE("Regression_weight/net.2.weight.txt") >> beta1;
  FILE("Regression_weight/net.2.bias.txt") >> bias1;
  FILE("Regression_weight/net.4.weight.txt") >> beta2;
  FILE("Regression_weight/net.4.bias.txt") >> bias2;
  arr x = ones(14,1);
  arr res;
  net_forward(res, x, beta0, bias0, beta1, bias1, beta2, bias2);
  return 0;
}
