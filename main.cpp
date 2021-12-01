#include <iostream>
#include <BotOp/bot.h>

using namespace std;

int main()
{
    rai::Configuration C("../../botop/rai-robotModels/scenarios/pandasTable.g");
    arr A = {1.,2,3,4};
    A.reshape(4);
    A = diag(A);
    arr A1 = A(2,{1,3});
    arr A2 = ~A;
    arr A3 = 1/A;
    arr B = {1.,1.2,1.3,1.5};
    arr E = (A,B);
    arr D = E % E;
    cout << D << endl;

    return 0;
}
