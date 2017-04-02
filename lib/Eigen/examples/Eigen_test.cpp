#include <Eigen.h>

using namespace Eigen;

MatrixXd I3(3,3) = MatrixXd::Identity(3,3);
VectorXd t(3), res(3);

void setup()
{
Serial.begin(115200);

t << 1., 2.3, 4.5;

res = I3*t;

Serial.println(t(0),4);
Serial.println(t(1),4);
serial.println(t(2),4);

}

void loop()
{
}
