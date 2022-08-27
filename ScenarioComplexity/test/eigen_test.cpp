#include <unsupported/Eigen/MatrixFunctions>
#include <iostream>

int main(){
    const double pi = std::acos(-1.0);
    Eigen::MatrixXd A(3,3);
    A<< 0, -pi/4, 0,
        pi/4, 0,  0,
        0,  0,    0;
    std::cout << "The Matrix exponential of A is :\n" << A.exp() << "\n\n" << std::endl;

    Eigen::MatrixXd B(3,3);
    B << 0.4, 0, 0,
         0.5, 0, 0,
         0.6, 0, 0;
    B = B * 3;
    std::cout << "The Matrix exponential of B is :\n" << B.exp() << "\n\n" << std::endl;

}