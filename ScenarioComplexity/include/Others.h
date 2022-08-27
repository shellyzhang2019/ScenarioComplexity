#pragma once
#include <chrono>
#include <iostream>
#include <random>

#define HAVE_CSTDDEF
#include <cppad/ipopt/solve.hpp>
#undef HAVE_CSTDDEF

using CppAD::AD;

namespace ScenarioComplexity
{
struct Status{

    double posx;
    double posy;
    double ax; // ms-2
    double ay;
    double vx; // ms-1
    double vy;
    double yaw; // rad
    double steer; //front wheel
    double beta;
    // bool wob = false;
};

struct ADStatus{

    ADStatus(){}
    ADStatus(const Status& status){
        posx = status.posx;
        posy = status.posy;
        ax = status.ax;
        ay = status.ay;
        vx = status.vx;
        vy = status.vy;
        yaw = status.yaw;
        steer = status.steer;
        beta = status.beta;
    }

    AD<double> posx;
    AD<double> posy;
    AD<double> ax; // ms-2
    AD<double> ay;
    AD<double> vx; // ms-1
    AD<double> vy;
    AD<double> yaw; // rad
    AD<double> steer; //front wheel
    AD<double> beta;
};

struct Shape{
    double length;
    double width;
};

struct ADShape{

    ADShape(){}
    ADShape(const Shape& shape){
        length = shape.length;
        width = shape.width;
    }

    AD<double> length;
    AD<double> width;
};

struct Physics{
    double max_steer; // rad
    double max_acc;
};

struct ADPhysics{
    AD<double> max_steer; // rad
    AD<double> max_acc;
};

class ControlSignal{
    public:
    double delta_f;
    double delta_a;

    void generateControlSignal(const Status& status, const double& sample_varience){
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine gen(seed);
        std::normal_distribution<double> acc(sqrt(status.ax*status.ax+status.ay*status.ay), sample_varience);
        std::normal_distribution<double> steer(0, sample_varience); 

        delta_f = steer(gen);
        delta_a = acc(gen);  
    }
};
} // namespace ScenarioComplexity
