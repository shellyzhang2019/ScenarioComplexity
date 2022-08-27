#include "Control.h"
#include <memory>

using namespace ScenarioComplexity;
using CppAD::AD;
using CppAD::atan;

/**
 * @brief Basic Akermann Control
 * 
 * @param dt delta_t
 * @param init_status current frame init status 
 * @param control control signal, using [a, s] as throttle and front_steer
 * @param shape vehicle physics
 * @return std::shared_ptr<Status> 
 */
std::shared_ptr<Status> applyAkermannControl(const double& dt,
                                             const Status& init_status, 
                                             const ControlSignal& control,
                                             const Shape& shape){
    double beta = atan(0.5 * tan(control.delta_f));
    double yaw = init_status.yaw + sqrt(pow(init_status.vx, 2) + pow(init_status.vy, 2)) / (shape.length / 2) * dt * sin(beta);
    double v = sqrt(pow(init_status.vx, 2) + pow(init_status.vy, 2)) + control.delta_a * dt;
    double posx = init_status.posx + v * cos(beta + yaw) * dt;
    double posy = init_status.posy + v * sin(beta + yaw) * dt;

    auto new_status = std::make_shared<Status>();
    new_status->posx = posx;
    new_status->posy = posy;
    new_status->beta = beta;
    new_status->yaw = yaw;
    new_status->vx = v * cos(new_status->yaw + new_status->beta);
    new_status->vy = v * sin(new_status->yaw + new_status->beta);
    
    new_status->ax = control.delta_a * cos(atan(new_status->vy / new_status->vx));
    new_status->ay = control.delta_a * sin(atan(new_status->vy / new_status->vx));

    new_status->steer = control.delta_f;

    return new_status;
}

/**
 * @brief Basic AD Akermann Control Model
 * 
 * @param dt delta_t
 * @param init_status current init status 
 * @param control AD vector, [a, f]
 * @param shape 
 * @return ADStatus 
 */
ADStatus applyADAkermannControl(const AD<double>& dt,
                                const ADStatus& init_status, 
                                const std::vector<AD<double>>& control,
                                const ADShape& shape){
    AD<double> beta = atan(0.5 * control[1].tan_me());
    AD<double> yaw = init_status.yaw + sqrt(pow(init_status.vx, 2) + pow(init_status.vy, 2)) / (shape.length / 2) * dt * sin(beta);
    AD<double> v = sqrt(pow(init_status.vx, 2) + pow(init_status.vy, 2)) + control[0] * dt;
    AD<double> posx = init_status.posx + v * cos(beta + yaw) * dt;
    AD<double> posy = init_status.posy + v * sin(beta + yaw) * dt;

    ADStatus new_status = ADStatus();
    new_status.posx = posx;
    new_status.posy = posy;
    new_status.beta = beta;
    new_status.yaw = yaw;
    new_status.vx = v * cos(new_status.yaw + new_status.beta);
    new_status.vy = v * sin(new_status.yaw + new_status.beta);
    
    new_status.ax = control[0] * cos(atan(new_status.vy / new_status.vx));
    new_status.ay = control[0] * sin(atan(new_status.vy / new_status.vx));

    new_status.steer = control[0];

    return new_status;
}