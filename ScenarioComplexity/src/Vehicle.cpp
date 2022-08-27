#include "Vehicle.h"
#include "math/polygon.h"

#include <algorithm>
#include <map>
#include <stdlib.h>

using namespace ScenarioComplexity;

void Vehicle::MonteCarlo(const double& dt, const double& sample){
    // random sample, sample times 100
    if (_future_status.size() <= 1){
        Status init_status = *_status;
        Point init_point(_status->posx, _status->posy);
        std::vector<std::shared_ptr<Status>> status;
        std::vector<std::shared_ptr<Point>> points;
        for (int i = 0; i < 1000; i++){

            auto new_status = applyAkermannControl(dt, init_status, sample);
            while (new_status->vx < 0){
                new_status = applyAkermannControl(dt, init_status, sample);
            }
            // while (new_status->yaw > 0.25 || new_status ->yaw < -0.25){
            //     new_status = applyAkermannControl(dt, init_status);
            // }
            auto new_point = std::make_shared<Point>(new_status->posx, new_status->posy);            
            
            status.emplace_back(new_status);
            // showStatusInfo(new_status);


            points.emplace_back(new_point);
        }
        // find boundary point
        auto bp = Polygon::findBoundaryPoint(points, init_point);

        // find convex part
        _future_points.emplace_back(std::make_pair(bp.first.second, bp.second.second));

        // store the status
        _future_status.emplace_back(
            status[bp.first.first], status[bp.second.first]);
        // showStatusInfo();
    } 
    else {
        // already compute one round 

        std::shared_ptr<Point> PUpper, PLower;
        std::shared_ptr<Status> SUpper, SLower;

        // start with upper
        auto init_status = _future_status.back().first;
        Point init_point(init_status->posx, init_status->posy);
        std::vector<std::shared_ptr<Status>> status;
        std::vector<std::shared_ptr<Point>> points;
        for (int i = 0; i < 1000; i++){

            auto new_status = applyAkermannControl(dt, *init_status, sample);
            while (new_status->vx < 0){
                new_status = applyAkermannControl(dt, *init_status, sample);
            }
            // while (new_status->yaw > 0.25 || new_status ->yaw < -0.25){
            //     new_status = applyAkermannControl(dt, *init_status);
            // }
            auto new_point = std::make_shared<Point>(new_status->posx, new_status->posy);            

            
            status.emplace_back(new_status);
            points.emplace_back(new_point);
        }
        // find boundary point
        auto bp = Polygon::findBoundaryPoint(points, init_point);

        // find convex
        PUpper = bp.first.second;
        SUpper = status[bp.first.first];

        //start with lower
        init_status = _future_status.back().second;
        init_point = Point(init_status->posx, init_status->posy);
        status.clear();
        points.clear();
        for (int i = 0; i < 1000; i++){
            // if (init_status.ax > 7 || init_status.ax < -7){
            //     exit(1);
            // }

            auto new_status = applyAkermannControl(dt, *init_status, sample);
            while (new_status->vx < 0){
                new_status = applyAkermannControl(dt, *init_status, sample);
            }
            // while (new_status->yaw > 0.25 || new_status ->yaw < -0.25){
            //     new_status = applyAkermannControl(dt, *init_status);
            // }
            auto new_point = std::make_shared<Point>(new_status->posx, new_status->posy);            

            
            status.emplace_back(new_status);
            points.emplace_back(new_point);
        }
        // find boundary point
        bp = Polygon::findBoundaryPoint(points, init_point);

        // find convex
        PLower = bp.second.second;
        SLower = status[bp.second.first];

        // find convex part
        _future_points.emplace_back(std::make_pair(PUpper, PLower));

        //store the status
        _future_status.emplace_back(
            std::make_pair(SUpper, SLower)
        );

        // showStatusInfo();
        // showStatusInfo(_future_status.back().first);
        // showStatusInfo(_future_status.back().second);
    }
}

void Vehicle::generateReachableConvex(const double& dt, const double& sample=1){
    // apply extreme motecarlo sample
    if (_reachable_status.size() < 1){
        Status init_status = *_status;
        Point init_point(_status->posx, _status->posy);
        std::vector<std::shared_ptr<Status>> status;
        std::vector<std::shared_ptr<Point>> points;
        for (int i = 0; i < 100; i++){

            auto new_status = applyAkermannControl(dt, init_status, sample);
            while (new_status->vx < 0){
                new_status = applyAkermannControl(dt, init_status, sample);
            }
            // while (new_status->yaw > 0.25 || new_status ->yaw < -0.25){
            //     new_status = applyAkermannControl(dt, init_status);
            // }
            auto new_point = std::make_shared<Point>(new_status->posx, new_status->posy);            
            
            status.emplace_back(new_status);
            // showStatusInfo(new_status);


            points.emplace_back(new_point);
        }
        // find boundary point
        auto bp = Polygon::findBoundaryPoint(points, init_point);

        // find convex part
        _reachable_convex.emplace_back(std::make_pair(bp.first.second, bp.second.second));

        // store the status
        _reachable_status.emplace_back(
            status[bp.first.first], status[bp.second.first]);
        showStatusInfo(_reachable_status.back().first);
    } 
    else {
        // already compute one round 

        std::cout << "already" << std::endl;

        std::shared_ptr<Point> PUpper, PLower;
        std::shared_ptr<Status> SUpper, SLower;

        // start with upper
        auto init_status = _reachable_status.back().first;
        Point init_point(init_status->posx, init_status->posy);
        std::vector<std::shared_ptr<Status>> status;
        std::vector<std::shared_ptr<Point>> points;
        for (int i = 0; i < 100; i++){

            auto new_status = applyAkermannControl(dt, *init_status, sample);
            while (new_status->vx < 0){
                new_status = applyAkermannControl(dt, *init_status, sample);
            }
            // while (new_status->yaw > 0.25 || new_status ->yaw < -0.25){
            //     new_status = applyAkermannControl(dt, *init_status);
            // }
            auto new_point = std::make_shared<Point>(new_status->posx, new_status->posy);            

            
            status.emplace_back(new_status);
            points.emplace_back(new_point);
        }
        // find boundary point
        auto bp = Polygon::findBoundaryPoint(points, init_point);

        // find convex
        PUpper = bp.first.second;
        SUpper = status[bp.first.first];

        //start with lower
        init_status = _reachable_status.back().second;
        init_point = Point(init_status->posx, init_status->posy);
        status.clear();
        points.clear();
        for (int i = 0; i < 100; i++){
            // if (init_status.ax > 7 || init_status.ax < -7){
            //     exit(1);
            // }

            auto new_status = applyAkermannControl(dt, *init_status, sample);
            while (new_status->vx < 0){
                new_status = applyAkermannControl(dt, *init_status, sample);
            }
            // while (new_status->yaw > 0.25 || new_status ->yaw < -0.25){
            //     new_status = applyAkermannControl(dt, *init_status);
            // }
            auto new_point = std::make_shared<Point>(new_status->posx, new_status->posy);            

            
            status.emplace_back(new_status);
            points.emplace_back(new_point);
        }
        // find boundary point
        bp = Polygon::findBoundaryPoint(points, init_point);

        // find convex
        PLower = bp.second.second;
        SLower = status[bp.second.first];

        // find convex part
        _reachable_convex.emplace_back(std::make_pair(PUpper, PLower));

        //store the status
        _reachable_status.emplace_back(
            std::make_pair(SUpper, SLower)
        );
        showStatusInfo(_reachable_status.back().first);
    }

}

void Vehicle::generateCenterConvex(const double& dt, const double& sample=0){
    // apply extreme motecarlo sample
    if (_center_status.size() <= 1){
        Status init_status = *_status;
        Point init_point(_status->posx, _status->posy);
        std::vector<std::shared_ptr<Status>> status;
        std::vector<std::shared_ptr<Point>> points;
        for (int i = 0; i < 100; i++){

            auto new_status = applyAkermannControl(dt, init_status, sample);
            while (new_status->vx < 0){
                new_status = applyAkermannControl(dt, init_status, sample);
            }
            // while (new_status->yaw > 0.25 || new_status ->yaw < -0.25){
            //     new_status = applyAkermannControl(dt, init_status);
            // }
            auto new_point = std::make_shared<Point>(new_status->posx, new_status->posy);            
            
            status.emplace_back(new_status);
            // showStatusInfo(new_status);


            points.emplace_back(new_point);
        }
        // find boundary point
        auto bp = Polygon::findBoundaryPoint(points, init_point);

        // find convex part
        _center_convex.emplace_back(std::make_pair(bp.first.second, bp.second.second));

        // store the status
        _center_status.emplace_back(
            status[bp.first.first], status[bp.second.first]);
        // showStatusInfo();
    } 
    else {
        // already compute one round 

        std::shared_ptr<Point> PUpper, PLower;
        std::shared_ptr<Status> SUpper, SLower;

        // start with upper
        auto init_status = _center_status.back().first;
        Point init_point(init_status->posx, init_status->posy);
        std::vector<std::shared_ptr<Status>> status;
        std::vector<std::shared_ptr<Point>> points;
        for (int i = 0; i < 100; i++){

            auto new_status = applyAkermannControl(dt, *init_status, sample);
            while (new_status->vx < 0){
                new_status = applyAkermannControl(dt, *init_status, sample);
            }
            // while (new_status->yaw > 0.25 || new_status ->yaw < -0.25){
            //     new_status = applyAkermannControl(dt, *init_status);
            // }
            auto new_point = std::make_shared<Point>(new_status->posx, new_status->posy);            

            
            status.emplace_back(new_status);
            points.emplace_back(new_point);
        }
        // find boundary point
        auto bp = Polygon::findBoundaryPoint(points, init_point);

        // find convex
        PUpper = bp.first.second;
        SUpper = status[bp.first.first];

        //start with lower
        init_status = _center_status.back().second;
        init_point = Point(init_status->posx, init_status->posy);
        status.clear();
        points.clear();
        for (int i = 0; i < 100; i++){
            // if (init_status.ax > 7 || init_status.ax < -7){
            //     exit(1);
            // }

            auto new_status = applyAkermannControl(dt, *init_status, sample);
            while (new_status->vx < 0){
                new_status = applyAkermannControl(dt, *init_status, sample);
            }
            // while (new_status->yaw > 0.25 || new_status ->yaw < -0.25){
            //     new_status = applyAkermannControl(dt, *init_status);
            // }
            auto new_point = std::make_shared<Point>(new_status->posx, new_status->posy);            

            
            status.emplace_back(new_status);
            points.emplace_back(new_point);
        }
        // find boundary point
        bp = Polygon::findBoundaryPoint(points, init_point);

        // find convex
        PLower = bp.second.second;
        SLower = status[bp.second.first];

        // find convex part
        _center_convex.emplace_back(std::make_pair(PUpper, PLower));

        //store the status
        _center_status.emplace_back(
            std::make_pair(SUpper, SLower)
        );

        // showStatusInfo();
        // showStatusInfo(_future_status.back().first);
        // showStatusInfo(_future_status.back().second);
    }

}

std::shared_ptr<Status> Vehicle::applyAkermannControl(const double& dt, const Status& init_status, const double& sample){
    _control.generateControlSignal(init_status, sample);
    _control.delta_a = std::clamp(_control.delta_a, -_physics.max_acc, _physics.max_acc);
    _control.delta_f = std::clamp(_control.delta_f, -_physics.max_steer, _physics.max_steer);

    
    return ScenarioComplexity::applyAkermannControl(dt, init_status, _control, _shape);
}

std::shared_ptr<Status> Vehicle::applyAkermannControl(const double& dt, const Status& init_status, const ControlSignal& control){

    double beta = atan(0.5 * tan(control.delta_f));
    double yaw = init_status.yaw + sqrt(pow(init_status.vx, 2) + pow(init_status.vy, 2)) / (_shape.length / 2) * dt * sin(beta);
    double v = sqrt(pow(init_status.vx, 2) + pow(init_status.vy, 2)) + control.delta_a * dt;
    double posx = init_status.posx + v * cos(beta + yaw) * dt;
    double posy = init_status.posy + v * sin(beta + yaw) * dt;

    auto new_status = std::make_shared<Status>();
    new_status->posx = posx;
    new_status->posy = posy;
    new_status->beta = beta;
    new_status->yaw = yaw;
    new_status->vx = v * cos(new_status->yaw);
    new_status->vy = v * sin(new_status->yaw);
    
    new_status->ax = control.delta_a * cos(atan(new_status->vy / new_status->vx));
    new_status->ay = control.delta_a * sin(atan(new_status->vy / new_status->vx));

    new_status->steer = control.delta_f;

    
    std::cout << "control" << control.delta_a << " " <<control.delta_f << std::endl;
    showStatusInfo(new_status);

    return new_status;
    // double beta = atan(0.5 * tan(control.delta_f));
    // double yaw = init_status.yaw + sqrt(pow(init_status.vx, 2) + pow(init_status.vy, 2)) * atan(init_status.vy / init_status.vx) / (_shape.length / 2) * dt *beta;
    // double v = sqrt(pow(init_status.vx, 2) + pow(init_status.vy, 2)) + control.delta_a * dt;
    

    // auto new_status = std::make_shared<Status>();
    // new_status->ax = control.delta_a * cos(atan(init_status.vy / init_status.vx));
    // new_status->ay = control.delta_a * sin(atan(init_status.vy / init_status.vx));
    // new_status->beta = beta;
    // new_status->yaw = yaw;
    // new_status->vx = v * cos(new_status->yaw + new_status->beta);
    // new_status->vy = v * sin(new_status->yaw + new_status->beta);
    // new_status->posx = init_status.posx + new_status->vx * dt;
    // new_status->posy = init_status.posy + new_status->vy * dt;
    
    // new_status->steer = control.delta_f;
}

std::vector<std::pair<int, std::shared_ptr<Point>>> Vehicle::applyExtremeAkermannControl(const double& dt, const Status& status, const Point& init_point){
    // By default the interval of acc is 0.5 ms-2
    // By default the interval of steer is 0.1 rad
    _control.delta_a = _physics.max_acc;
    _control.delta_f = -_physics.max_steer;
    while(_control.delta_f < _physics.max_steer){
        auto new_status = applyAkermannControl(dt, status, _control);
        if (new_status->vx < 0){
            // vehicle do not drive reverse
            _control.delta_f += 0.01;
            break;
        }
        _extreme_status.emplace_back(new_status);
        auto new_point = std::make_shared<Point>(new_status->posx, new_status->posy);
        _extreme_points.emplace_back(new_point);
        _control.delta_f += 0.01;
    }
    _convex.emplace_back(_extreme_points);
    // the extreme akermann circle is generated
    return Polygon::findConvex(_extreme_points, init_point);
}

void Vehicle::generateReachableConvex(const double& dt){
    if (_reachable_convex.size() == 0){
        _extreme_status.clear();
        _extreme_points.clear();
        auto convex_points = applyExtremeAkermannControl(dt, *_status, _point);

        _reachable_convex.emplace_back(
            std::make_pair(convex_points.back().second, convex_points.front().second)
        );
        _reachable_status.emplace_back(
            std::make_pair(_extreme_status[convex_points.back().first],
                           _extreme_status[convex_points.front().first])
        );
    }
    else{
        // already run a round
        std::cout << "already" << std::endl;
        _extreme_status.clear();
        _extreme_points.clear();
        auto upper_convex_points = applyExtremeAkermannControl(dt, *_reachable_status.back().first, *_reachable_convex.back().first);
        auto upper_convex_point = upper_convex_points.back().second;
        auto upper_convex_status = _extreme_status[upper_convex_points.back().first];
        _extreme_status.clear();
        _extreme_points.clear();
        auto lower_convex_points = applyExtremeAkermannControl(dt, *_reachable_status.back().second, *_reachable_convex.back().second);
        auto lower_convex_point = lower_convex_points.front().second;
        auto lower_convex_status = _extreme_status[lower_convex_points.front().first];
        _convex.emplace_back(_extreme_points);
        std::cout << "\n\n" << std::endl;

        _reachable_convex.emplace_back(
            std::make_pair(upper_convex_point, lower_convex_point)
        );
        _reachable_status.emplace_back(
            std::make_pair(upper_convex_status, lower_convex_status)
        );
    }
}

void Vehicle::findLatticeTrajectory(){
    for(int i=0; i < _reachable_convex.size(); i++){

    }
}

std::vector<std::pair<std::shared_ptr<Point>, std::shared_ptr<Point>>> Vehicle::getReachableConvex(const bool& is_wobbing=false){
    if (!is_wobbing){
        return _reachable_convex;
    }
}

void Vehicle::solve(){
    _solver.solve();
}



void Vehicle::showStatusInfo(){

    auto last_status = _future_status.back();
    if (last_status.first->posx == 0){
        std::cout << "nullptr found" << std::endl;
    }
    
    std::cout << "upper status\n" 
              << "posx " << last_status.first->posx
              << "posy " << last_status.first->posy
              << "ax " << last_status.first->ax
              << "ay " << last_status.first->ay
              << "vx " << last_status.first->vx
              << "vy " << last_status.first->vy
              << "yaw " << last_status.first->yaw << std::endl;
    std::cout << "\n" << std::endl;
    std::cout << "lower status\n" 
              << "posx " << last_status.second->posx
              << "posy " << last_status.second->posy
              << "ax " << last_status.second->ax
              << "ay " << last_status.second->ay
              << "vx " << last_status.second->vx
              << "vy " << last_status.second->vy
              << "yaw " << last_status.second->yaw << std::endl;

}

void Vehicle::showStatusInfo(const std::shared_ptr<Status>& status){
    std::cout << "status\n" 
              << "posx " << status->posx
              << "posy " << status->posy
              << "ax " << status->ax
              << "ay " << status->ay
              << "vx " << status->vx
              << "vy " << status->vy
              << "yaw " << status->yaw
              << "beta " << status->beta << std::endl;

}
