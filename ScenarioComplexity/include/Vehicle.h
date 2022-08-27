#include <memory>
#include <stdlib.h>

#include "Geo.h"
#include "math/solver.h"

namespace ScenarioComplexity{

class Vehicle{

    public:

    Vehicle(){};
    Vehicle(const double& x,
            const double& y,
            const double& ax,
            const double& ay,
            const double& vx,
            const double& vy,
            const double& yaw,
            const double& steer,
            const double& length,
            const double& width,
            const std::string& type){
                std::cout << "construct";
                _status = std::make_shared<Status>();
                // _point = std::make_unique<Point>(x, y);
                _status->posx = x;
                _status->posy = y;
                _status->ax = ax;
                _status->ay = ay;
                _status->vx = vx;
                _status->vy = vy;
                _status->yaw = yaw;
                _status->beta = atan(_status->vy / (_status->vx + 0.001)) - _status->yaw;
                _status->steer = steer;
                _shape.length = length;
                _shape.width = width;
                _type = type;
                if (_type.find("truck") != std::string::npos){
                    _physics.max_acc = 5.0;
                    _physics.max_steer = 10;
                }
                else if (_type.find("car") != std::string::npos){
                    _physics.max_acc = 6.0;
                    _physics.max_steer = 15 * 3.14 / 180;
                }
                _future_status.emplace_back(std::make_pair(_status, _status));
                _solver = Solver(0.1, 20, LaneInfo(), *_status, _shape, _physics, uu);
                std::cout << "ok";
            }
    ~Vehicle(){}

    void MonteCarlo(const double&, const double&); //  state update on every frame

    void createVehicleBox(); // first get position then rotate, called on every frame

    void collisionDecection(const Vehicle&); // collision detection

    void showStatusInfo();

    void findLatticeTrajectory();

    void showStatusInfo(const std::shared_ptr<Status>&);

    void generateReachableConvex(const double&);

    void generateReachableConvex(const double&, const double&);

    void generateCenterConvex(const double&, const double&);

    void solve();

    std::vector<std::pair<std::shared_ptr<Point>, std::shared_ptr<Point>>> getReachableConvex(const bool&);

    std::shared_ptr<Status> applyAkermannControl(const double&, const Status&, const double&);

    std::shared_ptr<Status> applyAkermannControl(const double&, const Status&, const ControlSignal&);

    std::vector<std::pair<int, std::shared_ptr<Point>>> applyExtremeAkermannControl(const double&, const Status&, const Point&);

    const Status getStatus(){
        return *_status;
    }

    const Shape getShape(){
        return _shape;
    }

    const std::vector<std::pair<std::shared_ptr<Point>, std::shared_ptr<Point>>> getFuturePoints(){
        return _future_points;
    }

    const std::vector<std::vector<std::shared_ptr<Point>>> getConvex(){
        return _convex;
    }

    const std::vector<std::pair<std::shared_ptr<Point>, std::shared_ptr<Point>>> getCenterConvex(){
        return _center_convex;
    }

    private:
    std::shared_ptr<Status> _status;
    Point _point;
    Shape _shape;
    std::string _type;
    Physics _physics;
    ControlSignal _control;
    Solver _solver;
    std::vector<std::pair<std::shared_ptr<Point>, std::shared_ptr<Point>>> _reachable_convex;
    std::vector<std::pair<std::shared_ptr<Status>, std::shared_ptr<Status>>> _reachable_status;
    std::vector<std::pair<std::shared_ptr<Point>, std::shared_ptr<Point>>> _center_convex;
    std::vector<std::pair<std::shared_ptr<Status>, std::shared_ptr<Status>>> _center_status;
    std::vector<std::pair<std::shared_ptr<Point>, std::shared_ptr<Point>>> _future_points;
    std::vector<std::pair<std::shared_ptr<Status>, std::shared_ptr<Status>>> _future_status;
    std::vector<std::shared_ptr<Status>> _extreme_status;
    std::vector<std::shared_ptr<Point>> _extreme_points;
    std::vector<Trajectory> _lattice_trajectory;
    
    std::vector<std::vector<std::shared_ptr<Point>>> _convex;


};

}