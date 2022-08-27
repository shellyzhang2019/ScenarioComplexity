/**
 * @file solver.h
 * @author your name (you@domain.com)
 * @brief This module computes the drivable area using akkermman dynamics model
 * @version 0.1
 * @date 2022-08-26
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "LaneInfo.h"
#include "Control.h"
#include <memory>

namespace ScenarioComplexity{

enum Gtype {lu, ll, ul, uu};

using CppAD::AD;

typedef CPPAD_TESTVECTOR(double) Dvector;


class FG_eval{

public:
    FG_eval(){}
    FG_eval(const double& dt, const ADStatus& adstatus, const ADShape& adshape, const Gtype& type):
    _dt(dt), _adshape(adshape), _type(type) {
        _adstatus.emplace_back(adstatus);
    } 

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    void operator()(ADvector& fg, const ADvector& x){
        // note that contraints ...
        int n = static_cast<int>(x.size() / 2);
        assert(fg.size()==n+3);

        // Fortran style indexing
        int m = 1;
        for (int i=0; i < x.size(); i+=2){
            std::vector<AD<double>> control(2);
            control[0] = x[i];
            control[1] = x[i+1];
            auto new_status = applyADAkermannControl(
                _dt, _adstatus.back(), control, _adshape
            );
            fg[m] = new_status.vx;
            _adstatus.emplace_back(new_status);
            m +=1;
        }
        switch (_type)
        {
        case ll:
            fg[0] = _adstatus.back().posx;
            break;
        
        case lu:
            fg[0] = _adstatus.back().posx;
            break;
        case ul:
            fg[0] = -_adstatus.back().posx;
            break;
        case uu:
            fg[0] = -_adstatus.back().posx;
            break;
        }
        fg[m] = _adstatus.back().posy;
        fg[m+1] = _adstatus.back().yaw; 
    }

private:
    AD<double> _dt;
    std::vector<ADStatus> _adstatus;
    ADShape _adshape;
    Gtype _type;
};

class Solver{

    public:
    Solver(){}
    Solver(const size_t&, const size_t&, const LaneInfo&, const Status&, const Shape&, const Physics&, const Gtype&);

    bool solve();


    private:
    FG_eval _fg_eval;
    int _time_window;
    LaneInfo _lane_info;
    Status _init_status;
    Shape _shape;
    Physics _physics;
    CppAD::ipopt::solve_result<Dvector> _solution;
};


}

