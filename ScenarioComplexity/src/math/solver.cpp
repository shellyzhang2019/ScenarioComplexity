#include "math/solver.h"
#include "Control.h"

using namespace ScenarioComplexity;

Solver::Solver(const size_t& dt,
               const size_t& time_window,
               const LaneInfo& lane_info,
               const Status& status,
               const Shape& shape,
               const Physics& physics,
               const Gtype& type):
    _time_window(time_window),
    _lane_info(lane_info),
    _init_status(status),
    _shape(shape),
    _physics(physics){
        auto fg_eval_status = ADStatus(status);
        auto fg_eval_shape = ADShape(shape);
        _fg_eval = FG_eval(dt, fg_eval_status, fg_eval_shape, type);
    }

bool Solver::solve(){
    bool ok = true;
    typedef CPPAD_TESTVECTOR(double) Dvector;
    size_t nx = static_cast<int>(_time_window * 2);
    size_t ng = _time_window + 2;
    Dvector x(nx);
    for(size_t i=0; i< nx; i++){
        x[i] = 0;
    }

    // lower and upper limits for x
    Dvector xl(nx), xu(nx);
    for (size_t i=0; i < nx; i+=2){
        xl[i] = -_physics.max_acc;
        xl[i+1] = -_physics.max_steer;
        xu[i] = _physics.max_acc;
        xu[i+1] = _physics.max_steer;
    }

    //lower and upper limits for g
    Dvector gl(ng), gu(ng);
    for (size_t i=0; i < _time_window; i +=1){
        gl[i] = 0; gu[i] = 1.0e19;
    }

    // only for test
    gl[_time_window] = 3.5; gu[_time_window] = 3.5;
    gl[_time_window + 1] = 0; gu[_time_window + 1] = 0;

    //options
    std::string options;
    options += "Integer print_level  0\n";
    options += "String  sb           yes\n";
    // maximum number of iterations
    options += "Integer max_iter     10\n";
    // approximate accuracy in first order necessary conditions;
    // see Mathematical Programming, Volume 106, Number 1,
    // Pages 25-57, Equation (6)
    options += "Numeric tol          1e-6\n";
    // derivative testing
    options += "String  derivative_test            second-order\n";
    // maximum amount of random pertubation; e.g.,
    // when evaluation finite diff
    options += "Numeric point_perturbation_radius  0.\n";

    CppAD::ipopt::solve<Dvector, FG_eval>(
        options, x, xl, xu, gl, gu, _fg_eval, _solution
    );

    std::cout << "solution: " << _solution.x << std::endl;


    ok &= _solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    return ok;

}