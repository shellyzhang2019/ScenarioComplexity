#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <vector>


namespace ScenarioComplexity{

namespace Zonotope{

class Zonotope{
    public:
    Zonotope();
    Zonotope(const std::vector<std::pair<float, float>>& points);
    Zonotope(const Eigen::Vector2f& center, const Eigen::MatrixXf& generators);
    ~Zonotope() {}

    void GeneratePoints(){
        points = Eigen::MatrixXf::Zero(_generators.rows() + 1, _generators.cols());
        points(0, 0) = _center(0, 0), points(0, 1) = _center(0, 1);
        for (size_t i = 0; i < _generators.rows(); i++){
            points(i+1, 0) = _center(0, 0) + _generators(i, 0);
            points(i+1, 1) = _center(0, 1) + _generators(i, 1);
        }
    }

    Eigen::MatrixXf points;

    private:
    Eigen::Vector2f _center;
    Eigen::MatrixXf _generators;
    
};

/**
 * @brief Get the Init Zonotope Linear object
 * 
 * @param init_status init vehicle position zonotope
 * @param state_trans system state update transform
 * @param correction noise consideration
 * @param r steps
 * @return const Zonotope& 
 */
const Zonotope& GetInitZonotopeLinearWithoutNoise(Zonotope& init_status,
                                const Eigen::MatrixXf& state_trans,
                                const Eigen::MatrixXf& correction,
                                const int& r,
                                const float& alpha=0.5){
    
    Zonotope* r_zonotope = new Zonotope();
    
    Eigen::MatrixXf me = (state_trans * r).exp();
    Eigen::MatrixXf init_points = init_status.points;
    Eigen::MatrixXf t = me * init_points;

    r_zonotope->points = Eigen::MatrixXf::Zero(static_cast<int>(init_points.rows() * init_points.rows()));
    
    for(size_t i = 0; i < init_points.rows(); i++){
        for (size_t j = 0; j < t.rows(); j++){
            r_zonotope->points(i+j, 0) = init_points(i, 0) + alpha * (t(j, 0) - init_points(i, 0));
            r_zonotope->points(i+j, 1) = init_points(i, 1) + alpha * (t(j, 1) - init_points(i, 1)); 
        }
    }
}

}
}