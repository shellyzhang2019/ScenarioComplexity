#include "math/zonotope.h"

using namespace ScenarioComplexity::Zonotope;

Zonotope::Zonotope(){}

Zonotope::Zonotope(const Eigen::Vector2f& center,
                   const Eigen::MatrixXf& generators):
    _center(center),
    _generators(generators){
    GeneratePoints();
}

Zonotope::Zonotope(const std::vector<std::pair<float, float>>& points):
    points(points){}