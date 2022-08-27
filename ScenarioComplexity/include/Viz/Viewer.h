#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <vector>
#include <memory>
#include "Geo.h"

namespace ScenarioComplexity{

class Viewer{

public:
void ShowConvex(const std::vector<std::pair<std::shared_ptr<Point>, std::shared_ptr<Point>>>&,
                const std::vector<std::pair<std::shared_ptr<Point>, std::shared_ptr<Point>>>&,
                const std::vector<std::pair<std::shared_ptr<Point>, std::shared_ptr<Point>>>&);

};
}