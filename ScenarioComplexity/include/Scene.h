#include "Vehicle.h"
#include "LaneInfo.h"
#include <vector>

namespace ScenarioComplexity
{
class Scene
{
public:
    Scene() {};
    ~Scene() {};

    void assertVehicleInScene();

    void getEdge();

private:
    std::vector<Vehicle> _sceneVehicles;
    std::vector<LaneInfo> _laneInfos;
    std::vector<std::pair<double, double>> _edge;

};


} // namespace ScenarioComplexity
