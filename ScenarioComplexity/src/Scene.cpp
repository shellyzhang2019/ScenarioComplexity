#include "Scene.h"
#include <math/polygon.h>

using namespace ScenarioComplexity;

void Scene::assertVehicleInScene(){
    auto vi = _sceneVehicles.begin();
    while(vi != _sceneVehicles.end() + 1){
        std::pair<double, double> vehicle_pos((*vi).getStatus().posx, (*vi).getStatus().posy);
        if(!Polygon::assertPointInConvex(vehicle_pos, _edge)){
            _sceneVehicles.erase(vi);
        }
        vi += 1;
    }
}

void Scene::getEdge(){
    // inverse clock get
    _edge.emplace_back(this->_laneInfos.front().x.front(), this->_laneInfos.back().y.front());
    _edge.emplace_back(this->_laneInfos.back().x.front(), this->_laneInfos.back().y.front());
    _edge.emplace_back(this->_laneInfos.back().x.back(), this->_laneInfos.back().y.back());
    _edge.emplace_back(this->_laneInfos.front().x.back(), this->_laneInfos.front().y.back());
}