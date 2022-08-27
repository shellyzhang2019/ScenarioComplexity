#pragma once
#include <vector>
#include <iostream>

namespace ScenarioComplexity{
struct Point {

    Point() {}
    Point(const float& x_, const float& y_):x(x_),y(y_){}
    float x;
    float y;
};

struct Trajectory{
    
    Trajectory() {}
    Trajectory(const std::vector<Point>& data): points(data) {}

    std::vector<Point> points;
};
}