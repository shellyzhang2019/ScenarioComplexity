#include <iostream>
#include <vector>
#include <cmath>

namespace ScenarioComplexity{

namespace Polygon{

/**
 * @brief 
 * 
 * @param vector0 origin vector
 * @param vector1 target vector
 * @return double positive if inverse
 */
double cross(const std::pair<double, double>& vector0, const std::pair<double, double>& vector1){
    return -vector0.second * vector1.first + vector0.first * vector1.second;
}

double cross(const Point& p0, const std::shared_ptr<Point>& p1, const std::shared_ptr<Point>& p2){
    // note: p0p1 is the edge of the convex
    return -(p1->y - p0.y) * (p2->x - p0.x) + (p1->x - p0.x) * (p2->y - p0.y);
}

// bool assertPointInConvex(const std::pair<double, double>& p2, const std::vector<std::pair<double,double>>& convex){
//     auto pi = convex.begin();
//     while(pi != convex.end()){
//         if(cross(*pi, *(pi+1), p2) < 0){
//             return false;
//         }
//         pi += 1;
//     }
//     return true;
    
// }

std::pair<std::pair<int, std::shared_ptr<Point>>, std::pair<int, std::shared_ptr<Point>>> findBoundaryPoint(const std::vector<std::shared_ptr<Point>>& points,
                                                                          const Point& init){
    auto upper = points[0];
    auto lower = points[0];
    int upper_idx=0, lower_idx=0;
    for (int i = 0; i < points.size(); i++){
        if (cross(init, upper, points[i]) >= 0){
            upper_idx = i;
            upper = points[i];
        }
    }
    for (int j = 0; j < points.size(); j++){
        if(cross(init, lower, points[j]) < 0){
            lower_idx = j;
            lower=points[j];
        }
    }
    
    return std::make_pair(std::make_pair(upper_idx, upper), std::make_pair(lower_idx, lower));

}

std::vector<std::pair<int, std::shared_ptr<Point>>> findConvex(std::vector<std::shared_ptr<Point>>& points, const Point& init_point){
    // first find the lower bound end point as the start of the convex
    std::vector<std::pair<int, std::shared_ptr<Point>>> convex;
    std::vector<std::pair<double, double>> vector_trj;
    auto bp = findBoundaryPoint(points, init_point);
    auto lower_point = bp.second.second;
    int lower_point_idx = bp.second.first;
    convex.emplace_back(std::make_pair(lower_point_idx, lower_point));
    auto vector_0 = std::make_pair(lower_point->x - init_point.x, lower_point->y - init_point.y);
    vector_trj.emplace_back(vector_0);   

    for (int i=0; i< points.size();i++){
        auto vector_1 = std::make_pair(points[i]->x - lower_point->x, points[i]->y - lower_point->y);
        if (cross(vector_0, vector_1) >= 0){
            convex.emplace_back(std::make_pair(i, points[i]));
            vector_trj.emplace_back(vector_1);
        }
        else{
            while (cross(vector_0, vector_1) < 0){
                if (convex.size() > 0){
                    convex.pop_back();
                    vector_trj.pop_back();
                    auto pPre = convex.back().second;
                    vector_0 = vector_trj.back();
                    vector_1 = std::make_pair(points[i]->x - pPre->x, points[i]->y - pPre->y);
                }else{
                    std::cout << "error, probably a bug" << std::endl;
                }
                
            }
            convex.emplace_back(std::make_pair(i, points[i]));
            vector_trj.emplace_back(vector_1);
        }

    }
    return convex;

}


}
}