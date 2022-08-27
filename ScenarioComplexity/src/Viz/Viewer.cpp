#include "Viz/Viewer.h"
#include "matplotlibcpp.h"

using namespace ScenarioComplexity;

namespace plt = matplotlibcpp;

void Viewer::ShowConvex(const std::vector<std::pair<std::shared_ptr<Point>, std::shared_ptr<Point>>>& monte_carlo_points,
                        const std::vector<std::pair<std::shared_ptr<Point>, std::shared_ptr<Point>>>& convex_points,
                        const std::vector<std::pair<std::shared_ptr<Point>, std::shared_ptr<Point>>>& center_points){
    // plot motecarlo
    int n = static_cast<int>(monte_carlo_points.size()*2);
    std::vector<double> x(n), y(n);
    for (int i=0; i< n-2; i+=2){
        int j = static_cast<int>(i / 2);
        x.at(i) = monte_carlo_points[j].first->x;
        y.at(i) = monte_carlo_points[j].first->y;
        x.at(i+1) = monte_carlo_points[j].second->x;
        y.at(i+1) = monte_carlo_points[j].second->y;
        std::cout << x[i] << std::endl;
    }
    // plt::figure_size(1200, 780);
    
    plt::plot(x, y, "*");
    
    // plot extreme reachable convex
    int m = static_cast<int>(convex_points.size()*2);
    std::vector<double> cx(m), cy(m);
    for (int i=0; i< m-2; i+=2){
        int j = static_cast<int>(i / 2);
        cx.at(i) = convex_points[j].first->x;
        cy.at(i) = convex_points[j].first->y;
        cx.at(i+1) = convex_points[j].second->x;
        cy.at(i+1) = convex_points[j].second->y;
    }
    plt::plot(cx, cy, "ro");

    // plot lower extreme points

    m = static_cast<int>(center_points.size()*2);
    std::vector<double> ccx(m), ccy(m);
    for (int i=0; i< m-2; i+=2){
        int j = static_cast<int>(i / 2);
        ccx.at(i) = center_points[j].first->x;
        ccy.at(i) = center_points[j].first->y;
        ccx.at(i+1) = center_points[j].second->x;
        ccy.at(i+1) = center_points[j].second->y;
    }
    plt::plot(ccx, ccy, "gx");
    // for(auto vec: convex){
    //     std::cout << vec.size() << std::endl;
    //     int n = static_cast<int>(vec.size());
    //     std::vector<double> ccx(n), ccy(n);
    //     for(int i =0; i< n; i++){
    //         ccx.at(i) = vec[i]->x;
    //         ccy.at(i) = vec[i]->y; 
    //     }
    //     std::cout << ccx[0] << ccx[1] << ccx[2] << std::endl;
    //     plt::plot(ccx, ccy, "gx");
    // }

    
    // plt::xlim(0, 40);
    // plt::ylim(-3.5, 3.5);
    plt::show();

}