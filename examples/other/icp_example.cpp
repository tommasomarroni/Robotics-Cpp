#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Eigen>

#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>

#include "other/icp.h"

int main()
{
    // Generate two random point clouds
    std::vector<Eigen::Vector2f> Q, P;

    // Define an initial transformation
    Eigen::Matrix2f R;
    Eigen::Vector2f t;

    R << cos(M_PI/4), - sin(M_PI/4),
         sin(M_PI/4), cos(M_PI/4);

    t << -2, 5;

    float y, x;
    for(int i = 0; i < 150; i++) {
        x = i/10.0;
        y = sin(x)*(x/5.0);
        Q.push_back(Eigen::Vector2f(x, y));
        P.push_back(R*Q.back() + t);
    }

    ICP_2D::icp_svd myIcp(Q, P, true, {-5.0, 15.0}, {-5.0, 15.0}, 1000, 1000);
    myIcp.make_draw();



}
