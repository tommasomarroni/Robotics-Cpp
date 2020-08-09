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
    Eigen::Matrix3f T;

    // Generate two random point clouds
    std::vector<Eigen::Vector2f> Q, P;

    // Define an initial "error" transformation
    Eigen::Matrix2f R, R_icp;
    Eigen::Vector2f t, t_icp;

    R << cos(M_PI/4.0), - sin(M_PI/4.0),
         sin(M_PI/4.0), cos(M_PI/4.0);

    t << -2, 5;

    // Apply it to P
    float y, x;
    for(int i = 0; i < 100; i++) {
        x = i/5.0;
        y = sin(x)*(x/7.0);
        Q.push_back(Eigen::Vector2f(x, y));
        P.push_back(R*Q.back() + t);
    }

    // Example: use ICP to align the point cloud P to the reference point cloud Q
    //ICP_2D::icp_svd icp(Q, P);
    ICP_2D::icp_svd icp(Q, P, 0.1, true, true, {-5.0, 25.0}, {-5.0, 25.0}, 800, 800);
    T = icp.start();

    // How to get R and t from T
    R_icp = T.block<2,2>(0,0);
    t_icp = T.block<2,1>(0,2);

    std::cout << "\n----------" << std::endl;
    std::cout << "\n[*] Original R:\n" << R << std::endl;
    std::cout << "\n[*] Original t:\n" << t << std::endl;
}
