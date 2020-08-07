#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <Eigen/Eigen>

#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>

#include "other/icp.h"

#define DELAY_DRAW 1000

/* -------------------- ICP: pseudo-code --------------------
    pseudo code - ICP

    Algorithm ICP
        Input: -
        Output: -


*/

using namespace ICP_2D;

icp_svd::icp_svd(std::vector<Eigen::Vector2f> Q_, std::vector<Eigen::Vector2f> P_, bool draw_, std::array<float, 2> x_lim_, std::array<float, 2> y_lim_, int window_x_, int window_y_): Q(Q_), P(P_), draw(draw_), x_lim(x_lim_), y_lim(y_lim_), window_x(window_x_), window_y(window_y_), scaling_x(window_x_/(x_lim_[0]-x_lim_[1])), scaling_y(window_y_/(y_lim_[0]-y_lim_[1])) {
    if (draw) {
        image = draw_init();
    }
}

cv::Mat icp_svd::draw_init() {
    // Initialize
    cv::Mat image(window_y, window_x, CV_8UC3, cv::Scalar(0, 0, 0));
    if (!image.data) {
        std::cout <<  "Could not open or find the image!" << std::endl ;
        exit(EXIT_FAILURE);
    }

    return image;
}

void icp_svd::make_draw() {
    for (Eigen::Vector2f point : Q) {
        cv::Point centerStart(scale(point)[0], scale(point)[1]);
        cv::circle(image, centerStart, 2, cv::Scalar(0, 0, 180), cv::FILLED); // CV_FILLED
    }

    for (Eigen::Vector2f point : P) {
        cv::Point centerStart(scale(point)[0], scale(point)[1]);
        cv::circle(image, centerStart, 2, cv::Scalar(180, 0, 0), cv::FILLED); // CV_FILLED
    }
    cv::imshow("ICP: SVD", image);
    //cv::waitKey(DELAY_DRAW);
    cv::waitKey(0);
}
