#pragma once

#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <Eigen/Eigen>

#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>

namespace ICP_2D {

class icp_svd {
private:
    std::vector<Eigen::Vector2f> Q;
    std::vector<Eigen::Vector2f> P;
    float epsilon;
    bool draw;
    bool verbose;
    std::array<float, 2> x_lim;
    std::array<float, 2> y_lim;
    int window_x;
    int window_y;
    cv::Mat image;

    Eigen::Matrix3f T = Eigen::Matrix3f::Identity();

public:
    icp_svd(std::vector<Eigen::Vector2f> Q_, std::vector<Eigen::Vector2f> P_, float epsilon_ = 0.1, bool draw_ = false, bool verbose_ = true, std::array<float, 2> x_lim_ = {-25.0, 25.0}, std::array<float, 2> y_lim_ = {-25.0, 25.0}, int window_x_ = 900, int window_y_ = 900);

    // Start ICP iterations
    Eigen::Matrix3f start();

    // Compute mean and eenter point clouds
    Eigen::Vector2f center_data(std::vector<Eigen::Vector2f>& PC, std::vector<Eigen::Vector2f>& PC_centered);

    // Compute correspondences between two point clouds
    void compute_correspondences(std::vector<Eigen::Vector2f>& Q_, std::vector<Eigen::Vector2f>& P_, std::vector<int>& correspondences_);

    // Compute covariance matrix from Q, P, and correspondences
    void compute_cov(std::vector<Eigen::Vector2f>& Q_, std::vector<Eigen::Vector2f>& P_, Eigen::Vector2f mu_Q_, Eigen::Vector2f mu_P_, std::vector<int>& correspondences_, Eigen::Matrix2f& cov_);

    // Draw initialization
    cv::Mat draw_init();

    // Draw point clouds
    void draw_points(std::vector<Eigen::Vector2f>& Q_, std::vector<Eigen::Vector2f>& P_);

    // Draw correspondences
    void draw_correspondences(std::vector<Eigen::Vector2f>& Q_, std::vector<Eigen::Vector2f>& P_, std::vector<int>& correspondences_);

    // General rule to scale x in [min(x), max(x)] to x' in [a, b]: x' = a + ((x - min(x))*(b - a))/(max(x) - min(x))
    std::vector<int> scale(Eigen::Vector2f point) { return {(int) round(((point[0] - x_lim[0])*window_x)/(x_lim[1] - x_lim[0])), (int) round(window_y - ((point[1] - y_lim[0])*window_y)/(y_lim[1] - y_lim[0]))}; };

    //Eigen::Vector2f descale(std::vector<int> pos) { return {x_lim[0] + (pos[0]*(x_lim[1] - x_lim[0]))/(window_x - 0), y_lim[0] + (((window_y - pos[1]) - 0)*(y_lim[1] - y_lim[0]))/(window_y - 0)}; };
};

}
