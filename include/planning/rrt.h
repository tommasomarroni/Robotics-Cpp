#pragma once

#include <iostream>
#include <vector>
#include <cmath>

#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>

class Node {
public:
    float x, y;
    Node* parent;
    std::vector<Node*> path;
    float cost;

    Node(float x_, float y_): x(x_), y(y_), parent(NULL), cost(0) {};
};

class RRT {
private:
    Node* start;
    Node* goal;
    std::vector<std::vector<float>> obstacles;
    int max_iter;
    float incr;
    float epsilon;
    float epsilon_goal;
    float scaling;
    bool draw;
    int window_x;
    int window_y;
    cv::Mat image;
    std::vector<Node*> nodes;

public:
    RRT(Node* start_, Node* goal_, std::vector<std::vector<float>> obstacles_, int max_iter_, float incr_, float epsilon_, float epsilon_goal_, float scaling_, bool draw_ = true, int window_x_ = 800, int window_y_ = 500);

    // Start path planning
    std::vector<Node*> planning();

    // Collision check
    bool collision_check(Node* q);

    // Initialize window, obstacles, start and goal configuration
    cv::Mat draw_init();

    // Draw obstacles, start and goal configuration
    void draw_env();

    // Draw configuration
    void draw_conf(Node* conf, Node* nearest);

    // Draw path
    void draw_path(Node* q);

    // Scale configuration space to window pixels
    int scale(float configuration, float scaling) { return (int) round(configuration*scaling); };

    // Scale window pixels to configuration space
    float descale(int pixel_pos, float scaling) { return pixel_pos/scaling; };
};
