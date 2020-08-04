#include <iostream>
#include <vector>
#include <random>
#include <cmath>

#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>

#include "planning/rrt.h"

#define DELAY_DRAW 25
#define INITIAL_DELAY_DRAW 1000

/* -------------------- RRT: pseudo-code --------------------
    pseudo code - RRT

    Algorithm RRT
        Input: Initial configuration q_init, number of vertices in RRT K, incremental distance delta_q, goal configuration q_goal
        Output: RRT graph G

        G.init(q_init)
        [draw]
        for k = 1 to K do
            q_rand <- RAND_CONF()
            q_near <- NEAREST_VERTEX(q_rand, G)
            q_new <- NEW_CONF(q_near, q_rand, delta_q)
            if collision:
                continue
            G.add_vertex(q_new)
            G.add_edge(q_near, q_new)
            [draw]
            if q_new is equal to q_goal:
                break
        return G
*/

RRT::RRT(Node* start_, Node* goal_, std::vector<std::vector<float>> obstacles_, int max_iter_, float incr_, float epsilon_, float epsilon_goal_, float scaling_, bool draw_, int window_x_, int window_y_): start(start_), goal(goal_), obstacles(obstacles_), max_iter(max_iter_), incr(incr_), epsilon(epsilon_), epsilon_goal(epsilon_goal_), scaling(scaling_), draw(draw_), window_x(window_x_), window_y(window_y_) {
    if (draw) {
        image = draw_init();
        draw_env();
    }
}

std::vector<Node*> RRT::planning() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> q_rand_gen_x(descale(0, scaling), descale(window_x, scaling)), q_rand_gen_y(descale(0, scaling), descale(window_y, scaling));

    Node* q_rand;
    Node* q_nearest;
    Node* q_new;

    float min_distance;
    float temp_dist;

    nodes.push_back(start);

    for (int i = 0; i < max_iter; i++) {
        // Step 1: Generate random configuration
        q_rand = new Node(q_rand_gen_x(gen), q_rand_gen_y(gen));

        // Step 2: Search for closest configuration
        min_distance = std::numeric_limits<float>::max();
        for (Node* node : nodes) {
            temp_dist = sqrtf(pow(q_rand->x - node->x, 2) + pow(q_rand->y - node->y, 2));
            if (temp_dist < min_distance) {
                min_distance = temp_dist;
                q_nearest = node;
            }
        }

        // Step 3: Scale for max increment and eventually compute new min_distance
        if (min_distance > incr) {
            q_new = new Node((q_rand->x - q_nearest->x)*(incr/min_distance) + q_nearest->x, (q_rand->y - q_nearest->y)*(incr/min_distance) + q_nearest->y);
            min_distance = sqrtf(pow(q_new->x - q_nearest->x, 2) + pow(q_new->y - q_nearest->y, 2));
        } else q_new = q_rand;

        // Step 4: Check if q_new is too close to q_nearest
        if (sqrtf(pow(q_new->x - q_nearest->x, 2) + pow(q_new->y - q_nearest->y, 2)) < epsilon) continue;

        // Step 5: Check collision
        if (collision_check(q_new)) continue;

        // Step 6: Add q_new to graph
        q_new->parent = q_nearest;
        q_new->cost = q_nearest->cost + min_distance;
        q_new->path = q_nearest->path;
        q_new->path.push_back(q_nearest);

        nodes.push_back(q_new);

        // Step 7: Refresh graph
        if (draw) draw_conf(q_new, q_nearest);

        // Step 8: Check if goal configuration is reached
        if (sqrtf(pow(q_new->x - goal->x, 2) + pow(q_new->y - goal->y, 2)) < epsilon_goal) {
            goal->parent = q_new;
            goal->cost = q_new->cost + sqrtf(pow(goal->x - q_new->x, 2) + pow(goal->y - q_new->y, 2));
            goal->path = q_new->path;
            goal->path.push_back(q_new);

            std::cout << "Found path! Total cost: " << goal->cost << std::endl;
            if (draw) {
                cv::putText(image, "FOUND PATH!", cv::Point(20, 35), cv::FONT_HERSHEY_TRIPLEX, 1.0, CV_RGB(0, 0, 200), 2);
                cv::imshow("RRT", image);

                goal->path.push_back(goal);
                draw_path(goal);
                goal->path.pop_back();
            }
                
            // Stop iterating if goal is reached
            break;
        }
    }

    if (goal->path.empty()) {
        std::cout << "Path not found!" << std::endl;
        if (draw) {
            cv::putText(image, "REACHED MAX ITERATIONS!", cv::Point(20, 35), cv::FONT_HERSHEY_TRIPLEX, 1.0, CV_RGB(0, 0, 200), 2);
            cv::imshow("RRT", image);
        }
    }

    if (draw) cv::waitKey(0);
    return goal->path;
}

bool RRT::collision_check(Node* q) {
    for (std::vector<float> obstacle : obstacles) {
        if ((q->x > obstacle[0]) && (q->x < obstacle[2]) && (q->y > obstacle[1]) && (q->y < obstacle[3])) return true;
    }

    return false;
}

cv::Mat RRT::draw_init() {
    // Initialize
    cv::Mat image(window_y, window_x, CV_8UC3, cv::Scalar(240, 240, 240));
    if (!image.data) {
        std::cout <<  "Could not open or find the image!" << std::endl ;
        exit(EXIT_FAILURE);
    }

    return image;
}

void RRT::draw_env() {
    // Draw obstacles, start and goal configurations
    for (std::vector<float> obstacle : obstacles) {
        cv::Point p1_obst(scale(obstacle[0], scaling), scale(obstacle[1], scaling)), p2_obst(scale(obstacle[2], scaling), scale(obstacle[3], scaling));
        cv::rectangle(image, p1_obst, p2_obst, cv::Scalar(150, 150, 150), cv::FILLED);
    }

    cv::Point centerStart(scale(start->x, scaling), scale(start->y, scaling));
    cv::Point centerGoal(scale(goal->x, scaling), scale(goal->y, scaling));
    cv::circle(image, centerStart, 10, cv::Scalar(0, 0, 180), cv::FILLED); // CV_FILLED
    cv::circle(image, centerGoal, 10, cv::Scalar(0, 0, 180), cv::FILLED); // CV_FILLED

    cv::imshow("RRT", image);
    cv::waitKey(INITIAL_DELAY_DRAW);
}

void RRT::draw_conf(Node* conf, Node* nearest) {
    // Draw new configuration and edge
    cv::Point centerConf(scale(conf->x, scaling), scale(conf->y, scaling));
    cv::circle(image, centerConf, 3, cv::Scalar(0, 0, 0), cv::FILLED); // CV_FILLED

    cv::Point p1(scale(conf->x, scaling), scale(conf->y, scaling)), p2(scale(nearest->x, scaling), scale(nearest->y, scaling));
    cv::line(image, p1, p2, cv::Scalar(0, 0, 0), 2);

    cv::imshow("RRT", image);
    cv::waitKey(DELAY_DRAW);
}

void RRT::draw_path(Node* q) {
    // Draw path
    for (Node* node : q->path) {
        if ((node->parent) != NULL) {
            cv::Point p1_path(scale(node->x, scaling), scale(node->y, scaling)), p2_path(scale((node->parent)->x, scaling), scale((node->parent)->y, scaling));
            cv::line(image, p1_path, p2_path, cv::Scalar(200, 0, 0), 3);
        }
    }

    cv::imshow("RRT", image);
}
