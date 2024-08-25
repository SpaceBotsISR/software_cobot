// Controller Node
#include "attitude_controller_cpp/controller.hpp"

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <iostream>
#include <casadi/casadi.hpp>

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);

    auto controller = new Controller();
    
    controller->setup(); 



    std::vector<double> w0 = {0.0, 0.0, 0.0};
    std::vector<double> w_dot0 = {0.0, 0.0, 0.0};
    std::vector<double> q0 = {0.1825742, 0.3651484, 0.5477226, 0.7302967};
    std::vector<double> desired_rotation = {0.1825742, 0.3651484, 0.5477226, 0.7302967};;

    controller->step(w0, w_dot0, q0, desired_rotation);
}
