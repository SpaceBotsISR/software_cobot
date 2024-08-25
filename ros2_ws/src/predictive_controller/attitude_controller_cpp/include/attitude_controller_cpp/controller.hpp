#ifndef __CONTROLLER__
#define __CONTROLLER__

#include <casadi/casadi.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>

#include <iostream>
#include <stdexcept>
#include <filesystem>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>

#include "attitude_controller_cpp/RotationCasadi.hpp"

typedef Eigen::Matrix<double, 3, 6> EigenMatrix3x6;


class Controller : public rclcpp::Node {
public:
  Controller(int N = 10, 
            double dt = 1.0 / 50.0);
  ~Controller();
  void setup();
  void step(std::vector <double> w0, 
            std::vector <double> w_dot0, 
            std::vector <double> q0, 
            std::vector <double> desired_rotation);



  static inline bool file_exist(std::string file_path) {
    std::ifstream file(file_path.c_str());
    return file.good();
  }

  /*
    Integrate quaternion q using angular velocity w over time t.

    Parameters:
        q: Quaternion (4x1 CasADi MX).
        w: Angular velocity (3x1 CasADi MX).
        t: Time step (double).

    Returns:
        Integrated quaternion (4x1 CasADi MX).
  */
  static casadi::MX quaternion_integration(const casadi::MX& q, const casadi::MX& w, double t) {
      double epsilon = 0.000001;
      casadi::MX w_ = w + epsilon; 
      casadi::MX w_norm = norm_2(w_);

      // Compute the quaternion p
      casadi::MX p = vertcat(
          w / (w_norm) * sin((w_norm) * t / 2),
          cos(w_norm * t / 2)
      );

      // Create the q_ matrix
      auto q_ = casadi::MX::vertcat ({
          casadi::MX::horzcat({ q(3),  q(2), -q(1), q(0)}),
          casadi::MX::horzcat({-q(2),  q(3),  q(0), q(1)}),
          casadi::MX::horzcat({ q(1), -q(0),  q(3), q(2)}),
          casadi::MX::horzcat({-q(0), -q(1), -q(2), q(3)})
      });

      // Multiply q_ by p to get the integrated quaternion
      return casadi::MX::mtimes(q_, p);
  }

  template <typename T>
  static casadi::DM eigenToCasadi(const Eigen::MatrixBase<T>& eigen_matrix) {
    // Access the underlying derived type to use the data() method
    const auto& matrix = eigen_matrix.derived();
    
    // Create a vector from the Eigen matrix's data
    std::vector<double> data(matrix.data(), matrix.data() + matrix.size());
    
    // Return a reshaped CasADi DM based on the dimensions of the Eigen matrix
    return casadi::DM::reshape(casadi::DM(data), matrix.rows(), matrix.cols());
  }


  static inline Eigen::Matrix3d vecToMatrix3d(std::vector<double> vec) {
    if (vec.size() < 9) {
      throw std::invalid_argument("Vector must have 9 elements");
    }

    vec.resize(9);

    Eigen::Matrix3d mat;
    mat << vec[0], vec[1], vec[2],
           vec[3], vec[4], vec[5],
           vec[6], vec[7], vec[8];
    return mat;
  }

  static inline Eigen::Vector3d vecToeigVector(std::vector<double> vec) {
    if (vec.size() < 3) {
      throw std::invalid_argument("Vector must have 3 elements");
    }

    vec.resize(3);

    Eigen::Vector3d eig_vec;
    eig_vec << vec[0], vec[1], vec[2];
    return eig_vec;
  }

  static inline EigenMatrix3x6 vecToMatrix3_6(std::vector<double> vec) {
    if (vec.size() < 18) {
      throw std::invalid_argument("Vector must have 18 elements");
    }

    // Resize the vector to 18 elements
    vec.resize(18);

    EigenMatrix3x6 mat;
    mat << vec[0], vec[1], vec[2], vec[3], vec[4], vec[5],
           vec[6], vec[7], vec[8], vec[9], vec[10], vec[11],
           vec[12], vec[13], vec[14], vec[15], vec[16], vec[17];
    return mat;
  }

  int N;
  double dt;

  casadi::Opti * opti;

  bool setup_done = false;

  RotationCasadi rotation;

  // Parameters
  casadi::MX w0; 
  casadi::MX w_dot0;
  casadi::MX q0;
  casadi::MX desired_rotation;

  // Systems parameters
  casadi::MX J; 
  casadi::MX g; 
  casadi::MX A; 
  casadi::MX c;

  // Variables
  casadi::MX w;
  casadi::MX w_dot; 
  casadi::MX q; 
  casadi::MX u;

  Eigen::Matrix3d J_;
  Eigen::Vector3d c_;
  Eigen::Vector3d g_;

  EigenMatrix3x6 A_;
};

#endif
