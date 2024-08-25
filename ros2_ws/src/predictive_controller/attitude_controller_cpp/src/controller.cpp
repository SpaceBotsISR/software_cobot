#include "attitude_controller_cpp/controller.hpp"
#include "attitude_controller_cpp/libbnpy/include/npy.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"



Controller::Controller(int N, double dt)
    : Node("controller"), N(N), dt(dt)
{

    std::string package_name = "attitude_controller_cpp";
    std::string package_share_directory = ament_index_cpp::get_package_share_directory(package_name);

    RCLCPP_INFO(get_logger(), "Package share directory: %s", package_share_directory.c_str());

    // Create the variables for the optimization problem
    std::string J_file_path = {"system_parameters/J.npy"};
    std::string c_file_path = {"system_parameters/c.npy"};
    std::string g_file_path = {"system_parameters/g.npy"};
    std::string A_file_path = {"system_parameters/A.npy"};

    J_file_path = package_share_directory + "/" + J_file_path;
    c_file_path = package_share_directory + "/" + c_file_path;
    g_file_path = package_share_directory + "/" + g_file_path;
    A_file_path = package_share_directory + "/" + A_file_path;

    // Load the system parameters
    npy::npy_data J = npy::read_npy<double> (J_file_path);
    npy::npy_data c = npy::read_npy<double> (c_file_path);
    npy::npy_data g = npy::read_npy<double> (g_file_path);
    npy::npy_data A = npy::read_npy<double> (A_file_path);

    this->J_ = vecToMatrix3d(J.data);
    this->c_ = vecToeigVector(c.data);
    this->g_ = vecToeigVector(g.data);
    this->A_ = vecToMatrix3_6(A.data);

    std::cout << "J: " << std::endl << this->J_ << std::endl;
    std::cout << "c: " << std::endl << this->c_ << std::endl;
    std::cout << "g: " << std::endl << this->g_ << std::endl;
    std::cout << "A: " << std::endl << this->A_ << std::endl;

    this->opti = new casadi::Opti();

    // parameter required for the optimization problem
    this->w0 = this->opti->parameter(3, 1);
    this->w_dot0 = this->opti->parameter(3, 1);
    this->q0 = this->opti->parameter(4, 1);
    this->desired_rotation = this->opti->parameter(4, 1);

    // Systems parameter
    this->J = this->opti->parameter(3, 3);
    this->g = this->opti->parameter(3, 1);
    this->A = this->opti->parameter(3, 6);
    this->c = this->opti->parameter(3, 1);

    // Variables
    this->w = this->opti->variable(3, N);
    this->w_dot = this->opti->variable(3, N);
    this->q = this->opti->variable(4, N);
    this->u = this->opti->variable(6, N);
}

Controller::~Controller()
{
}

void Controller::setup() {
    if (this->setup_done) {
        std::cout << "leaving setup stage " << std::endl;
        return;
    }

    // Setup system parameter
    this->opti->set_value(this->J, this->eigenToCasadi(this->J_));
    this->opti->set_value(this->g, this->eigenToCasadi(this->g_));
    this->opti->set_value(this->A, this->eigenToCasadi(this->A_));
    this->opti->set_value(this->c, this->eigenToCasadi(this->c_));

    // Setup initial conditions
    this->opti->set_value(this->w0, {0, 0, 0});
    this->opti->set_value(this->w_dot0, {0, 0, 0});
    this->opti->set_value(this->q0, {0, 0, 0, 1});
    this->opti->set_value(this->desired_rotation, {0, 0, 0, 1});

    this->opti->subject_to(this->q(casadi::Slice(), 0) == this->q0);
    this->opti->subject_to(this->w(casadi::Slice(), 0) == this->w0);
    this->opti->subject_to(this->w_dot(casadi::Slice(), 0) == this->w_dot0);

    
    for (int i = 0; i < N; i++) {
        // Make sure the norm of the quaternion is 1 for all the horizon, this will help to make sure we dont go to bad rotations
        this->opti->set_initial(this->q(casadi::Slice(), i), {0, 0, 0, 1}); // initial guess for the quaternion must have norm 1
        //this->opti->subject_to(norm_2(this->q(casadi::Slice(), i)) == 1);

        // Add boundaries for value of the actuation
        for (int j = 0; j < 6; j++) {
            this->opti->subject_to(this->opti->bounded(-2, this->u(j, i), 2));
        }
    }


    for (int i = 0; i < this->N; i++) {
        this->rotation.fromQuat(this->q(casadi::Slice(), i));
        auto c_r = this->rotation.toRotationMatrix();
        

        // System dynamics
        this->opti->subject_to(
            (casadi::MX::mtimes(J, this->w_dot(casadi::Slice(), i)) +
            casadi::MX::mtimes(casadi::MX::skew(this->w(casadi::Slice(), i)),casadi::MX::mtimes(this->J, this->w(casadi::Slice(), i))) -
            casadi::MX::mtimes(casadi::MX::skew(this->c), casadi::MX::mtimes(c_r.T(), this->g)) )==
            casadi::MX::mtimes(this->A, this->u(casadi::Slice(), i))
        );   

        if (i < this->N - 1) {
            // Integrate quaternion
            this->opti->subject_to(
                this->q(casadi::Slice(), i + 1) == this->quaternion_integration(this->q(casadi::Slice(), i), this->w(casadi::Slice(), i), this->dt)
            );

            // Integrate angular velocity
            this->opti->subject_to(
                this->w(casadi::Slice(), i + 1) == this->w(casadi::Slice(), i) + this->dt *  this->w_dot(casadi::Slice(), i)
            );
        }
    }

    this->rotation.fromQuat(this->desired_rotation);
    auto desired_rot = this->rotation.toRotationMatrix();
    desired_rot = desired_rot.T();
    

    casadi::MX cost = 0;
    for (int i = 0; i < this->N; i++) {
        this->rotation.fromQuat(this->q(casadi::Slice(), i));
        auto c_r = this->rotation.toRotationMatrix();
        // desired rotation is a quat here, need to convert to rotation matrix
        cost += casadi::MX::trace(casadi::MX::eye(3) - casadi::MX::mtimes(c_r, desired_rot));
    }

    this->opti->minimize(cost);

    this->opti->solver("ipopt");


    
    this->setup_done = true;



     

}

void Controller::step(std::vector <double> w0, 
                      std::vector <double> w_dot0, 
                      std::vector <double> q0, 
                      std::vector <double> desired_rotation) {
    if (!this->setup_done) {
        throw std::runtime_error("The setup method must be called before calling the step method");
    }

    if (w0.size() != 3) {
        throw std::invalid_argument("The initial angular velocity must have 3 elements");
    }

    if (w_dot0.size() != 3) {
        throw std::invalid_argument("The initial angular acceleration must have 3 elements");
    }

    if (q0.size() != 4) {
        throw std::invalid_argument("The initial quaternion must have 4 elements");
    }

    if (desired_rotation.size() != 4) {
        throw std::invalid_argument("The desired rotation must have 4 elements");
    }

    this->opti->set_value(this->w0, w0);
    this->opti->set_value(this->w_dot0, w_dot0);
    this->opti->set_value(this->q0, q0);
    this->opti->set_value(this->desired_rotation, desired_rotation);



    this->opti->solve();

    auto A = this->opti->value(this->A);
    auto c = this->opti->value(this->c);
    auto g = this->opti->value(this->g);
    auto J = this->opti->value(this->J);

    std::cout << "A: " << std::endl << A << std::endl;
    std::cout << "c: " << std::endl << c << std::endl;
    std::cout << "g: " << std::endl << g << std::endl;
    std::cout << "J: " << std::endl << J << std::endl;


    std::cout << "u" << std::endl;

    for (int i = 0; i < this->N; i++) {
        auto u = this->opti->value(this->u(casadi::Slice(), i));
        std::cout << u << std::endl;
    }

    std::cout << "w" << std::endl; 

    for (int i = 0; i < this->N; i++) {
        auto q = this->opti->value(this->w(casadi::Slice(), i));
        std::cout << q << std::endl;
    }

    std::cout << "w_dot" << std::endl;

    for (int i = 0; i < this->N; i++) {
        auto q = this->opti->value(this->w_dot(casadi::Slice(), i));
        std::cout << q << std::endl;
    }

    std::cout << "q" << std::endl;
    for (int i = 0; i < this->N; i++) {
        auto q = this->opti->value(this->q(casadi::Slice(), i));
        std::cout << q << std::endl;
    }

    auto skew_c = this->opti->value(casadi::MX::skew(this->c));
    std::cout << "skew_c: " << std::endl << skew_c << std::endl;
    // Solve the optimization problem

    // all to rotation matrix 

    for (int i = 0; i < this->N; i++) {
        auto q = this->opti->value(this->q(casadi::Slice(), i));
        this->rotation.fromQuat(q);
        auto c_r = this->opti->value(this->rotation.toRotationMatrix());
        
        std::cout << "Rotation matrix: " << std::endl << c_r << std::endl;
    }


    return;
}


