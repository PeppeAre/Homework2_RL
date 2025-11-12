#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <functional>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"


#include "ros2_kdl_package/action/execute_trajectory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"   // Aggiunto per il subscriber ArUco

using FloatArray = std_msgs::msg::Float64MultiArray;

using TrajectoryAction = ros2_kdl_package::action::ExecuteTrajectory;
using GoalHandleTrajectory = rclcpp_action::ServerGoalHandle<TrajectoryAction>;
using namespace std::chrono_literals;

class Iiwa_pub_sub : public rclcpp::Node
{
public:
    Iiwa_pub_sub()
    : Node("ros2_kdl_node"),
      node_handle_(std::shared_ptr<Iiwa_pub_sub>(this, [](Iiwa_pub_sub*){}))
    {
        // Parametri esistenti
        declare_parameter("cmd_interface", "position");
        get_parameter("cmd_interface", cmd_interface_);
        declare_parameter("ctrl", "velocity_ctrl");
        get_parameter("ctrl", ctrl_);

        // Caricamento parametri da file (Punto 1a)
        declare_parameter("traj_duration", 1.5);
        declare_parameter("acc_duration", 0.5);
        declare_parameter("total_time", 1.5);
        declare_parameter("trajectory_len", 150);
        declare_parameter("Kp", 5.0);
        declare_parameter("end_position_x", 0.4);
        declare_parameter("end_position_y", 0.015);
        declare_parameter("end_position_z", 0.4);

        // Leggi i parametri
        get_parameter("traj_duration", traj_duration_);
        get_parameter("acc_duration", acc_duration_);
        get_parameter("total_time", total_time_);
        get_parameter("trajectory_len", trajectory_len_);
        get_parameter("Kp", Kp_);
        get_parameter("end_position_x", end_pos_x_);
        get_parameter("end_position_y", end_pos_y_);
        get_parameter("end_position_z", end_pos_z_);

        RCLCPP_INFO(get_logger(),"Current cmd interface is: '%s'", cmd_interface_.c_str());
        RCLCPP_INFO(get_logger(),
                    "Loaded parameters (from params file):\n"
                    "  traj_duration: %.2f\n"
                    "  acc_duration: %.2f\n"
                    "  total_time: %.2f\n"
                    "  trajectory_len: %d\n"
                    "  Kp: %.2f\n"
                    "  end_position: [%.3f, %.3f, %.3f]",
                        traj_duration_,
                        acc_duration_,
                        total_time_,
                        trajectory_len_,
                        Kp_,
                        end_pos_x_,
                        end_pos_y_,
                        end_pos_z_);

        if (!(cmd_interface_ == "position" || cmd_interface_ == "velocity" || cmd_interface_ == "effort" ))
        {
            RCLCPP_ERROR(get_logger(),"Selected cmd interface is not valid! Use 'position', 'velocity' or 'effort' instead...");
            throw std::runtime_error("Invalid cmd_interface");
        }

        declare_parameter("traj_type", "linear");
        get_parameter("traj_type", traj_type_);
        RCLCPP_INFO(get_logger(),"Current trajectory type is: '%s'", traj_type_.c_str());
        if (!(traj_type_ == "linear" || traj_type_ == "circular"))
        {
            RCLCPP_ERROR(get_logger(),"Selected traj type is not valid!");
            throw std::runtime_error("Invalid traj_type");
        }

        declare_parameter("s_type", "trapezoidal");
        get_parameter("s_type", s_type_);
        RCLCPP_INFO(get_logger(),"Current s type is: '%s'", s_type_.c_str());
        if (!(s_type_ == "trapezoidal" || s_type_ == "cubic"))
        {
            RCLCPP_ERROR(get_logger(),"Selected s type is not valid!");
            throw std::runtime_error("Invalid s_type");
        }

        iteration_ = 0; t_ = 0.0;
        joint_state_available_ = false;

        // robot_description 
        auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_handle_, "robot_state_publisher");
        while (!parameters_client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                rclcpp::shutdown();
            }
            RCLCPP_INFO(this->get_logger(), "robot_state_publisher parameters service not available, waiting again...");
        }
        auto parameter = parameters_client->get_parameters({"robot_description"});

        // KDL::Tree robot_tree; // <-- Spostato a membro della classe
        if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)){
            throw std::runtime_error("Failed to retrieve robot_description param!");
        }
        robot_ = std::make_shared<KDLRobot>(robot_tree);


        unsigned int nj = robot_->getNrJnts();
        KDL::JntArray q_min(nj), q_max(nj);
        q_min.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96;
        q_max.data <<  2.96, 2.09, 2.96, 2.09, 2.96, 2.09, 2.96;
        robot_->setJntLimits(q_min,q_max);

        joint_positions_.resize(nj);
        joint_velocities_.resize(nj);
        joint_positions_cmd_.resize(nj);
        joint_velocities_cmd_.resize(nj);
        joint_efforts_cmd_.resize(nj);
        joint_efforts_cmd_.data.setZero();

        // Subscriber agli stati giunto
        jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, std::placeholders::_1));

        // Attendi il primo JointState
        while(!joint_state_available_){
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for /joint_states ...");
            rclcpp::spin_some(node_handle_);
        }
        
        // --- Punto 2b --- //
        // Subscriber alla posa dell'ArUco marker
        arucoSubscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco_single/pose",    // nome topic pubblicato da aruco_ros
            300,                      // queue size
            std::bind(&Iiwa_pub_sub::aruco_pose_callback, this, std::placeholders::_1)
        );
        // ---  2b --- //


        // Inizializza robot, EE, controller
        robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
        KDL::Frame f_T_ee = KDL::Frame::Identity();
        robot_->addEE(f_T_ee);
        robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

        init_cart_pose_ = robot_->getEEFrame();

        controller_ = std::make_shared<KDLController>(*robot_);

        // Publisher del comando
        if (cmd_interface_ == "position") {
            cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa_arm_controller/commands", 10);
        } else if (cmd_interface_ == "velocity") {
            cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
            
            // --- Punto 2b ( timer) --- //
            if(ctrl_ == "vision"){
                RCLCPP_INFO(get_logger(), "ATTIVAZIONE CONTROLLO VISION (Punto 2b)");
                cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(100), // 10 Hz
                std::bind(&Iiwa_pub_sub::cmd_publisher_vision, this));
            }
            // --- Punto 2b --- //

        } else { // effort
            cmdPublisher_ = this->create_publisher<FloatArray>("/effort_controller/commands", 10);
        }

        // ACTION SERVER (Punto 1c)
        action_server_ = rclcpp_action::create_server<TrajectoryAction>(
            this,
            "execute_trajectory", // nome action server (aggiornato)
            std::bind(&Iiwa_pub_sub::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&Iiwa_pub_sub::handle_cancel, this, std::placeholders::_1),
            std::bind(&Iiwa_pub_sub::handle_accepted, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Action server 'execute_trajectory' ready to receive goals.");
    }

private:
    // ACTION HANDLERS (Punto 1c)
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const TrajectoryAction::Goal> goal)
    {

        RCLCPP_INFO(this->get_logger(), "Received new goal request!");
        (void)uuid;
        
        // --- Punto 2b --- //
        // Rifiuta il goal se il controllore vision è attivo
        if (ctrl_ == "vision") {
            RCLCPP_ERROR(this->get_logger(), "Cannot execute trajectory goal, node is in 'vision' control mode!");
            return rclcpp_action::GoalResponse::REJECT;
        }
        
        // Controlla il goal
        if (!goal->start) {
            RCLCPP_ERROR(this->get_logger(), "Goal rejected, 'start' flag is false.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        // --- Punto 2b --- //

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleTrajectory> goal_handle)
    {
        RCLCPP_WARN(this->get_logger(), "Received request to cancel trajectory");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleTrajectory> goal_handle)
    {
        // Esegui in thread separato per non bloccare l'esecutore
        std::thread{std::bind(&Iiwa_pub_sub::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleTrajectory> goal_handle)
    {
    RCLCPP_INFO(this->get_logger(), "Starting trajectory execution ...");

    RCLCPP_INFO(get_logger(), "Executing goal with parameters (from YAML):");
    RCLCPP_INFO(get_logger(), "  traj_duration: %.2f", traj_duration_);
    RCLCPP_INFO(get_logger(), "  acc_duration: %.2f", acc_duration_);
    RCLCPP_INFO(get_logger(), "  total_time: %.2f", total_time_);
    RCLCPP_INFO(get_logger(), "  trajectory_len: %d", trajectory_len_);
    RCLCPP_INFO(get_logger(), "  Kp: %.2f", Kp_);
    RCLCPP_INFO(get_logger(), "  end_position: [%.3f, %.3f, %.3f]", end_pos_x_, end_pos_y_, end_pos_z_);


    auto feedback = std::make_shared<TrajectoryAction::Feedback>();
    auto result   = std::make_shared<TrajectoryAction::Result>();

    iteration_ = 0;
    t_ = 0.0;

    //Pianifica traiettoria
    //Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data) - Eigen::Vector3d(0,0,0.1));
    Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data));
    Eigen::Vector3d end_position; end_position << end_pos_x_, end_pos_y_, end_pos_z_;
    double traj_radius = 0.15;

    if(traj_type_ == "linear"){
        planner_ = KDLPlanner(traj_duration_, acc_duration_, init_position, end_position);
        p_ = (s_type_ == "trapezoidal") ? planner_.linear_traj_trapezoidal(t_) : planner_.linear_traj_cubic(t_);
    } else {
        planner_ = KDLPlanner(traj_duration_, init_position, traj_radius, acc_duration_);
        p_ = (s_type_ == "trapezoidal") ? planner_.circular_traj_trapezoidal(t_) : planner_.circular_traj_cubic(t_);
    }

    
    int loop_rate = (total_time_ > 1e-3) ? (int)(trajectory_len_ / total_time_) : 100;
    if (loop_rate <= 0) loop_rate = 100; // fallback
    double dt = 1.0 / loop_rate;
    
    rclcpp::Rate rate(loop_rate);

    bool canceled = false;
    unsigned int nj = robot_->getNrJnts();

    while (rclcpp::ok() && t_ < total_time_)
    {
        if (goal_handle->is_canceling()) {
            canceled = true; 
            RCLCPP_WARN(get_logger(), "Trajectory execution canceled by client.");
            break; 
        }

        iteration_++;
        t_ += dt;

        // Aggiorna il punto della traiettoria
        if(traj_type_ == "linear"){
            p_ = (s_type_ == "trapezoidal") ? planner_.linear_traj_trapezoidal(t_) : planner_.linear_traj_cubic(t_);
        } else {
            p_ = (s_type_ == "trapezoidal") ? planner_.circular_traj_trapezoidal(t_) : planner_.circular_traj_cubic(t_);
        }

        // Calcola errore e stato attuale
        KDL::Frame cartpos = robot_->getEEFrame();
        
        
        Eigen::Vector3d error   = computeLinearError(p_.pos, Eigen::Vector3d(cartpos.p.data));
        Eigen::Vector3d o_error = computeOrientationError(toEigen(init_cart_pose_.M), toEigen(cartpos.M));

        // BLOCCO DI CONTROLLO
        
        if(cmd_interface_ == "position"){
                    Eigen::Vector3d p_des_vel = p_.vel + (Kp_ * error);
                    KDL::Frame nextFrame; 
                    nextFrame.M = cartpos.M; 
                    nextFrame.p = cartpos.p + (toKDL(p_des_vel))*dt;


                    // Compute IK
                    joint_positions_cmd_ = joint_positions_;
                    robot_->getInverseKinematics(nextFrame, joint_positions_cmd_);
                }
                else if(cmd_interface_ == "velocity")
                {
                    // VELOCITY CONTROL
                    if(ctrl_ == "velocity_ctrl") // Punto 1.b
                    {

                        Eigen::Matrix<double, 6, 1> cartvel;

                        cartvel << p_.vel + Kp_ * error, o_error;
                        joint_velocities_cmd_.data = pseudoinverse(robot_->getEEJacobian().data) * cartvel;
                        RCLCPP_INFO_THROTTLE(
                        this->get_logger(),
                        *this->get_clock(),
                        1000, // ogni 1 secondo
                        "[TRAJ DEBUG] t=%.2f | pos=(%.3f, %.3f, %.3f) | vel=(%.3f, %.3f, %.3f) | err=%.4f",
                        t_,
                        p_.pos[0], p_.pos[1], p_.pos[2],
                        p_.vel[0], p_.vel[1], p_.vel[2],
                        error.norm()
                    );
                    }
                    else if(ctrl_ == "velocity_ctrl_null") // Punto 1.b
                    {
                        // Null-space velocity control
                        joint_velocities_cmd_.data = controller_->velocity_ctrl_null(
                            p_.pos,
                            Eigen::Vector3d(cartpos.p.data),
                            Kp_);
                    }
                }


                // Update KDLrobot structure
                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

                if(cmd_interface_ == "position"){
                    // Set joint position commands
                    for (unsigned int i = 0; i < nj; ++i) {
                        desired_commands_[i] = joint_positions_cmd_(i);
                    }
                }
                else if(cmd_interface_ == "velocity"){
                    // Set joint velocity commands
                    for (unsigned int i = 0; i < nj; ++i) {
                        desired_commands_[i] = joint_velocities_cmd_(i);
                    }
                }
                else if(cmd_interface_ == "effort"){
                    // Set joint effort commands
                    for (unsigned int i = 0; i < nj; ++i) {
                        desired_commands_[i] = joint_efforts_cmd_(i);
                    }
                } 

        std_msgs::msg::Float64MultiArray cmd_msg;
        cmd_msg.data = desired_commands_;
        cmdPublisher_->publish(cmd_msg);


        feedback->current_error_norm = error.norm();
        goal_handle->publish_feedback(feedback);


        rate.sleep();
    }

    // FASE DI STOP 
    if (canceled) {
        RCLCPP_WARN(this->get_logger(), "Trajectory canceled.");
        // Ferma il robot
        if (cmd_interface_ == "velocity" || cmd_interface_ == "effort") {
            for (auto &val : desired_commands_) val = 0.0;
            std_msgs::msg::Float64MultiArray stop_msg;
            stop_msg.data = desired_commands_;
            cmdPublisher_->publish(stop_msg);
        }
        

        result->success = false;
        result->message = "Trajectory canceled by client.";
        goal_handle->canceled(result);

        return;
    }


    if (cmd_interface_ == "velocity" || cmd_interface_ == "effort") {
        for (auto &val : desired_commands_) val = 0.0;
        std_msgs::msg::Float64MultiArray stop_msg;
        stop_msg.data = desired_commands_;
        cmdPublisher_->publish(stop_msg);
    }

    RCLCPP_INFO(this->get_logger(), "Trajectory executed successfully.");
    

    result->success = true;
    result->message = "Trajectory executed successfully.";
    goal_handle->succeed(result);

    }


    // --- Funzioni Aggiunte Punto 2b --- //

    /**
     * Loop di pubblicazione comandi per il controllore vision-based.
     * Chiamato da un timer solo se ctrl_ == "vision".
     */
    void cmd_publisher_vision(){
            // Verifica che il messaggio ArUco sia valido
            if (!aruco_msg) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "No ArUco message available!");
                return;
            }


            // Aggiorna lo stato KDL del robot con gli ultimi dati dal subscriber

            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

 
            iteration_ = iteration_ + 1;
 
            // Calcola matrice di trasformazione da aruco a image Plane (come da topic)
            KDL::Frame T_imgP_aruco = toKDL(aruco_msg->pose);
 
            // Matrice della camera rispetto all'EE (DA URDF)
            KDL::Frame T_ee_cam = KDL::Frame(
                KDL::Rotation::RPY(3.14, -1.57, 0.0),
                KDL::Vector(0.0, 0.0, 0.17)
            );
            
            // Posa dell'aruco rispetto all'EE
            current_aruco_frame_ee = T_ee_cam*T_imgP_aruco;
 
            // Calcola frame EE attuale (ORA E' AGGIORNATO)
            KDL::Frame cartpos = robot_->getEEFrame();
 
            // Calcola Frame Desiderato (aruco nel frame base)
            KDL::Frame desFrame = cartpos*current_aruco_frame_ee;
 

            // Calcola l'errore di direzione
            Eigen::Vector3d _P_c_o(T_imgP_aruco.p.x(), T_imgP_aruco.p.y(), T_imgP_aruco.p.z());
            double p_norm = _P_c_o.norm();
            Eigen::Vector3d s(0,0,1);
            if (p_norm > 1e-6) {
                s = _P_c_o / p_norm;
            }
            Eigen::Vector3d s_d(0, 0, 1);
            double direction_error_norm = (s - s_d).norm();

            // Calcola errore di posizione
            Eigen::Vector3d error_pos = computeLinearError(
                Eigen::Vector3d(desFrame.p.data),
                Eigen::Vector3d(cartpos.p.data));
            double p_err_norm = (double)error_pos.norm();
 
            t_ ++;
 
            // Soglie di errore
            double epsilon = 0.05; // Soglia minima (0.05 radianti, circa 3 gradi)
            double e_max = 10.0;   // Soglia massima (ignora se marker è troppo lontano/errore pos)
 

            if(direction_error_norm < epsilon || p_err_norm > e_max){
                // Ferma il robot se è allineato o l'errore è troppo grande
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = 0.0;
                }
                RCLCPP_INFO_ONCE(this->get_logger(), "Aruco tag centered or error too large. Stopping.");
            }
            else{
                // Esegui il controllo
                KDL::Chain chain_;
                robot_tree.getChain(
                                    robot_tree.getRootSegment()->first,
                                    "iiwa_link_ee",
                                    chain_);
 
                unsigned int n = robot_->getNrJnts();
 
                double K_gain = 1.0; // Guadagno del controllore
                Eigen::MatrixXd K = Eigen::MatrixXd::Identity(n,n)*K_gain;
 
                // Calcola velocità di giunto
                joint_velocities_cmd_.data = controller_->vision_control(this->aruco_msg,
                                                                        chain_,
                                                                        K);
 
                // Set joint velocity commands
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = joint_velocities_cmd_(i);
                }
 
            }//
            
            // Crea messaggio e pubblica
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);
 
        } //

    // --- Fine Funzioni Punto 2b --- //


    // JointState callback 
    void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg){
        joint_state_available_ = true;
        for (unsigned int i  = 0; i < sensor_msg.position.size() && i < joint_positions_.rows(); i++){
            joint_positions_.data[i] = sensor_msg.position[i];
            if (i < sensor_msg.velocity.size())
                joint_velocities_.data[i] = sensor_msg.velocity[i];
        }
    }

    // --- Callback per Punto 2b --- //
    void aruco_pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg){
            aruco_state_avaliable_ = true;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, // Logga ogni secondo
                "ArucoTag pose received: [%.3f, %.3f, %.3f]",
                msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
            
            aruco_msg = msg; // Memorizza il messaggio
    }
    // --- Fine Callback 2b --- //


    //  Membri 
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
    rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
    rclcpp::Node::SharedPtr node_handle_;
    rclcpp::TimerBase::SharedPtr timer_;
    // rclcpp::TimerBase::SharedPtr subTimer_;

    std::shared_ptr<KDLController> controller_;
    std::shared_ptr<KDLRobot> robot_;
    rclcpp_action::Server<TrajectoryAction>::SharedPtr action_server_;

    std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    KDL::JntArray joint_positions_;
    KDL::JntArray joint_velocities_;
    KDL::JntArray joint_positions_cmd_;
    KDL::JntArray joint_velocities_cmd_;
    KDL::JntArray joint_efforts_cmd_;

    KDLPlanner planner_;
    trajectory_point p_;

    int iteration_;
    bool joint_state_available_;
    double t_;
    
    // Parametri (utilizzati dall'action goal o dal file .yaml)
    double traj_duration_;
    double acc_duration_;
    double total_time_;
    int trajectory_len_;
    double end_pos_x_;
    double end_pos_y_;
    double end_pos_z_;
    double Kp_;

    std::string cmd_interface_;
    std::string traj_type_;
    std::string s_type_;
    std::string ctrl_; // Usato per selezionare velocity_ctrl, velocity_ctrl_null, o vision

    KDL::Frame init_cart_pose_;
    KDL::Tree robot_tree;

    // --- Membri Aggiunti per Punto 2b --- //
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr arucoSubscriber_;
    geometry_msgs::msg::PoseStamped::ConstSharedPtr aruco_msg;
    KDL::Frame current_aruco_frame_ee;
    bool aruco_state_avaliable_ = false;
    // --- Membri 2b --- //
    
};

int main( int argc, char** argv )
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Iiwa_pub_sub>());
    rclcpp::shutdown();
    return 0;
}
