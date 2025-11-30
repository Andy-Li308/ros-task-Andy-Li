#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>

class P_Controller_Node : public rclcpp::Node
{
    public: 
        P_Controller_Node() : Node("P_Controller_Node")
        {
            //create publisher for cmd_vel
            cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
            //create subscriber for odometry
             odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&P_Controller_Node::odomCallback, this, std::placeholders::_1));
            //create timer to call control loop 
            timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&P_Controller_Node::controllerLoop, this));
        }

    private: 
    /* FUNCTIONS */

        //extracts the current x, y, and theta from odometry message
         void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
            current_x_ = msg->pose.pose.position.x;
            current_y_ = msg->pose.pose.position.y;
            
            //extract quaternion 
            double x = msg->pose.pose.orientation.x;
            double y = msg->pose.pose.orientation.y;
            double z = msg->pose.pose.orientation.z;
            double w = msg->pose.pose.orientation.w;

            //get yaw (theta) from quaternion
            current_theta_ = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
            
            //update odom received flag
            if (!odom_received_){
                odom_received_ = true;
            }
         }

         void controllerLoop(){

            if (!odom_received_){//only run the loop once the robot has odom data
                RCLCPP_INFO(this->get_logger(), "Waiting for odometry msg");
                return;
            }

            if (target_pose_reached_){//if target already reached, do nothing
                return;
            }

            //set up message to be filled and published
            geometry_msgs::msg::Twist cmd_vel_msg; 
            double vel_linear; 
            double vel_angular;

            //calculate distance and angle errors
            double error_x = target_x_ - current_x_;
            double error_y = target_y_ - current_y_;
            double distance_error = std::sqrt(error_x * error_x + error_y * error_y); //shouldnt this be able to be negative as well if you overshoot? NO bc thats accounted for by the angular correction (angle to target)

            double angle_to_target = std::atan2(error_y, error_x); //required angle to turn to in order to head in right direction to target position

            //make sure that the angle to target is in [-pi, pi] to avoid large rotations
            if(angle_to_target > M_PI || angle_to_target < -M_PI){
                while (angle_to_target >= M_PI) angle_to_target -= 2 * M_PI;
                while (angle_to_target <= -M_PI) angle_to_target += 2 * M_PI;
            }

            if (distance_error <= target_tolerance_){//robot is at correct x and y position
                vel_linear = 0.0; //stop linear movement

                if (std::abs(current_theta_ - target_theta_) > target_angle_tolerance_){ //not at correct orientation yet, need to rotate 
                    vel_angular = Kp_angular_ * (target_theta_ - current_theta_);
                    RCLCPP_INFO(this->get_logger(), "Final Rotation Phase: Current Theta: %.2f, Target Theta: %.2f, Angular Velocity: %.2f", current_theta_, target_theta_, vel_angular);
                }

                else { //robot is at correct pose, finished 
                    vel_angular = 0.0; //stop angular movement
                    RCLCPP_INFO(this->get_logger(), "Target Reached"); 
                    target_pose_reached_ = true;
                }
            }

            else{ //robot is NOT at correct x and y position

                if (std::abs(current_theta_ - angle_to_target) > target_angle_tolerance_){ //robot is not at the right spot AND not facing right direction, perform rotation to get robot on the correct direction to start moving towards target 
                    vel_linear = 0.0; //robot should only rotate 
                    vel_angular = Kp_angular_ * (angle_to_target - current_theta_);

                    driving_phase_ = false;
                    RCLCPP_INFO(this->get_logger(), "Initial Rotation Phase: Current Theta: %.2f, Target Theta: %.2f, Angular Velocity: %.2f", current_theta_, angle_to_target, vel_angular);
                }

                else{//robot is not at the right spot but IS facing right direction, drive towards target coords
                    vel_linear = Kp_linear_ * distance_error;
                    vel_angular = Kp_angle_trim_ * (angle_to_target - current_theta_); //small angle corrections while driving forward

                    RCLCPP_INFO(this->get_logger(), "Driving Phase: Current X: %.2f, Current Y: %.2f, Target X: %.2f, Target Y: %.2f, Linear Velocity: %.2f, Angular Velocity: %.2f", current_x_, current_y_, target_x_, target_y_, vel_linear, vel_angular);

                }
            }

            //publish twist message
            cmd_vel_msg.linear.x = vel_linear;
            cmd_vel_msg.angular.z = vel_angular;
            cmd_vel_pub_->publish(cmd_vel_msg);
            //show current stats for logging
            RCLCPP_INFO(this->get_logger(), "Current Position:  X= %.2f, Y= %.2f, Theta= %.2f", current_x_, current_y_, current_theta_);
         }

        /* VARIABLES */
    
        // Publishers and Subscribers
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_; 

        //timer
        rclcpp::TimerBase::SharedPtr timer_;

        // Control parameters
        double Kp_linear_ = 1.0;
        double Kp_angular_ = 1.0; //for large rotations
        double Kp_angle_trim_ = 0.5; //for small angle corrections while driving forward

        //target pose
        double target_x_ = 5.0;
        double target_y_ = -5.0;
        double target_theta_ = M_PI / 2;

        //current pose
        double current_x_ = 0; 
        double current_y_ = 0;
        double current_theta_ = 0; //dummy values, these will be updated as soon as odom messages are received

        //tolerances
        double target_tolerance_ = 0.01; //tolerance for distance to target, need to eventually get this down to 5cm
        double target_angle_tolerance_ = 0.05;  //tolerance for target angle

        //status flags 
        bool odom_received_ = false; 
        bool target_pose_reached_ = false; 

        //bruh these are all highkey useless 
        bool initial_rotation_phase_ = false; 
        bool driving_phase_ = false;
        bool final_rotation_phase_ = false;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<P_Controller_Node>());
    rclcpp::shutdown();
    return 0;
}