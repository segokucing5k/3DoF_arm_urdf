#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <kdl/frames.hpp>
#include <kdl/path.hpp>
#include <kdl/path_circle.hpp>
#include <kdl/path_composite.hpp>
#include <kdl/path_line.hpp>
#include <kdl/path_point.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/velocityprofile_rect.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <vector>

double l1 = 10.0; // length of the first link
double l2 = 10.0; // length of the second link

// double theta1, theta2;
// double x,y,z;

std::vector<double> fk(double theta1, double theta2){
    // double _theta1 = start_deg[1];
    // _theta2 = start_deg[2];

    double x = l1 * cos(theta1) + l2 * cos(theta1 + theta2);
    double y = l1 * sin(theta1) + l2 * sin(theta1 + theta2);

    std::vector cartesian = {x, y};
    return cartesian;
}

std::vector<double> ik(double x, double y, double z){
    // double x = position.p.x();
    // double y = position.p.y();

    double cos_theta2 = (x*x + y*y - l1*l1 - l2*l2) / (2 * l1 * l2);
    double sin_theta2 = sqrt(1 - cos_theta2 * cos_theta2); // assuming the elbow-up configuration

    double theta2 = atan2(sin_theta2, cos_theta2);

    double k1 = l1 + l2 * cos_theta2;
    double k2 = l2 * sin_theta2;

    double theta1 = atan2(y, x) - atan2(k2, k1);
    std::vector<double> theta = std::vector<double>{theta1, theta2};
    return theta;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "joint_state_publisher");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10); 
    
    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(3);
    joint_state.position.resize(joint_state.name.size());
    joint_state.name = {"hip", "shoulder", "elbow"}; 

    std::vector<double> start_deg = {0,0};
    std::vector<double> start_cartesian = fk(start_deg[0], start_deg[1]);

    std::vector<double> stop_deg = {30,30};
    std::vector<double> stop_cartesian = fk(stop_deg[0], stop_deg[1]);

    KDL::Frame start_pos(KDL::Vector(start_cartesian[0], start_cartesian[1], 0));
    KDL::Frame stop_pos(KDL::Vector(stop_cartesian[0], stop_cartesian[1], 0));

    KDL::Path_Line* line = new KDL::Path_Line(start_pos, stop_pos, new KDL::RotationalInterpolation_SingleAxis(), 1);
    KDL::VelocityProfile_Trap* velocity_profile = new KDL::VelocityProfile_Trap(1, 1);
    double desired_duration = 10.0; // Adjust the desired duration here
    velocity_profile->SetProfileDuration(0, line->PathLength(), desired_duration);
    KDL::Trajectory_Segment* trajectory = new KDL::Trajectory_Segment(line, velocity_profile);
    double total_time = trajectory->Duration();
    double time_step = 0.1;

    // double desired_duration = 5.0; // Adjust the desired duration here
    // velocity_profile->SetProfileDuration(0, line->PathLength(), desired_duration);
    // KDL::Trajectory_Segment* trajectory = new KDL::Trajectory_Segment(line, velocity_profile);
    // double total_time = trajectory->Duration();
    while(ros::ok()){
        for (double time = 0; time <= total_time; time += time_step) {
            KDL::Frame position = trajectory->Pos(time);
            std::vector<double> theta = ik(position.p.x(), position.p.y(), position.p.z());

            // printf("Joint Angles: %f %f\n", theta1, theta2);
            // printf("Position: %f %f %f\n", position.p.x(), position.p.y(), position.p.z());
            joint_state.position = {0, theta[0], theta[1]}; //in radians | hip, shoulder, elbow
            joint_state.header.stamp = ros::Time::now();
            joint_state_pub.publish(joint_state);   
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    // delete line;
    // delete velocity_profile;
    // delete trajectory;

    return 0;
}


