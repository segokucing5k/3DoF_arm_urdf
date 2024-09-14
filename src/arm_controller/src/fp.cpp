#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

float shoulder, elbow, wrist;

float radToDeg(float radians) {
    return radians * (180.0 / M_PI);
}

double degToRad(double degrees) {
    return degrees * (M_PI / 180.0);
}

void inverse_kinematics(float distance){
    float torso, upper_arm, forearm, hand;
    torso = 125; //in mm
    upper_arm = 100;
    forearm = 100;
    hand = 40;

    float a, b, c, d, e, f;
    float diagonal1 = sqrt(pow(torso, 2) + pow(upper_arm, 2));
    float diagonal2 = sqrt(pow(upper_arm, 2) + pow(forearm, 2));
    b = asin(sin(90) * upper_arm / diagonal1);
    e = acos((pow(upper_arm, 2) + pow(forearm, 2) - pow(diagonal2, 2)) / (2 * upper_arm * forearm));
    c = degToRad((180 - radToDeg(e))/2);

    shoulder = b + c;
    elbow = e;
    wrist = e;
}

/*
TOrso dipendekin
divided = a, b, c
distance 
!!!!! fp dulu masih pake duty cycle brooo !!!!
*/

int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, "joint_state_publisher");
    ros::NodeHandle nh;

    // Publisher for the joint states
    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
    
    ros::Rate loop_rate(10); // Set the loop rate to 10 Hz
    
    float distance;

    while(ros::ok()){
        // Create a JointState message
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();

        joint_state.name.resize(4); // Resize the name array
        joint_state.position.resize(joint_state.name.size());
        
                            //hip, shoulder, elbow, wrist
        joint_state.name = {"servo1_joint", "servo2_joint", "servo3_joint", "servo4_joint"}; //initialize joint names, should match the URDF file

        printf("Enter the distance in centimeters: ");
        scanf("%f", &distance);
        inverse_kinematics(distance*10);

        printf("%f, %f, %f\n", shoulder, elbow, wrist);
        joint_state.position = {0, shoulder, elbow, wrist}; // Set the joint positions
        joint_state_pub.publish(joint_state); // Publish the joint state message

        ros::spinOnce(); // Process a single round of ROS callbacks

        loop_rate.sleep(); // Sleep for the remaining time to let us run at 10 Hz
    }
    return 0;
}
