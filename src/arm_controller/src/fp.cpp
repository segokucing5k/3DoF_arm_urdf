#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

float shoulder, elbow, wrist, finger;
float shoulder_offset = 90; 
float elbow_offset = 100; 
float wrist_offset = 100; 

float radToDeg(float radians) {
    return radians * (180.0 / M_PI);
}

double degToRad(double degrees) {
    return degrees * (M_PI / 180.0);
}

// void inverse_kinematics(float distance){
//     float torso, upper_arm, forearm, hand;
//     torso = 125; //in mm
//     upper_arm = 100;
//     forearm = 100;
//     hand = 40;
//     float a, b, c, d, e, f;
//     float diagonal1 = sqrt(pow(torso, 2) + pow(upper_arm, 2));
//     float diagonal2 = sqrt(pow(upper_arm, 2) + pow(forearm, 2));
//     b = asin(sin(90) * upper_arm / diagonal1);
//     e = acos((pow(upper_arm, 2) + pow(forearm, 2) - pow(diagonal2, 2)) / (2 * upper_arm * forearm));
//     c = degToRad((180 - radToDeg(e))/2);
//     shoulder = b + c;
//     elbow = e;
//     wrist = e;
// }

void inverse_kinematics(float distance){
    float l1 = 12.5, l2 = 10, l3 = 10, l4 = 14.5, l5 = 4;
    float a, b, c, a1, a2, b1, b2, x, c2;
    float t2, t3, t4;
    
    x = distance - 4;

    c = std::sqrt(l1 * l1 + x * x);

    c2 = radToDeg(std::acos((c * c - (l2 * l2 + l3 * l3)) / (2 * l2 * l3)));
    a2 = (180 - c2) / 2;
    b2 = a2;

    b1 = radToDeg(std::asin((l1 * std::sin(0.5)) / c));
    a1 = 180 - 90 - b1;

    t2 = a1 + a2;
    t3 = c2;
    t4 = 180 - b1 - b2;

    shoulder = degToRad(t2);
    elbow = degToRad(t3);
    wrist = degToRad(t4);
}

int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, "joint_state_publisher");
    ros::NodeHandle nh;

    // Publisher for the joint states
    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
    
    ros::Rate loop_rate(10); // Set the loop rate to 10 Hz
    
    float distance;

    while(ros::ok()){
        sensor_msgs::JointState joint_state; // Create a joint state message
        joint_state.header.stamp = ros::Time::now();

        joint_state.name.resize(6); // Resize the name array
        joint_state.position.resize(joint_state.name.size());
        
                            //hip, shoulder, elbow, wrist, right, left
        joint_state.name = {"servo1_joint", "servo2_joint", "servo3_joint", "servo4_joint", "servo5_joint", "servo6_joint"}; //initialize joint names, should match the URDF file

        printf("Enter the distance: ");
        scanf("%f", &distance);
        inverse_kinematics(distance);

        shoulder = degToRad(radToDeg(shoulder) - shoulder_offset); 
        elbow = degToRad(radToDeg(elbow) - elbow_offset);
        wrist = degToRad(radToDeg(wrist) - wrist_offset);

        // printf("%f, %f, %f\n", radToDeg(shoulder), radToDeg(elbow), radToDeg(wrist));
        joint_state.position = {0, shoulder, elbow, wrist, 0, 0}; // Set the joint positions
        joint_state_pub.publish(joint_state); // Publish the joint state message

        ros::spinOnce(); // Process a single round of ROS callbacks
        loop_rate.sleep(); // Sleep for the remaining time to let us run at 10 Hz
    }
    return 0;
}
