#include <ecn_common/color_detector.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <math.h>

using std::cout;
using std::endl;

// some global variables to be used in callbacks
cv::Mat im, im2;
bool im_received;
std::string color_to_detect;
std::vector<std::string> jointNames = {"Baxter_leftArm_joint1", "Baxter_leftArm_joint6", "Baxter_rightArm_joint1", "Baxter_rightArm_joint6"};

void getFrame(const sensor_msgs::Image &frame)
{
    try
    {
        im = cv_bridge::toCvCopy(frame, "bgr8")->image;
        im_received = true;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void getColor(const std_msgs::String &color)
{
    color_to_detect = color.data.c_str();
}

int main(int argc, char **argv)
{
    // declare node and loop rate at 10 Hz
    ros::init(argc, argv, "control_node");
    ros::NodeHandle nh("~"), nhg;
    ros::Rate loop(10);

    // default values
    im_received = false;
    color_to_detect = "blue";

    //  get color parameter for this node

    std::string color = "blue";
    nh.getParam("color", color);

    //  get corresponding RGB color
    int r, g, b;

    nh.getParam("/colors/" + color + "/r", r);
    nh.getParam("/colors/" + color + "/g", g);
    nh.getParam("/colors/" + color + "/b", b);

    // init color detector for this (R,G,B) color
    ecn::ColorDetector cd(r, g, b);

    cd.setSaturationValue(165, 165);

    cv::Mat im_processed;

    ros::Publisher cmd_pub = nh.advertise<sensor_msgs::JointState>("/vrep_ros_interface/joint_command", 1);
    ros::Publisher imagePub = nh.advertise<sensor_msgs::Image>("/processed_image", 1);
    ros::Subscriber subFrame = nh.subscribe("/image", 1, getFrame);
    ros::Subscriber subColor = nh.subscribe("/color_to_detect", 1, getColor);
    sensor_msgs::JointState cmd;

    cmd.name = jointNames;
    cmd.position.resize(4, 0);

    double X, Z;

    while (ros::ok())
    {
        if (im_received && color == color_to_detect) //  also check that the color to detect is the one this node detects
        {
            // process image
            cd.process(im, im_processed);

            Z = 0.106 / sqrt(cd.area());
            X = cd.x() * Z;

            // control robot from X and Z
            if (X < 0)
            {
                cmd.position[0] = 0;
                cmd.position[1] = -M_PI / 2;
                cmd.position[2] = M_PI / 4 - atan2(-X - 0.25, Z);
                cmd.position[3] = 0;
            }
            else
            {
                cmd.position[0] = -M_PI / 4 + atan2(X - 0.25, Z);
                cmd.position[1] = 0;
                cmd.position[2] = 0;
                cmd.position[3] = -M_PI / 2;
            }

            // publish
            cmd_pub.publish(cmd);
            cv::cvtColor(im_processed, im2, cv::COLOR_BGR2RGB);
            imagePub.publish(cv_bridge::CvImage(std_msgs::Header(), "rgb8", im2).toImageMsg());
        }

        // spin and wait
        loop.sleep();
        ros::spinOnce();
    }
}
