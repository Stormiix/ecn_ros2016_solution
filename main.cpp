#include <ecn_common/color_detector.h>
#include <ros/ros.h>

// TODO include files for considered messages

using std::cout;
using std::endl;

// some global variables to be used in callbacks
cv::Mat im;
bool im_received;
std::string color_to_detect;


int main(int argc, char** argv)
{
    // declare node and loop rate at 10 Hz
    ros::init(argc, argv, "control_node");
    ros::NodeHandle nh("~"), nhg;
    ros::Rate loop(10);
    
    

    // default values
    im_received = false;
    color_to_detect = "";

    // TODO get color parameter for this node
    std::string color;

    // TODO get corresponding RGB color
    int r,g,b;

    // init color detector for this (R,G,B) color
    ecn::ColorDetector cd(r,g,b);
    cd.setSaturationValue(165, 165);
    cv::Mat im_processed;

    // TODO init other things before control loop
    // publishers, subscribers...

    double X, Z;

    while(ros::ok())
    {
        if(im_received) // TODO also check that the color to detect is the one this node detects
        {
            // process image
            cd.process(im, im_processed);
            

            // TODO
            // control robot from X and Z
            if(X < 0)
            {


            }
            else
            {


            }



            // publish




        }

        // spin and wait
        ros::spinOnce();
        loop.sleep();
    }
}
