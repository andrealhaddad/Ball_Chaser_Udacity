#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
//#include <iostream>
#include <string>
#include <vector>
#include <numeric>
using namespace std;

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    string ss;
    ss = "Requested motion: Linear_x : " + to_string(lin_x) + ",  angular z: "+ to_string(ang_z);
    ROS_INFO_STREAM(ss);

    ball_chaser::DriveToTarget srv;
    srv.request.angular_z = ang_z;
    srv.request.linear_x = lin_x;

    if (!client.call(srv) )
        ROS_ERROR("Failed call service");
}

vector<int> ball_info(vector<uint8_t> data_x)
{
    float mx=0 , m =0;
    vector<int> ball;
    int width = 0;

    for (int i =0; i < data_x.size(); i++){
        mx += data_x[i]*i;
        m += data_x[i];
        if(data_x[i] > 0 ){
            width += 1;
        }
    }

    ball.push_back( (int) mx/m);
    ball.push_back(width);
    return ball;
}


bool hit_ball(vector<uint8_t> data_x)
{   
    int counter = 0;
    for (int i = 0; i< data_x.size(); i++){
            if( data_x[i]>0){
                counter +=1;
            }
    }
    if (counter > 650){
        return true;}
    else{
        return false;
    }

}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    string ss;
    int white_pixel = 255, CM, width;
    vector<int> ball_val;
    vector<uint8_t> img_R;
    vector<uint8_t> bin_x(img.width, 0);
    int bin_x_sum = 0;
    int Im_cent = 400, Im_cent1 = 350, Im_cent2 = 450;
    bool f;
    float vel_str ;

    //Create red image only
    for (int i =0; i < img.height*img.step; i+=3 ){
        img_R.push_back(img.data[i]);
    }

    //Look at only Red (white has 255 in all colors)
    for(int i =0; i < img_R.size(); i += 1){
        //img.height*img.step
        if(img_R[i] == white_pixel){
            bin_x[i%img.height] += 1;
        }
        
    }
    
    for (int i =0; i< bin_x.size(); i++){
        bin_x_sum += bin_x[i];
    }

    if (bin_x_sum == 0){
        //No white ball:  STOP
        ss = "No Ball detected: Stopping motion";
        drive_robot(0.0, 0.0);
        ROS_INFO_STREAM(ss);
        }
    else{
        
        ball_val = ball_info(bin_x);
        CM= ball_val[0];
        width = ball_val[1]      ;

        if (CM < Im_cent1){
            ss = "CM = "+ to_string(CM) + "  ball to the left";
            drive_robot(0.0, 0.1);
        }
        else if(CM > Im_cent2){
            ss = "CM = "+ to_string(CM) + "  Ball to the right";
            drive_robot(0.0, -0.1);

        }
        else{
            f = hit_ball(bin_x);
            if (f == false){
                vel_str = 0.2;
                if (width >400){vel_str = 1.0*(10.0/width);}
                drive_robot(vel_str, 0.0);
                ss = "Ball close to center, CM:" + to_string(CM) + "   Distnace to perfect is  "+ to_string(CM-Im_cent);
            }
            else{
                drive_robot(0.0, 0.0);
                ss = "Reached ball. Horraayyy!!!";
            }

        }
        ROS_INFO_STREAM(ss);
        }
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
