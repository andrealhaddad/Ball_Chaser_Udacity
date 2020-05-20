#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
//#include <iostream>
#include <string>
#include <vector>
#include <numeric>
using namespace std;

// Define a global client that can request services
ros::ServiceClient client;
float Current_vel_x, Current_ang_z, set_vel_x, set_ang_z, distance_near= 0, angle_near = 0;
bool Stuck_flag = false;
int Stuck_counter= 0;

bool check_if_stuck(float lin_x, float ang_z){
    if (lin_x >0.05){
        if ( (abs(lin_x - Current_vel_x) > lin_x*0.8 )&& (distance_near < 0,1)){
            ROS_INFO_STREAM("I might be stuck!!! Cannot move forwards " + to_string(Stuck_flag));
            return true;
        }
        else if(abs(lin_x - Current_vel_x) < lin_x*0.4 ){
            Stuck_flag = false;
            Stuck_counter= 0;
            ROS_INFO_STREAM("Not stuck anymore " + to_string(Stuck_flag));

            return false;
        }
        if ((set_ang_z > 0.05) && (abs(ang_z - Current_ang_z) > ang_z*0.5) && (distance_near < 0,1)){
            ROS_INFO_STREAM("I might be stuck!!! Cannot Turn " + to_string(Stuck_flag));
            return true;
        }
    }
    return false;
}
// This function calls the command_robot service to drive the robot in the specified direction
bool drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    string ss;
    //ss = "Requested motion: Linear_x : " + to_string(lin_x) + ",  angular z: "+ to_string(ang_z);
    //ROS_INFO_STREAM(ss);

    ball_chaser::DriveToTarget srv;
    srv.request.angular_z = ang_z;
    srv.request.linear_x = lin_x;

    if (!client.call(srv) ){
        ROS_ERROR("Failed call service");}
    

    if (check_if_stuck(lin_x, ang_z) == true){
        ros::Duration(0.4).sleep();
        //check_again:
        if (check_if_stuck(lin_x, ang_z) == true){
            Stuck_counter +=1;
        }
        else{Stuck_counter = 0;}
    }
    return check_if_stuck(lin_x, ang_z);
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
    if (counter > 650 && distance_near < 0.3){
        return true;}
    else{
        return false;
    }

}
void rescue_procedure(int dir){
    ROS_INFO_STREAM("//Rescue starting ////////" );
    set_vel_x = -0.1;
    set_ang_z = dir*0.2;
    Stuck_flag = drive_robot(set_vel_x, set_ang_z);
    ros::Duration(2).sleep();
    set_vel_x = 0.25;
    set_ang_z = dir*0.0;
    Stuck_flag = drive_robot(set_vel_x, set_ang_z);
    ros::Duration(2).sleep();
    set_vel_x = 0.0;
    set_ang_z = -0.2*dir;
    Stuck_flag = drive_robot(set_vel_x, set_ang_z);
    ros::Duration(2).sleep();
    ROS_INFO_STREAM("///Resque done");
}
// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    string ss;
    int white_pixel = 255, CM, width, pix = 0;
    vector<int> ball_val;
    vector<uint8_t> img_T;
    vector<uint8_t> bin_x(img.width, 0), img_bin(img.width,0);
    int bin_x_sum = 0;
    int Im_cent = 400, Im_cent1 = 350 , Im_cent2 = 450;
    bool f;
    float vel_str ;
    

    //Create total intesity image (mean value of all three colors)
    //I prefere creating a 2D image to work with, for more flexibility.
    for (int i =0; i < img.height*img.step; i+=3 ){
        pix =(img.data[i]+img.data[i+1]+img.data[i+2])/3;
        img_T.push_back(pix);
    }

    //Look at the tot_image for white ball
    for(int i =0; i < img_T.size(); i += 1){
        //img.height*img.step
        if(img_T[i] == white_pixel){
            bin_x[i%img.width] += 1;
        }
        img_bin[i%img.width] += img_T[i];
    }
    
    for (int i =0; i< bin_x.size(); i++){
        bin_x_sum += bin_x[i];
    }

    if (bin_x_sum == 0){
        //No white ball:  STOP
        ss = "No Ball detected: Stopping motion";
        //Check ODOM values:

        if ((Current_vel_x > 0.000) || (Current_ang_z > 0.0000)){
            set_vel_x = 0.0;
            set_ang_z = 0.0;
            Stuck_flag = drive_robot(set_vel_x, set_ang_z);
        }

        //ROS_INFO_STREAM(ss);
        }
    else{
        
        ball_val = ball_info(bin_x);
        CM= ball_val[0];
        width = ball_val[1]      ;

        if (CM < Im_cent1){
            ss = "CM = "+ to_string(CM) + "  ball to the left";
            set_vel_x = 0.0;
            set_ang_z = 0.15;
            Stuck_flag = drive_robot(set_vel_x, set_ang_z);
        }
        else if(CM > Im_cent2){
            ss = "CM = "+ to_string(CM) + "  Ball to the right";
            set_vel_x = 0.0;
            set_ang_z = -0.15;
            Stuck_flag = drive_robot(set_vel_x, set_ang_z);

        }
        else{
            f = hit_ball(bin_x);
            if (f == false){
                vel_str = 0.25;
                if ( width  > 200){vel_str = 1.0*(20.0/width);}
                set_vel_x = vel_str;
                set_ang_z = 0.0;
                Stuck_flag = drive_robot(set_vel_x, set_ang_z);
                ss = "Ball close to center, CM:" + to_string(CM) + "   Distnace to perfect is  "+ to_string(distance_near) ;
            }
            else{
                set_vel_x = 0.0;
                set_ang_z = 0.0;
                Stuck_flag = drive_robot(set_vel_x, set_ang_z);
                ros::Duration(2).sleep();
                Stuck_flag = false;
                Stuck_counter = 0;
                ss = "/////////   Reached ball. Horraayyy!!!    /////////";
                ROS_INFO_STREAM(ss);
            }

        }
        //ROS_INFO_STREAM(ss);
    }
    if (Stuck_flag == true && Stuck_counter > 15){
        //Check image to see wall!
        
        //distance_near
        if (angle_near > 0){
            //ROS_INFO_STREAM("Wall to the right");
            rescue_procedure(-1);}
        else{
            //ROS_INFO_STREAM("Wall to the Left");
            rescue_procedure(1);}
        //initiate rescue
        
        Stuck_counter = 0;
    }

}


void process_odom(const nav_msgs::Odometry Odom){
    string ss;
    Current_vel_x = Odom.twist.twist.linear.x;
    Current_ang_z = Odom.twist.twist.angular.z;
    //ss = "Current Odom = linX: "+ to_string(Current_vel_x)+ " / angZ: "+ to_string(Current_ang_z);
    //ROS_INFO_STREAM(ss);

}

void process_Laser_Scan(const sensor_msgs::LaserScan scan){
    float angle_min, angle_max, angle_increment= 0, range_min, range_max, angle= 0;
    vector<float> ranges, angles; //this detector has not intensities measured.
    float close_range, close_angle;
    int index_close;
    string ss;
    
    angle_increment = scan.angle_increment;
    angle_min = scan.angle_min;
    angle_max = scan.angle_max;
    range_min = scan.range_min;
    range_max = scan.range_max;
    ranges = scan.ranges;
    angle = angle_min;
    for (int i =0; i<ranges.size(); i++){
        angle +=  angle_increment;
        angles.push_back(angle);
    }

    // Clean out of range values
    /*for (int i =0; i<ranges.size(); i++){
        if ( ranges[i]  < range_min || ranges[i]> range_max){
            ranges[i] = range_max;//I care about the near ones. Ideally erase value
            angles[i] = range_max;
        }
    }*/

    index_close = min_element(ranges.begin(), ranges.end()) - ranges.begin();
    distance_near = ranges[index_close];
    angle_near = angles[index_close];

    ss= "CLose by : Dist = " + to_string(distance_near) + "//Angle = " + to_string(angle_near);
    ROS_INFO_STREAM(ss);
    
}


int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /odom Odometer to get real velocity
    ros::Subscriber sub2 = n.subscribe("/odom", 20, process_odom);

    //Subscrive to Laser scan measurement
    ros::Subscriber sub3 = n.subscribe("/scan", 10, process_Laser_Scan);

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
