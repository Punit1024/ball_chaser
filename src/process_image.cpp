        #include "ros/ros.h"
        #include "ball_chaser/DriveToTarget.h"
        #include <sensor_msgs/Image.h>
        #include <image_transport/image_transport.h>
        #include <cv_bridge/cv_bridge.h>
        #include <sensor_msgs/image_encodings.h>
        #include "opencv2/imgproc/imgproc.hpp"
        #include "opencv2/highgui/highgui.hpp"
        #include "opencv2/opencv.hpp"

        // Define a global client that can request services
    ros::ServiceClient client;
    enum Dirc { None, Center, Left, Right, Reached };
    float prev_lx;
    float prev_az;
    const float LINEAR_PROPOTIONAL = 0.1;
    const float ANGULAR_PROPOTIONAL = 0.1;
        // Set up detector with params
    cv::Ptr<cv::SimpleBlobDetector> detector;

    void init_blob_detector(){
            // Setup SimpleBlobDetector parameters.
        cv::SimpleBlobDetector::Params params;

            // Change thresholds
        params.minThreshold = 0;
        params.maxThreshold = 255;

            // Filter by Area.
        params.filterByArea = true;
        params.minArea = 100;
        params.maxArea = 500000;

            // Filter by Circularity
        params.filterByCircularity = true;
        params.minCircularity = 0.70;

            // Filter by Convexity
        params.filterByConvexity = true;
        params.minConvexity = 0.88;

            // Filter by Inertia
        params.filterByInertia = true;
        params.minInertiaRatio = 0.65;

        detector  = cv::SimpleBlobDetector::create(params);
    }

    class BallFinder
    {
      ros::NodeHandle nh_;
      image_transport::ImageTransport it_;
      image_transport::Subscriber image_sub_;

    public:
      BallFinder()
      : it_(nh_)
      {
            // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1,
          &BallFinder::imageCb, this);
            // Define a client service capable of requesting services from command_robot
        client = nh_.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/DriveToTarget");


    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
          cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
 

      cv::Mat greyMat;
      cv::cvtColor(cv_ptr->image, greyMat, cv::COLOR_BGR2GRAY);
      cv::Mat invGreyMat = cv::Scalar::all(255) - greyMat;;

            // Storage for blobs
      std::vector<cv::KeyPoint> keypoints;

            // Detect blobs
      detector->detect( invGreyMat, keypoints);
      if(keypoints.size()==0){
        detector->detect( greyMat, keypoints);
    }
    if(keypoints.size()!=0){
        findVelocity(keypoints[0].pt.x,greyMat.size().width ,true);
    }else{findVelocity((float)0,greyMat.size().width,false);}
         }
      catch (cv_bridge::Exception& e)
      {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
      }
    }


    void findVelocity(float location,int width, bool ball_found){
      Dirc ball_location = None;
      if(ball_found){
          if(location < width*0.45){
            ROS_INFO_STREAM("ball is on Left");
            ball_location = Left;
        }
        else if(location > 0.55*width){
            ROS_INFO_STREAM("ball is on Right");
            ball_location = Right;
        }
        else{
            ROS_INFO_STREAM("Ball is in center");
            ball_location = Center;

        }
        }

        switch(ball_location)
        {
            case None  :
            drive_robot(0,0);
            break;

            case Center:
            drive_robot(0.3,0);
            break;

            case Left:
            drive_robot(0.0,0.2);
            break;


            case Right :
            drive_robot(0.0,-0.2);
            break;

            case Reached :
            drive_robot(0,0);
            break;

            default:
            drive_robot(0,0);
            break;
        }


    }
    void drive_robot(float lin_x, float ang_z)
    {
            // TODO: Request a service and pass the velocities to it to drive the robot
            //ROS_INFO_STREAM("Transmitting move command");
        if(lin_x!= prev_lx || ang_z != prev_az){
            prev_lx = lin_x;
            prev_az = ang_z;
            ball_chaser::DriveToTarget srv;
            srv.request.linear_x = lin_x;
            srv.request.angular_z = ang_z;

            // Call the safe_move service and pass the requested joint angles
            if (!client.call(srv))
                ROS_ERROR("Failed to call service DriveToTarget");
            // ROS_INFO_STREAM("Transmitted");
        }

    }


    };




        // This function calls the command_robot service to drive the robot in the specified direction
    

    int main(int argc, char** argv)
    {


        prev_lx = 0;
        prev_az = 0;
            // Initialize the process_image node and create a handle to it
        ros::init(argc, argv, "process_image");
        ros::NodeHandle n;
        init_blob_detector();
        BallFinder bf;
        ros::spin();

        return 0;
    }
