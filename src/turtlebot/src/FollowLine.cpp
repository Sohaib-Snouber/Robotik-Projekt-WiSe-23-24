#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

ros::Publisher cmdVelPublisher;


int height = 720;
int width = 960;
int squareWidth = width / 13; // war 12
int squareHeight = height / 20; 
int center_box = width / 2 - squareWidth / 2;
int left_box = center_box - squareWidth;
int right_box = center_box + squareWidth;
int left_left_box = center_box - 2*squareWidth;
int right_right_box = center_box + 2*squareWidth;

// Variablen, um die durchschnittliche Intensität jedes Kastens zu speichern
double averageLeftBox = 0.0;
double averageCenterBox = 0.0;
double averageRightBox = 0.0;
double averageLeftLeftBox = 0.0;
double averageRightRightBox = 0.0;
double positive_difference;
double all_average;
double umgebung_licht;
double twist;

double tolerance = 5.0; // Toleranzwert



void binaryImageCallback(const sensor_msgs::ImageConstPtr& msg) {
    ROS_INFO("Binary image callback called");
    
    // Konvertiere ROS-Bildnachricht in OpenCV-Bild
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    // Berechne die Anzahl der Pixel pro Meter
    double pixelsPerMeter = (60.0/40.0)*1000.0;
    const double PI = 3.14159265358979323846;

    // Extrahiere Regionen von Interesse (ROIs) für jeden Kasten
    cv::Mat leftBoxROI = cv_ptr->image(cv::Rect(left_box, height - (squareHeight+5), squareWidth, squareHeight));
    cv::Mat centerBoxROI = cv_ptr->image(cv::Rect(center_box, height - (squareHeight+5), squareWidth, squareHeight));
    cv::Mat rightBoxROI = cv_ptr->image(cv::Rect(right_box, height - (squareHeight+5), squareWidth, squareHeight));
    cv::Mat leftleftBoxROI = cv_ptr->image(cv::Rect(left_left_box, height - (squareHeight+5), squareWidth, squareHeight));
    cv::Mat rightrightBoxROI = cv_ptr->image(cv::Rect(right_right_box, height - (squareHeight+5), squareWidth, squareHeight));

    
    // Zeichne Rechtecke um die ROIs auf dem Ausgabebild
    cv::rectangle(cv_ptr->image, cv::Rect(left_box, height - (squareHeight + 5), squareWidth, squareHeight), cv::Scalar(0), 1);
    cv::rectangle(cv_ptr->image, cv::Rect(center_box, height - (squareHeight + 5), squareWidth, squareHeight), cv::Scalar(0), 1);
    cv::rectangle(cv_ptr->image, cv::Rect(right_box, height - (squareHeight + 5), squareWidth, squareHeight), cv::Scalar(0), 1);
    cv::rectangle(cv_ptr->image, cv::Rect(left_left_box, height - (squareHeight + 5), squareWidth, squareHeight), cv::Scalar(0), 1);
    cv::rectangle(cv_ptr->image, cv::Rect(right_right_box, height - (squareHeight + 5), squareWidth, squareHeight), cv::Scalar(0), 1);


    
    
    // Berechne die durchschnittliche Intensität jedes Kastens
    averageLeftBox = cv::mean(leftBoxROI)[0];
    averageCenterBox = cv::mean(centerBoxROI)[0];
    averageRightBox = cv::mean(rightBoxROI)[0];
    averageLeftLeftBox = cv::mean(leftleftBoxROI)[0];
    averageRightRightBox = cv::mean(rightrightBoxROI)[0];
    all_average = (averageLeftBox + averageCenterBox + averageRightBox)/3;
    umgebung_licht = all_average/255;
    
    // Drucke die Durchschnitte
    ROS_INFO("right right average Value: %f", averageRightRightBox);
    ROS_INFO("left average Value: %f", averageLeftBox);
    ROS_INFO("center average Value: %f", averageCenterBox);
    ROS_INFO("right average Value: %f", averageRightBox);
    ROS_INFO("left left average Value: %f", averageLeftLeftBox);



    // 0.22rad/sec ist die max Geschwindigkeit für beide lineare und Winkelgeschwindigkeiten.
    // +0.22 Winkelgeschwindigkeit ist links, und -0.22 ist rechts
    geometry_msgs::Twist twistMsg;

    twistMsg.linear.x = ((255-averageCenterBox) /2000) * umgebung_licht; 
    twist =(averageRightBox - averageLeftBox)/ 800;
    twistMsg.angular.z = twist + ((averageRightRightBox - averageLeftLeftBox)/ 400);
    
    if (averageLeftBox > 200 && averageCenterBox > 200 && averageRightBox > 200){
        twistMsg.linear.x = -0.030;
    }

    // Prüfe, ob die durchschnittlichen Intensitäten innerhalb der Toleranz liegen
    if (std::abs(averageLeftBox - averageCenterBox) <= tolerance && 
        std::abs(averageCenterBox - averageRightBox) <= tolerance && 
        std::abs(averageLeftBox - averageRightBox) <= tolerance) {
        twistMsg.linear.x = 0.0; // Stoppe den Roboter
        twistMsg.linear.z = 0.0; // Stoppe den Roboter
    }

    cv::imshow("Endbild", cv_ptr->image);
    cv::waitKey(1);

    // Veröffentliche die Geschwindigkeit
    cmdVelPublisher.publish(twistMsg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "Follow_Line_node");
    ros::NodeHandle nh;

    ros::Subscriber binaryimageSubscriber = nh.subscribe("BEW_img_mit_LDS", 1, binaryImageCallback);

    
    // Werbe für das cmd_vel-Thema
    cmdVelPublisher = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    ros::spin();

    return 0;
}
