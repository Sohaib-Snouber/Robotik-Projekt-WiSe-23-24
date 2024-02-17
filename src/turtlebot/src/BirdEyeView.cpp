#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core.hpp>
#include <XmlRpcValue.h>
#include <opencv2/calib3d/calib3d.hpp>

cv::Mat transformationMatrix; // Globale Variable zur Speicherung der Transformationsmatrix
std::vector<cv::Point3f> global3DPoints; // Globale Variable zur Speicherung der 3D-Punkte

image_transport::Publisher binaryImagePublisher; // Publisher für das Binärbild
bool imageSaved = false;


void store3DPoints(const sensor_msgs::LaserScanConstPtr& scanMsg) {
    // Lösche den globalen 3D-Punkte-Vektor
    global3DPoints.clear();

    // Iteriere durch die Laserscandaten
    for (int i = 0; i < scanMsg->ranges.size(); ++i) {
        // Überspringe ungültige Messungen
        if (std::isinf(scanMsg->ranges[i]) || std::isnan(scanMsg->ranges[i])) {
            continue;
        }

        // Berechne den Winkel in Radianten
        double angle = (scanMsg->angle_min + (i * scanMsg->angle_increment));

        // Berechne die x, y, z Koordinaten in der Welt
        double x = scanMsg->ranges[i] * sin(angle);
        double y = scanMsg->ranges[i] * cos(angle);
        double z = 0.18; // die Höhe des Lidar-Sensors über dem Boden

        // Rotation der Koordinaten
        double y_cam = -z;
        double z_cam = y;
        // Spiegelung von x
        x = -x;

        // Translation der Koordinaten (Verschiebung Laserscanner zu Kamera)
        double z_img = z_cam + 0.07;
        double y_img = y_cam + 0.07;

        // Verschiebung am Fußpunkt von Objekt
        y_img = y_img + 0.25;

        // z > 0, damit die Punkte hinter dem Roboter von cv.projectPoints nicht auch transformiert werden
        if (z_img > 0.25 && z_img < 1 && x > -0.5 && x < 0.5) {
            // Speichere den 3D-Punkt im globalen Vektor
            global3DPoints.push_back(cv::Point3f(x, y_img, z_img));
        } else {
            // Damit das Array sonst nicht leer ist
            global3DPoints.push_back(cv::Point3f(10, 10, 0));
        }
    }
}

void apply3DPointsToImage(const sensor_msgs::ImageConstPtr& msg) {
    // Konvertiere ROS-Bildnachricht zu OpenCV-Bild
    cv_bridge::CvImagePtr rectified_img;
    try {
        rectified_img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat image_rect = rectified_img->image;
    cv::imshow("Rektifiziertes Bild1", image_rect);
    cv::waitKey(1);
    cv::Mat transformedImg;

    // Wende perspektivische Transformation mit der geladenen Matrix an
    if (!transformationMatrix.empty()) {
        // Überprüfe, ob 3D-Punkte verfügbar sind
        if (global3DPoints.empty()) {
            ROS_WARN("Keine 3D-Punkte für die Projektion verfügbar.");
            return;
        }

        // Kamerakalibrierungsparameter
        cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 846.329967, 0, 484.642918, 0, 849.890359, 363.073651, 0, 0, 1);
        cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << 0.077532, -0.167841, 0.002297, 0.001625, 0);

        // Konvertiere 3D-Punkte in 2D-Bildpunkte
        std::vector<cv::Point2f> imagePoints;
        cv::projectPoints(global3DPoints, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), cameraMatrix, distCoeffs, imagePoints);
        //Zeichne die projizierten Punkte auf das Bild
        for (const auto& point : imagePoints) {
            cv::circle(image_rect, point, 5, cv::Scalar(255, 255, 255), -1);
        }
        
        cv::warpPerspective(image_rect, transformedImg, transformationMatrix, image_rect.size());

        // Veröffentliche das transformierte Bild
        sensor_msgs::ImagePtr transformedImageMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", transformedImg).toImageMsg();
        binaryImagePublisher.publish(transformedImageMsg);

        if (!transformationMatrix.empty() && !imageSaved) {

            // Speichere das rektifizierte Bild auf dem Desktop
            // Ersetzen Sie "/home/username/Desktop/rektifiziertes_bild.jpg" mit Ihrem tatsächlichen Pfad
            cv::imwrite("/home/ubuntu/Desktop/front_view_mit_neue_kallibrierung.jpg", image_rect);
            ROS_INFO("Rektifiziertes Bild wurde gespeichert.");
            imageSaved = true; // Verhindern Sie, dass das Bild erneut gespeichert wird

        }
        cv::imshow("Rektifiziertes Bild", image_rect);
        cv::waitKey(1);

    } else {
        ROS_INFO("Transformationsmatrix nicht verfügbar.");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "Bird_View_node");
    ros::NodeHandle nh;

    // Lade die Transformationsmatrix aus der YAML-Datei im Konfigurationsordner
    XmlRpc::XmlRpcValue matrix_param;
    if (nh.getParam("/transformation_matrix", matrix_param)) {
        if (matrix_param.getType() == XmlRpc::XmlRpcValue::TypeArray && matrix_param.size() == 9) {
            // Extrahiere die Matrixelemente
            double transformationMatrixData[3][3];
            int k = 0;
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    transformationMatrixData[i][j] = static_cast<double>(matrix_param[k]);
                    ++k;
                }
            }

            // Kopiere die Matrixdaten in ein cv::Mat
            transformationMatrix = cv::Mat(3, 3, CV_64F, transformationMatrixData).clone();
        } else {
            ROS_ERROR("Transformationsmatrix sollte 9 Elemente enthalten.");
        }
    } else {
        ROS_ERROR("Fehler beim Abrufen der Transformationsmatrix vom Parameter-Server.");
    }

    // Werbe für das Binärbildthema
    image_transport::ImageTransport it(nh);
    binaryImagePublisher = it.advertise("BEW_img_mit_LDS", 1);

    // Abonniere das Originalbildthema
    // image_transport::Subscriber subImage = it.subscribe("raspicam_node/image_rect", 1, apply3DPointsToImage);
    image_transport::Subscriber subImage = it.subscribe("raspicam_node/image_rect", 1, apply3DPointsToImage);

    // Abonniere das Scan-Thema, um 3D-Punkte zu speichern
    ros::Subscriber subScan = nh.subscribe("scan", 1, store3DPoints);

    cv::startWindowThread();

    ros::spin();

    cv::destroyAllWindows();
    return 0;
}
