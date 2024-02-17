#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <fstream>
#include <iostream>

int main() {
    // Bild lesen
    cv::Mat image = cv::imread("/home/ubuntu/Desktop/catkin_ws/src/turtlebot/images/front_view_vor_BEV.png");
    if (image.empty()) {
        std::cout << "Bild konnte nicht geladen werden." << std::endl;
        return -1;
    }

    // In Graustufen konvertieren
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    // Schachbrettgröße definieren (Breite x Höhe)
    cv::Size board_size(9, 6); // Anpassen basierend auf Ihrem Schachbrett

    // Schachbrettecken finden
    std::vector<cv::Point2f> corners;
    bool pattern_found = cv::findChessboardCorners(gray, board_size, corners);

    if (pattern_found) {
        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 50, 0.01));

        std::vector<cv::Point2f> outer_corners = {
        corners[board_size.width*board_size.height-board_size.width+3],   // Ecke 0: Oben-links
        corners[3],                    // Ecke 1: Oben-rechts
        corners[board_size.width*board_size.height-1],     // Ecke 2: Unten-links
        corners[board_size.width-1]  // Ecke 3: Unten-rechts
        };

        // Kreise und Beschriftungen für jede Ecke auf dem Originalbild zeichnen
        for (size_t i = 0; i < outer_corners.size(); ++i) {
            int x = static_cast<int>(outer_corners[i].x);
            int y = static_cast<int>(outer_corners[i].y);        
            cv::circle(image, cv::Point(x, y), 5, cv::Scalar(0, 0, 255), -1); // Zeichnet einen roten Kreis
            cv::putText(image, "Ecke " + std::to_string(i), cv::Point(x - 35, y - 20),cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1); // Beschriftet jede Ecke
        }

        // Bild mit nummerierten Ecken anzeigen
        cv::namedWindow("Ecken nummeriert", cv::WINDOW_NORMAL);
        cv::imshow("Ecken nummeriert", image);
        cv::waitKey(0);
        cv::destroyAllWindows();

        // Berechnung der neuen Punkte für die perspektivische Transformation
        float left_difference = outer_corners[0].x - outer_corners[2].x;
        float right_difference = outer_corners[3].x - outer_corners[1].x;

        // Berechnung der neuen Zielkoordinaten für die perspektivische Transformation
        cv::Point2f tlnew(outer_corners[0].x, outer_corners[0].y - left_difference - right_difference);
        cv::Point2f trnew(outer_corners[1].x, outer_corners[1].y - left_difference - right_difference);
        cv::Point2f blnew(outer_corners[2].x + left_difference, outer_corners[2].y);
        cv::Point2f brnew(outer_corners[3].x - right_difference, outer_corners[3].y);

        std::vector<cv::Point2f> src_points = { outer_corners[0], outer_corners[1], outer_corners[2], outer_corners[3] };
        std::vector<cv::Point2f> dst_points = { tlnew, trnew, blnew, brnew };

        // Berechnen der Homographie und Anwenden der perspektivischen Transformation
        cv::Mat H = cv::findHomography(src_points, dst_points);
        cv::Mat transformed_img;
        cv::warpPerspective(image, transformed_img, H, image.size());


        // Transformiertes Bild anzeigen
        cv::namedWindow("Transformiertes Bild", cv::WINDOW_NORMAL);
        cv::imshow("Transformiertes Bild", transformed_img);
        cv::waitKey(0);
        cv::destroyAllWindows();


        // Transformationsmatrix in eine YAML-Datei speichern
        std::ofstream file("/home/ubuntu/Desktop/catkin_ws/src/turtlebot/config/transformation_matrix.yaml");
        if (file.is_open()) {
            file << "transformation_matrix:\n";
            for (int i = 0; i < H.rows; ++i) {
                for (int j = 0; j < H.cols; ++j) {
                    file << std::fixed << std::setprecision(6) << "  - " << H.at<double>(i, j) << "\n";                }
            }
            file.close();
        } else {
            std::cout << "Datei konnte nicht geöffnet werden." << std::endl;
        }
    } else {
        std::cout << "Schachbrettmuster nicht gefunden." << std::endl;
    }

    return 0;
}
