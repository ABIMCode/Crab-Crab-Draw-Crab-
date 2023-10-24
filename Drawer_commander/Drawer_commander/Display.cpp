//
//  Displayer.cpp
//
//  Created by Quentin Huet on 26/09/2022.
//

#include "Display.hpp"

Displayer::Displayer(){
    m_canvas = cv::Mat(m_window_height,m_window_width, CV_8UC3);
    m_drawing = cv::Mat(m_window_height,m_window_width, CV_8UC3);
    m_drawing = cv::Scalar(255, 255, 255);
    draw_robot(0, 0);
    //cv::waitKey();
}
Displayer::~Displayer(){
    
}
void Displayer::draw_robot_servo_cmd(int M1_cmd, int M2_cmd){
    double theta_1 = -M1_cmd*M_PI/(2048.0/2.0);
    double theta_2 = -M2_cmd*M_PI/(2048.0/2.0);
    draw_robot(theta_1, theta_2);
}


void Displayer::draw_robot(double theta_1, double theta_2){
    m_drawing.copyTo(m_canvas);
    //m_canvas = cv::Scalar(255, 255, 255);
    cv::Point drawing_circle(m_window_width/2, ZERO_Y + std::sqrt((DIM_L1 + DIM_L2/2.0)*(DIM_L1 + DIM_L2/2.0) - (DIM_d/2.0)*(DIM_d/2.0))*TOPIX);
    cv::circle(m_canvas, drawing_circle, (DIM_L2*TOPIX)/2, cv::Scalar(0), 3);
    
    cv::Point M1(m_window_width/2 - (DIM_d/2.0)*TOPIX, ZERO_Y);
    cv::Point M2(m_window_width/2 + (DIM_d/2.0)*TOPIX, ZERO_Y);

    cv::circle(m_canvas, M1, 20, cv::Scalar(0), 2);
    cv::circle(m_canvas, M2, 20, cv::Scalar(0), 2);
    cv::line(m_canvas, M1, M2, cv::Scalar(0));
    cv::circle(m_canvas, M1, 7, cv::Scalar(0), -1);
    cv::circle(m_canvas, M2, 7, cv::Scalar(0), -1);

    double angle_M1 = theta_1 + M_PI/2;
    cv::Point C1(DIM_L1*TOPIX*cos(angle_M1), DIM_L1*TOPIX*sin(angle_M1));
    C1 += M1;
    double angle_M2 = theta_2 + M_PI/2;
    cv::Point C2(DIM_L1*TOPIX*cos(angle_M2), DIM_L1*TOPIX*sin(angle_M2));
    C2 += M2;
    
    //cv::circle(m_canvas, C1, DIM_L2*TOPIX, cv::Scalar(0), 1);
    //cv::circle(m_canvas, C2, DIM_L2*TOPIX, cv::Scalar(0), 1);

    cv::circle(m_canvas, C1, 20, cv::Scalar(0), 2);
    cv::line(m_canvas, M1, C1, cv::Scalar(0), 2);
    cv::circle(m_canvas, C2, 20, cv::Scalar(0), 2);
    cv::line(m_canvas, M2, C2, cv::Scalar(0), 2);
    cv::circle(m_canvas, C1, 7, cv::Scalar(0), -1);
    cv::circle(m_canvas, C2, 7, cv::Scalar(0), -1);
    cv::Point C_mid = C1 + (C2 - C1)/2.0;
    //cv::circle(m_canvas, C_mid, 10, cv::Scalar(0), 2);
    
    double X = C2.x - C1.x;
    double Y = C2.y - C1.y;
    double h = std::sqrt((DIM_L2*TOPIX*DIM_L2*TOPIX) - (X*X  + Y*Y)/4.0);
    double angle_C1 = std::acos(X/(std::sqrt(X*X + Y*Y))) + M_PI/2.0;
    if(C_mid.x>m_window_width/2.0){
        angle_C1 = -angle_C1 + M_PI;
    }
    cv::Point P = C_mid + cv::Point(h*cos(angle_C1), h*sin(angle_C1));
    cv::circle(m_drawing, P, 1, cv::Scalar(255, 0, 0), -1);
    cv::circle(m_canvas, P, 20, cv::Scalar(0), 2);
    cv::line(m_canvas, C1, P, cv::Scalar(0), 2);
    cv::line(m_canvas, C2, P, cv::Scalar(0), 2);
    cv::circle(m_canvas, P, 7, cv::Scalar(0), -1);

    cv::flip(m_canvas, m_canvas, 0);
    cv::imshow(WINDOW_NAME, m_canvas);

}
