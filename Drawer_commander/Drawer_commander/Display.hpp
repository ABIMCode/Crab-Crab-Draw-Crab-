//
//  Displayer.hpp
//
//  Created by Quentin Huet on 26/09/2022.
//

#ifndef Displayer_hpp
#define Displayer_hpp

#include <stdio.h>
#include <opencv2/opencv.hpp>
#define WINDOW_NAME "Drawer master"
#define TOPIX 2
#define DIM_d 90
#define DIM_L1 100
#define DIM_L2 140
#define ZERO_Y 20

class Displayer {
protected:
    int m_window_height = 600;
    int m_window_width = 800;
    cv::Mat m_canvas;
    cv::Mat m_drawing;
    
public:
    void draw_robot_servo_cmd(int M1_cmd, int M2_cmd);
    void draw_robot(double theta_1, double theta_2);
    Displayer();
    ~Displayer();
};

#endif /* Displayer_hpp */
