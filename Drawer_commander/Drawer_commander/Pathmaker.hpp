//
//  Pathmaker.hpp
//  Drawer_commander
//
//  Created by Quentin Huet on 12/03/2023.
//

#ifndef Pathmaker_hpp
#define Pathmaker_hpp

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "Display.hpp"

#define CODE_NO_PATH -10000
#define CODE_END_PATH -22222
#define CODE_LIFT 11111
#define CODE_LOWER 33333

class Pathmaker{
    //attributes
    cv::Point2d m_coordinate;
    cv::Mat m_source_image;
    cv::Mat m_computed_image;
    int m_low_Canny_threshold = 90;
    int m_current_contour = 0;
    int m_current_point = 0;
    bool m_lifted = true;
    std::vector<std::vector<cv::Point>> m_lines_to_draw;
    std::vector<std::vector<cv::Point2d>> m_lines_in_world_frame;
    //methodes
    static void ChangeCannyThreshold(int, void*);
    void compute_lines_to_draw();
    cv::Mat& get_source_image(){return m_source_image;}
    int get_low_Canny_threshold(){return m_low_Canny_threshold;}
    std::vector<std::vector<cv::Point>>& get_lines_to_draw(){return m_lines_to_draw;}
public:
    Pathmaker();
    void createPath();
    cv::Point2d get_next_point_in_path();
    void reset_path(){m_current_point = 0; m_current_contour = 0;}
};
#endif /* Pathmaker_hpp */
