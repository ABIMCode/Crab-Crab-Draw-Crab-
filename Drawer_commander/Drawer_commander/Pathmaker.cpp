//
//  Pathmaker.cpp
//  Drawer_commander
//
//  Created by Quentin Huet on 12/03/2023.
//

#include "Pathmaker.hpp"

Pathmaker::Pathmaker(){
    m_coordinate = cv::Point2d(70, 100);
}

void Pathmaker::ChangeCannyThreshold(int, void* object){
    cv::Mat dst, source_image_gray, filtered_image, detected_edges;
    cv::Mat source_image = ((Pathmaker*)object)->get_source_image();
    dst.create( source_image.size(), source_image.type() );
    cv::cvtColor( source_image, source_image_gray, cv::COLOR_BGR2GRAY );
    cv::GaussianBlur( source_image_gray, filtered_image, cv::Size(5,5), 0);
    const int ratio = 3;
    const int kernel_size = 3;
    cv::Canny( filtered_image, detected_edges, ((Pathmaker*)object)->get_low_Canny_threshold(), ((Pathmaker*)object)->get_low_Canny_threshold()*ratio, kernel_size, true);
    dst = cv::Scalar(0, 0, 0);
    //((Pathmaker*)object)->get_source_image().copyTo(dst, detected_edges);
    //cv::imshow("detected edges", detected_edges);
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours( detected_edges, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE );
    /*std::cout<<contours.size()<<std::endl;
    std::cout<<hierarchy.size()<<std::endl;
    std::cout<<hierarchy[0]<<std::endl;*/
    //filtering out the small contours (noise)
    int cpt = 0;
    for (std::vector<std::vector<cv::Point>>::iterator it = contours.begin(); it != contours.end(); ){
        if(it->size()<=80 || hierarchy[cpt][3]!= -1){
          it = contours.erase(it);
        }else{
          ++it;
        }
        cpt ++;
    }

    //displaying the contours
    cv::RNG rng(1234);
    for( size_t i = 0; i< contours.size(); i++ ){
        cv::Scalar color = cv::Scalar(rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
        drawContours( dst, contours, (int)i, color, 2, cv::LINE_8, hierarchy, 0 );
    }
    cv::imshow("Extracted lines", dst);

    ((Pathmaker*)object)->get_lines_to_draw().resize(contours.size());
    for(int contour = 0; contour<contours.size(); contour++){
        ((Pathmaker*)object)->get_lines_to_draw()[contour].resize(contours[contour].size());
        for(int point = 0; point<contours[contour].size(); point ++){
            ((Pathmaker*)object)->get_lines_to_draw()[contour][point] = contours[contour][point];
        }
    }
    
}

void Pathmaker::compute_lines_to_draw(){
    const char* window_get_lines = "Extracted lines";
    cv::namedWindow( window_get_lines, cv::WINDOW_AUTOSIZE );
    const int max_lowThreshold = 100;
    cv::createTrackbar( "Canny threshold : ", window_get_lines, &m_low_Canny_threshold, max_lowThreshold, &Pathmaker::ChangeCannyThreshold, this);
    ChangeCannyThreshold(m_low_Canny_threshold, this);
    cv::waitKey(0);
    cv::destroyWindow(window_get_lines);
}

void Pathmaker::createPath(){
    //std::string image_file = "/Users/quentinhuet/Desktop/Projets_info/Drawer_commander/Source_images/round.png";
    std::string image_file = "/Users/quentinhuet/Desktop/Projets_info/Drawer_commander/Source_images/square.png";
    //std::string image_file = "/Users/quentinhuet/Desktop/Projets_info/Drawer_commander/Source_images/test.bmp";

    m_source_image = cv::imread(image_file, cv::IMREAD_COLOR);
    if(m_source_image.empty()){
        std::cout << "Could not read the image : " << image_file << std::endl;
        return;
    }
    compute_lines_to_draw();
    cv::Mat to_draw;
    cv::RNG rng(1234);
    cv::Point point_prev(0, 0);
    point_prev = m_lines_to_draw[0][0];
    m_source_image = cv::Scalar(0);
    for( size_t i = 0; i< m_lines_to_draw.size(); i++ ){
        cv::line(m_source_image, point_prev, m_lines_to_draw[i][0], cv::Scalar(0, 255, 255), 2);
        point_prev = m_lines_to_draw[i][0];
        cv::Scalar color = cv::Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
        drawContours( m_source_image, m_lines_to_draw, (int)i, color, 2, cv::LINE_8);
    }
    cv::cvtColor( m_source_image, to_draw, cv::COLOR_BGR2GRAY );
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(to_draw, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE );
    cv::Point2f center;
    float radius;
    cv::minEnclosingCircle(contours[0], center, radius);
    cv::circle(to_draw, center, radius, cv::Scalar(255, 0, 0));
    //double ratio = (DIM_L2 - 6)/(2*radius);
    double ratio = (DIM_L2 - 100)/(2*radius);
    std::cout<<"ratio = "<<ratio<<std::endl;
    cv::Point2d world_frame_center(0, std::sqrt((DIM_L1 + DIM_L2/2.0)*(DIM_L1 + DIM_L2/2.0) - (DIM_d/2.0)*(DIM_d/2.0)));
    m_lines_in_world_frame.resize(m_lines_to_draw.size());
    for(int i = 0; i< m_lines_to_draw.size(); i++ ){
        m_lines_in_world_frame[i].resize(m_lines_to_draw[i].size());
        for(int pt = 0; pt<m_lines_to_draw[i].size(); pt++){
            m_lines_in_world_frame[i][pt].x = (m_lines_to_draw[i][pt].x - center.x)*ratio;
            m_lines_in_world_frame[i][pt].y = (m_lines_to_draw[i][pt].y - center.y)*ratio + world_frame_center.y;
        }
    }
    cv::imshow("Computed image", to_draw);
    cv::waitKey(0);
    //cv::destroyWindow("Computed image");
}

cv::Point2d Pathmaker::get_next_point_in_path(){
    if(m_lines_to_draw.size() == 0){
        return cv::Point2d(CODE_NO_PATH, CODE_NO_PATH);
    }
    if((m_current_contour >= m_lines_to_draw.size()) && (m_lifted == true)){
        return cv::Point2d(CODE_END_PATH, CODE_END_PATH);
    }
    if(m_current_point >= m_lines_to_draw[m_current_contour].size()){
        m_current_point = 0;
        m_current_contour++;
        m_lifted = true;
        return cv::Point2d(CODE_LIFT, CODE_LIFT);
    }
    if((m_current_point == 1) && (m_lifted == true)){
        m_lifted = false;
        return cv::Point2d(CODE_LOWER, CODE_LOWER);
    }
    cv::Point2d next_point = m_lines_in_world_frame[m_current_contour][m_current_point];
    m_current_point++;
    return next_point;
}
