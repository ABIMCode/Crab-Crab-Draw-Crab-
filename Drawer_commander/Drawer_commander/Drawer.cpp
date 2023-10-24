//
//  Drawer.cpp
//  Drawer_commander
//
//  Created by Quentin Huet on 10/12/2022.
//

#include "Drawer.hpp"

void Drawer::display(){

}

bool Drawer::compute_cmd(){
    
    double x1 = m_cartesian_point[0] + DIM_d/2.0;
    double y1 = m_cartesian_point[1];
    double M1_P = std::sqrt(x1*x1 + y1*y1);
    double C1_M1_P = std::acos((DIM_L2*DIM_L2 - DIM_L1*DIM_L1 - M1_P*M1_P)/(-2*DIM_L1*M1_P));
    double Y_M1_P = std::acos((x1*x1 - y1*y1 - M1_P*M1_P)/(-2*y1*M1_P));
    if(x1>0){
        Y_M1_P *= -1;
    }
    m_angle_cmd[0] = C1_M1_P + Y_M1_P;
    
    double x2 = m_cartesian_point[0] - DIM_d/2.0;
    double y2 = m_cartesian_point[1];
    double M2_P = std::sqrt(x2*x2 + y2*y2);
    double C2_M2_P = std::acos((DIM_L2*DIM_L2 - DIM_L1*DIM_L1 - M2_P*M2_P)/(-2*DIM_L1*M2_P));
    double Y_M2_P = std::acos((x2*x2 - y2*y2 - M2_P*M2_P)/(-2*y2*M2_P));
    if(x2<0){
        Y_M2_P *= -1;
    }
    m_angle_cmd[1] = -(C2_M2_P + Y_M2_P);
    if(isnan(m_angle_cmd[0]) || isnan(m_angle_cmd[1])){
        return false;
    }
    compute_servomotors_cmd();
    return true;
}
void Drawer::compute_servomotors_cmd(){
    //4096 steps --> 2*M_PI//
    //1 step --> 0.087890625 Rad
    float ratio = (2.0*0.087890625*M_PI)/180.0;
    m_servomotors_cmd[0] = round(m_angle_cmd[1]/ratio);
    //std::cout<<-m_angle_cmd[0]*2048/M_PI<<" VS "<<m_servomotors_cmd[0]<<std::endl;
    m_servomotors_cmd[1] = round(m_angle_cmd[0]/ratio);
}

void *Drawer::http_server(void* context){
    std::string msg_header = "HTTP/1.1 200 OK\nContent-Type: text/plain\nContent-Length: 20\n\n";
    int new_socket;
    Drawer* drawer = ((Drawer*)context);
    while(!drawer->get_quit()){
        //if(drawer->get_should_send_data()){
        //printf("\n+++++++ Waiting for new connection ++++++++\n");
        if ((new_socket = accept( drawer->get_server_fd(), drawer->get_sockaddr(), drawer->get_addrlen())) < 0 ){
            perror("In accept");
            exit(EXIT_FAILURE);
        }
        
        
        long valread;
        char buffer[300] = {0};
        valread = read( new_socket , buffer, 300);
        //printf("%s\n",buffer );
        std::string rec_msg = buffer;
        std::string::size_type sz;
        rec_msg = rec_msg.substr(16);
        double current_angle_1 = std::stoi(rec_msg, &sz);
        rec_msg = rec_msg.substr(sz + 1);
        double current_angle_2 = std::stoi(rec_msg, &sz);
        std::cout<<current_angle_1<<" and "<<current_angle_2<<std::endl;
        //if((current_angle_1 == drawer->get_servomotors_cmd(0)) && (current_angle_2 == drawer->get_servomotors_cmd(0))){

        
        
        drawer->compute_next_cmd();
        std::stringstream msg;
        msg<<msg_header<<"("<<drawer->get_servomotors_cmd(0)<<";"<<drawer->get_servomotors_cmd(1)<<")";
        //std::cout<<msg.str()<<std::endl;
        write(new_socket , msg.str().c_str() , msg.str().length());
        //printf("------------------message sent------------------- : %d", debug_cpt++);
        close(new_socket);
        //drawer->set_should_send_data(false);
        //}
    }
    pthread_exit(NULL);
}

void Drawer::init(){
    m_angle_cmd[0] = 0.;
    m_cartesian_point[0] = 0.2;
    m_cartesian_point[1] = 0.;
    m_pathmaker.createPath();
    
    m_addrlen = sizeof(m_address);
    // Creating socket file descriptor
    if ((m_server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0){
        perror("In socket");
        exit(EXIT_FAILURE);
    }
    m_address.sin_family = AF_INET;
    m_address.sin_addr.s_addr = INADDR_ANY;
    m_address.sin_port = htons( PORT );
    memset(m_address.sin_zero, '\0', sizeof m_address.sin_zero);
    if (bind(m_server_fd, (struct sockaddr *)&m_address, sizeof(m_address))<0){
        perror("In bind");
        exit(EXIT_FAILURE);
    }
    if (listen(m_server_fd, 10) < 0){
        perror("In listen");
        exit(EXIT_FAILURE);
    }
    int ret = pthread_create(&m_http_thread, NULL, &Drawer::http_server, this);
    if(ret != 0) {
        printf("Error: pthread_create() failed\n");
        exit(EXIT_FAILURE);
    }
}

void Drawer::run(){
    compute_cmd();
    display();
    while(!m_quit){
        //compute_next_cmd();
        if(m_servomotors_cmd[0] < CODE_LIFT){
            m_displayer.draw_robot_servo_cmd(m_servomotors_cmd[0], m_servomotors_cmd[1]);
        }
        cv::waitKey(1);
    }
    cv::waitKey(0);
}

void Drawer::compute_next_cmd(){
    cv::Point2d next = m_pathmaker.get_next_point_in_path();
    if(next.x == CODE_NO_PATH){
        return;
    }
    if(next.x == CODE_END_PATH){
        m_quit = true;
        return;
    }
    if((next.x == CODE_LIFT) || (next.x == CODE_LOWER)){
        m_servomotors_cmd[0] = next.x;
        m_servomotors_cmd[1] = next.y;
    }else{
        m_cartesian_point[0] = next.x;
        m_cartesian_point[1] = next.y;
        compute_cmd();
    }
}
