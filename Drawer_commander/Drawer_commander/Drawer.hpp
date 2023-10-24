//
//  Drawer.hpp
//  Drawer_commander
//
//  Created by Quentin Huet on 10/12/2022.
//

#ifndef Drawer_hpp
#define Drawer_hpp

#include <cmath>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <sys/socket.h>
#include <unistd.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>
#include <sstream>
#include <pthread.h>

#include "Display.hpp"
#include "Pathmaker.hpp"

#define PORT 8080
#define SERVO_ZERO 0

class Drawer{
protected:
    bool m_quit = false;
    double m_angle_cmd[2] = {0.0};
    double m_cartesian_point[2] = {0.0};
    int m_servomotors_cmd[2] = {SERVO_ZERO};
    //For http com
    int m_server_fd;
    struct sockaddr_in m_address;
    int m_addrlen;
    pthread_t m_http_thread;
    
    Displayer m_displayer;
    Pathmaker m_pathmaker;
    void display();
    bool compute_cmd();
    void compute_servomotors_cmd();
    static void* http_server(void* context);
public:
    Drawer(){}
    void init();
    void run();
    void compute_next_cmd();
    bool get_quit(){return m_quit;}
    struct sockaddr *get_sockaddr(){return (struct sockaddr *)&m_address;}
    socklen_t* get_addrlen(){return (socklen_t*)&m_addrlen;}
    int get_server_fd(){return m_server_fd;}
    double get_cartesian_point(int i){return m_cartesian_point[i];}
    double get_servomotors_cmd(int i){return m_servomotors_cmd[i];}
};
#endif /* Drawer_hpp */
