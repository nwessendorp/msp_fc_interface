#pragma once
#include <thread>
#include <math.h>
#include <ros/ros.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>



class Controller {
    private:
        std::thread control_job_;

    public:
        uint16_t state_t;
        uint16_t state_p;
        uint16_t state_r;
        uint16_t state_y;
		//robot_t robot;
        //signals<float> signals_f;
        //signals<uint16_t> signals_i;
        Controller();
        ~Controller();
        void do_nothing();
        void set_keys(void);
        int getch(void);
        //void control_job();
        //void altitude_control();
		//void position_control();
		//void velocity_control(float, float);
        //void toActuators();
		//void rateBound(signals<float> *signal);
};