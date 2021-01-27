#include "controller.hpp"


void Controller::set_keys(void) {
    char key(' ');
    key = this->getch();
    if ((key == 'D' || key == 'd') && state_p < 1600){
        this->state_p += 10;
    } else if ((key == 'A' || key == 'a') && state_p > 1400) {
        this->state_p -= 10;
    } else if ((key == 'W' || key == 'w') && state_r < 1600) {
        this->state_r += 10;
    } else if ((key == 'S' || key == 's') && state_r > 1400) {
        this->state_r -= 10;
    } else if ((key == 'Q' || key == 'q') && state_y > 1400) {
        this->state_y -= 10;
    } else if ((key == 'E' || key == 'e') && state_y < 1600) {
        this->state_y += 10;
    } else if (key == '\x03') {
        printf("stopping...\n");//ROS_INFO_STREAM
        ros::shutdown();
    }
}

int Controller::getch(void){
    int ch;
    struct termios oldt;
    struct termios newt;

    // Store old settings, and copy to new settings
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    // Make required changes and apply the settings
    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_iflag |= IGNBRK;
    newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
    newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
    newt.c_cc[VMIN] = 0;
    newt.c_cc[VTIME] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &newt);

    // Get the current character
    ch = getchar();

    // Reapply old settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return ch;
}

void Controller::do_nothing() {
    ros::Rate rate_1(50);
    state_t = 1500;
    state_p = 1500;
    state_r = 1500;
    state_y = 1500;
    while(1) {
        this->set_keys();
        rate_1.sleep();
    }
}

// constructor
Controller::Controller() {    
    try {
        control_job_ = std::thread(&Controller::do_nothing, this);
        printf("[ctrl] thread spawned!\n");
    } catch (...) {
        printf("[ctrl] can't start thread!\n");
    }
}

Controller::~Controller() {
    // fflush all files
    // if (control_job_.joinable()) {
        control_job_.detach();
    // }
    printf("[ctrl] thread killed!\n");
}