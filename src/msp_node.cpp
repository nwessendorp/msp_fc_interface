// Adopted from AscendNTNU: 
// https://github.com/AscendNTNU/msp_flightcontroller_interface

#include "msp.hpp"

#include <algorithm>

// threading
// #include <chrono>
// #include <thread>


#include "msp_node.hpp"

Payload MspInterface::serialize_rc_data() {
    Payload result;
    for (int i = 0; i < (int) this->rcData.size(); i++) {
        result.put_u16(this->rcData[i]);
    }
    return result;
}

MspInterface::MspInterface() {

    // Set thurst to 0
    this->rcData[2] = 1000;

    // https://stackoverflow.com/questions/42877001/how-do-i-read-gyro-information-from-cleanflight-using-msp
    msp.register_callback(MSP::ATTITUDE, [this](Payload payload) {
        std::vector<int16_t> attitudeData(payload.size() / 2);
        for (int i = 0; i < (int) attitudeData.size(); i++) {
            // mapping an unsigned to a signed?!
            attitudeData[i] = payload.get_u16();
        }

        std::vector<float> att_f (3, 0);
        // weird 1/10 degree convention betaflight?
        att_f[0] = ((float) attitudeData[0]) / 10.0;
        att_f[1] = ((float) attitudeData[1]) / 10.0;
        att_f[2] = ((float) attitudeData[2]);

        // also weird frame transformation
        //controller->robot.att.pitch = -D2R * att_f[1];
        //controller->robot.att.roll  = D2R * att_f[0];
        //controller->robot.att.yaw   = D2R * att_f[2];
        });
}

void MspInterface::write_to_bf() {

    this->rcData[0] = controller->state_t;
    this->rcData[2] = controller->state_p;
    this->rcData[1] = controller->state_r;
    this->rcData[3] = controller->state_y;

    // Send rc data
    msp.send_msg(MSP::SET_RAW_RC, serialize_rc_data());

    // Recieve new msp messages
    msp.recv_msgs();
}

void MspInterface::read_from_bf() {
    // Request telemetry
    msp.send_msg(MSP::ATTITUDE, {});

    // TODO: get also the arming signals for safety
    // msp.send_msg(MSP::RC, {});

    // Recieve new msp messages
    msp.recv_msgs();
}

msp_node::msp_node() {
    this->msp_node_thread_ = std::thread(&msp_node::msp_node_main, this);
    printf("[msp] thread spawned!\n");
}

void msp_node::msp_node_main() {
    while(1) {
        iface.read_from_bf();
        iface.write_to_bf();
        // 100 Hz
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

msp_node::~msp_node() {
    msp_node_thread_.detach();
    printf("[msp] thread killed!\n");
    printf("[msp] sending disarm signal!\n");
}
