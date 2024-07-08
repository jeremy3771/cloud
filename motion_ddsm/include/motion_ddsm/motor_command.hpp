#ifndef DDSM_CPP_PY_MOTOR_COMMAND_HPP
#define DDSM_CPP_PY_MOTOR_COMMAND_HPP

#include <boost/asio.hpp>
#include <iostream>
#include <vector>
#include <array>

using namespace boost::asio;
using std::array;
using std::pair;
using std::vector;

class MOTOR_COMMAND {
public:
    MOTOR_COMMAND(int num);
    void ID_SET(uint8_t ID);
    void ID_QUERY();
    void SWITCH_VELOCITY_MODE(uint8_t ID);
    void SWITCH_ANGLE_MODE(uint8_t ID);
    void SET_VELOCITY(uint8_t ID, int SPEED, uint8_t ACC = 0);
    void SET_ANGLE(uint8_t ID, int ANGLE);
    void BRAKE(uint8_t ID);

private:
    io_service io;
    serial_port port;
    pair<uint8_t, uint8_t> decimal_to_hex_bytes(int decimal);
    uint8_t calculate_crc(const vector<uint8_t>& data);
    void send_data(const vector<uint8_t>& data);

    static const array<uint8_t, 256> CRC8_MAXIM_table;
};

#endif // DDSM_CPP_PY_MOTOR_COMMAND_HPP