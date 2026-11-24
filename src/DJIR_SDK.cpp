#include "DJIR_SDK.h"
#include "Handle.h"
#include "CmdCombine.h"

#include "SocketCAN.h"

enum FLAG : uint8_t {
    BIT1 = 0x01,
    BIT2 = 0x02,
    BIT3 = 0x04,
    BIT4 = 0x08,
    BIT5 = 0x10,
    BIT6 = 0x20,
    BIT7 = 0x40
};

DJIR_SDK::DJIRonin::DJIRonin()
{
    _position_ctrl_byte = 0;
    _speed_ctrl_byte  = 0;

    _position_ctrl_byte |= BIT1; //MoveMode - ABSOLUTE_CONTROL
    _speed_ctrl_byte |= BIT3; //SpeedControl - DISABLED, FocalControl - DISABLED
    _cmd_cmb = new CmdCombine();
    _can_conn = nullptr;
    _pack_thread = nullptr;
}

DJIR_SDK::DJIRonin::~DJIRonin()
{
    disconnect();
    delete (CmdCombine*)_cmd_cmb;
}

bool DJIR_SDK::DJIRonin::connect(const std::string& can_interface)
{
    uint32_t send_id = 0x223;
    uint32_t recv_id = 0x222;

    disconnect();

    // Connect to DJIR gimbal over SocketCAN
    SocketCAN* can = new SocketCAN(send_id, recv_id, can_interface);
    if (!can->open())
    {
        delete can;
        return false;
    }
    _can_conn = can;
    _pack_thread = new DataHandle(_can_conn);
    ((DataHandle*)_pack_thread)->start();
    return true;
}

bool DJIR_SDK::DJIRonin::disconnect()
{
    if (_pack_thread)
    {
        ((DataHandle*)_pack_thread)->stop();
        delete (DataHandle*)_pack_thread;
        _pack_thread = nullptr;
    }
    if (_can_conn)
    {
        delete (SocketCAN*)_can_conn;
        _can_conn = nullptr;
    }
    return true;
}

bool DJIR_SDK::DJIRonin::move_to(int16_t yaw, int16_t roll, int16_t pitch, uint16_t time_ms)
{
    uint8_t cmd_type = 0x03;
    uint8_t cmd_set = 0x0E;
    uint8_t cmd_id = 0x00;
    uint8_t time = (uint8_t)(time_ms/100);
    std::vector<uint8_t> data_payload =
    {
        ((uint8_t*)&yaw)[0],((uint8_t*)&yaw)[1],
        ((uint8_t*)&roll)[0],((uint8_t*)&roll)[1],
        ((uint8_t*)&pitch)[0],((uint8_t*)&pitch)[1],
        _position_ctrl_byte, time
    };
    if (!_can_conn || !_pack_thread)
        return false;
    auto cmd = ((CmdCombine*)_cmd_cmb)->combine(cmd_type, cmd_set, cmd_id, data_payload);
    ((DataHandle*)_pack_thread)->add_cmd(cmd);
    int ret = ((SocketCAN*)_can_conn)->send_cmd(cmd);
    if (ret > 0)
        return true;
    else
        return false;
}

bool DJIR_SDK::DJIRonin::set_inverted_axis(DJIR_SDK::AxisType axis, bool invert)
{
    if (axis == AxisType::YAW)
    {
        if (invert)
            _position_ctrl_byte |= BIT2;
        else
            _position_ctrl_byte &= ~BIT2;
    }

    if (axis == AxisType::ROLL)
    {
        if (invert)
            _position_ctrl_byte |= BIT3;
        else
            _position_ctrl_byte &= ~BIT3;
    }

    if (axis == AxisType::PITCH)
    {
        if (invert)
            _position_ctrl_byte |= BIT4;
        else
            _position_ctrl_byte &= ~BIT4;
    }

    return true;
}

bool DJIR_SDK::DJIRonin::set_move_mode(DJIR_SDK::MoveMode type)
{
    if (type == MoveMode::INCREMENTAL_CONTROL)
    {
        _position_ctrl_byte &= ~BIT1;
    }else
        _position_ctrl_byte |= BIT1;

    return true;
}

bool DJIR_SDK::DJIRonin::set_speed(uint16_t yaw, uint16_t roll, uint16_t pitch)
{
    uint8_t cmd_type = 0x03;
    uint8_t cmd_set = 0x0E;
    uint8_t cmd_id = 0x01;

    std::vector<uint8_t> data_payload =
    {
        ((uint8_t*)&yaw)[0],((uint8_t*)&yaw)[1],
        ((uint8_t*)&roll)[0],((uint8_t*)&roll)[1],
        ((uint8_t*)&pitch)[0],((uint8_t*)&pitch)[1],
        _speed_ctrl_byte
    };
    if (!_can_conn || !_pack_thread)
        return false;
    auto cmd = ((CmdCombine*)_cmd_cmb)->combine(cmd_type, cmd_set, cmd_id, data_payload);
    ((DataHandle*)_pack_thread)->add_cmd(cmd);

    int ret = ((SocketCAN*)_can_conn)->send_cmd(cmd);
    if (ret > 0)
    {
        return true;
    }
    else
        return false;
}

bool DJIR_SDK::DJIRonin::set_speed_mode(DJIR_SDK::SpeedControl speed_type, DJIR_SDK::FocalControl focal_type)
{
    if (speed_type == SpeedControl::DISABLED)
    {
        _speed_ctrl_byte &= ~BIT7;
    }else
        _speed_ctrl_byte |= BIT7;

    if (focal_type == FocalControl::ENABLED)
    {
        _speed_ctrl_byte &= ~BIT3;
    }else
        _speed_ctrl_byte |= BIT3;

    return true;
}

bool DJIR_SDK::DJIRonin::get_current_position(int16_t &yaw, int16_t &roll, int16_t &pitch)
{
    uint8_t cmd_type = 0x03;
    uint8_t cmd_set = 0x0E;
    uint8_t cmd_id = 0x02;

    std::vector<uint8_t> data_payload =
    {
        0x01
    };
    if (!_can_conn || !_pack_thread)
        return false;
    auto cmd = ((CmdCombine*)_cmd_cmb)->combine(cmd_type, cmd_set, cmd_id, data_payload);
    ((DataHandle*)_pack_thread)->add_cmd(cmd);
    ((DataHandle*)_pack_thread)->clear_position();

    int ret = ((SocketCAN*)_can_conn)->send_cmd(cmd);
    if (ret > 0)
    {
        return ((DataHandle*)_pack_thread)->get_position(yaw, roll, pitch, 1000);
    }
    else
        return false;
}
