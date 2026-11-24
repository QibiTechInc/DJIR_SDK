#include "DJIR_SDK.h"
#include "Handle.h"
#include "CmdCombine.h"
#include "SocketCAN.h"

#include <algorithm>

using namespace DJIR_SDK;

namespace {

// 2.3.4.1 position control ctrl_byte
const uint8_t POS_ABSOLUTE      = 0x01;  // bit0
const uint8_t POS_YAW_INVALID   = 0x02;  // bit1
const uint8_t POS_ROLL_INVALID  = 0x04;  // bit2
const uint8_t POS_PITCH_INVALID = 0x08;  // bit3

// 2.3.4.2 speed control ctrl_byte
const uint8_t SPEED_TAKE_OVER   = 0x80;  // bit7
const uint8_t SPEED_IGNORE_FOCAL = 0x08; // bit3

const uint8_t CMD_TYPE_REPLY_NEEDED = 0x03;

inline void put_i16(std::vector<uint8_t>& v, int16_t x)
{
    v.push_back((uint8_t)(x & 0xFF));
    v.push_back((uint8_t)((x >> 8) & 0xFF));
}

inline int16_t rd_i16(const std::vector<uint8_t>& d, size_t i)
{
    return (int16_t)(d[i] | (d[i + 1] << 8));
}

inline uint8_t time_byte(uint16_t time_ms)
{
    int t = (time_ms + 50) / 100;           // unit 0.1 s
    return (uint8_t)std::max(1, std::min(255, t));
}

}

const char* DJIR_SDK::to_string(ReturnCode code)
{
    switch (code)
    {
    case ReturnCode::EXECUTION_SUCCESSFUL: return "OK";
    case ReturnCode::PARSE_ERROR:          return "command parse error";
    case ReturnCode::EXECUTION_FAILS:      return "command execution failed";
    case ReturnCode::UNDEFINED_ERROR:      return "undefined error";
    case ReturnCode::NO_REPLY:             return "no reply";
    }
    return "unknown";
}

DJIRonin::DJIRonin()
{
    _position_ctrl_byte = POS_ABSOLUTE;
    _speed_ctrl_byte = SPEED_IGNORE_FOCAL;
    _cmd_cmb = new CmdCombine();
    _can_conn = nullptr;
    _pack_thread = nullptr;
    _reply_timeout_ms = 200;
    _last_rc = ReturnCode::EXECUTION_SUCCESSFUL;
}

DJIRonin::~DJIRonin()
{
    disconnect();
    delete (CmdCombine*)_cmd_cmb;
}

bool DJIRonin::connect(const std::string& can_interface)
{
    const uint32_t send_id = 0x223;
    const uint32_t recv_id = 0x222;

    disconnect();

    SocketCAN* can = new SocketCAN(send_id, recv_id, can_interface);
    if (!can->open())
    {
        delete can;
        return false;
    }
    _can_conn = can;
    _pack_thread = new DataHandle(can);
    ((DataHandle*)_pack_thread)->start();
    return true;
}

bool DJIRonin::disconnect()
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

bool DJIRonin::is_connected() const
{
    return _can_conn && _pack_thread && ((SocketCAN*)_can_conn)->is_open();
}

void DJIRonin::set_reply_timeout(int timeout_ms)
{
    _reply_timeout_ms = std::max(10, timeout_ms);
}

bool DJIRonin::request(uint8_t cmd_set, uint8_t cmd_id, const std::vector<uint8_t>& payload,
                       std::vector<uint8_t>* reply_payload)
{
    if (!is_connected())
    {
        _last_rc = ReturnCode::NO_REPLY;
        return false;
    }

    std::vector<uint8_t> cmd;
    {
        std::lock_guard<std::mutex> lk(_cmd_mutex);
        cmd = ((CmdCombine*)_cmd_cmb)->combine(CMD_TYPE_REPLY_NEEDED, cmd_set, cmd_id, payload);
    }

    std::vector<uint8_t> reply;
    if (!((DataHandle*)_pack_thread)->request(cmd, &reply, _reply_timeout_ms))
    {
        _last_rc = ReturnCode::NO_REPLY;
        return false;
    }

    // Reply DATA = CmdSet, CmdID, return code, ... (then CRC32)
    if (reply.size() < 12 + 3 + 4)
    {
        _last_rc = ReturnCode::UNDEFINED_ERROR;
        return false;
    }
    if (reply_payload)
        reply_payload->assign(reply.begin() + 14, reply.end() - 4);

    _last_rc = (ReturnCode)reply[14];
    return _last_rc == ReturnCode::EXECUTION_SUCCESSFUL;
}

bool DJIRonin::move_axes(int16_t yaw, int16_t roll, int16_t pitch, uint16_t time_ms,
                         MoveMode mode, bool yaw_enabled, bool roll_enabled, bool pitch_enabled)
{
    uint8_t ctrl = 0;
    if (mode == MoveMode::ABSOLUTE_CONTROL) ctrl |= POS_ABSOLUTE;
    if (!yaw_enabled)   ctrl |= POS_YAW_INVALID;
    if (!roll_enabled)  ctrl |= POS_ROLL_INVALID;
    if (!pitch_enabled) ctrl |= POS_PITCH_INVALID;

    std::vector<uint8_t> payload;
    put_i16(payload, yaw);
    put_i16(payload, roll);
    put_i16(payload, pitch);
    payload.push_back(ctrl);
    payload.push_back(time_byte(time_ms));
    return request(0x0E, 0x00, payload, nullptr);
}

bool DJIRonin::move_to(int16_t yaw, int16_t roll, int16_t pitch, uint16_t time_ms)
{
    uint8_t c = _position_ctrl_byte;
    return move_axes(yaw, roll, pitch, time_ms,
                     (c & POS_ABSOLUTE) ? MoveMode::ABSOLUTE_CONTROL : MoveMode::INCREMENTAL_CONTROL,
                     !(c & POS_YAW_INVALID), !(c & POS_ROLL_INVALID), !(c & POS_PITCH_INVALID));
}

bool DJIRonin::set_axis_enabled(AxisType axis, bool enabled)
{
    uint8_t bit = axis == AxisType::YAW  ? POS_YAW_INVALID :
                  axis == AxisType::ROLL ? POS_ROLL_INVALID : POS_PITCH_INVALID;
    if (enabled)
        _position_ctrl_byte &= ~bit;
    else
        _position_ctrl_byte |= bit;
    return true;
}

bool DJIRonin::set_inverted_axis(AxisType axis, bool invert)
{
    return set_axis_enabled(axis, !invert);
}

bool DJIRonin::set_move_mode(MoveMode type)
{
    if (type == MoveMode::INCREMENTAL_CONTROL)
        _position_ctrl_byte &= ~POS_ABSOLUTE;
    else
        _position_ctrl_byte |= POS_ABSOLUTE;
    return true;
}

bool DJIRonin::set_speed(int16_t yaw, int16_t roll, int16_t pitch)
{
    std::vector<uint8_t> payload;
    put_i16(payload, yaw);
    put_i16(payload, roll);
    put_i16(payload, pitch);
    payload.push_back(_speed_ctrl_byte);
    return request(0x0E, 0x01, payload, nullptr);
}

bool DJIRonin::set_speed_mode(SpeedControl speed_type, FocalControl focal_type)
{
    if (speed_type == SpeedControl::ENABLED)
        _speed_ctrl_byte |= SPEED_TAKE_OVER;
    else
        _speed_ctrl_byte &= ~SPEED_TAKE_OVER;

    if (focal_type == FocalControl::ENABLED)
        _speed_ctrl_byte &= ~SPEED_IGNORE_FOCAL;
    else
        _speed_ctrl_byte |= SPEED_IGNORE_FOCAL;
    return true;
}

bool DJIRonin::get_angles(uint8_t type, int16_t& yaw, int16_t& roll, int16_t& pitch)
{
    std::vector<uint8_t> r;
    if (!request(0x0E, 0x02, {type}, &r))
        return false;
    // r: [0] return code, [1] data type, [2..7] yaw, roll, pitch
    if (r.size() < 8 || r[1] != type)
        return false;
    yaw = rd_i16(r, 2);
    roll = rd_i16(r, 4);
    pitch = rd_i16(r, 6);
    return true;
}

bool DJIRonin::get_current_position(int16_t& yaw, int16_t& roll, int16_t& pitch)
{
    return get_attitude_angles(yaw, roll, pitch);
}

bool DJIRonin::get_attitude_angles(int16_t& yaw, int16_t& roll, int16_t& pitch)
{
    return get_angles(0x01, yaw, roll, pitch);
}

bool DJIRonin::get_joint_angles(int16_t& yaw, int16_t& roll, int16_t& pitch)
{
    return get_angles(0x02, yaw, roll, pitch);
}

bool DJIRonin::recenter()
{
    return request(0x0E, 0x0E, {0xFE, 0x01}, nullptr);
}

bool DJIRonin::set_auto_calibration(bool start, bool single_attitude_mode)
{
    uint8_t value = (start ? 0x01 : 0x00) | (single_attitude_mode ? 0x02 : 0x00);
    // TLV: id 0x00 (self-tuning), length 1, value
    return request(0x0E, 0x0F, {0x00, 0x01, value}, nullptr);
}

CalibrationStatus DJIRonin::get_calibration_status()
{
    if (!_pack_thread)
        return CalibrationStatus();
    return ((DataHandle*)_pack_thread)->get_calibration_status();
}

bool DJIRonin::set_push_enabled(bool enabled)
{
    return request(0x0E, 0x07, {(uint8_t)(enabled ? 0x01 : 0x02)}, nullptr);
}

bool DJIRonin::get_push_data(GimbalPushData& data)
{
    if (!_pack_thread)
        return false;
    return ((DataHandle*)_pack_thread)->get_push_data(data);
}

bool DJIRonin::get_version(uint32_t& version)
{
    std::vector<uint8_t> r;
    // Device ID 0x00000001 = DJI R SDK
    if (!request(0x0E, 0x09, {0x01, 0x00, 0x00, 0x00}, &r) || r.size() < 9)
        return false;
    version = (uint32_t)r[5] | ((uint32_t)r[6] << 8) | ((uint32_t)r[7] << 16) | ((uint32_t)r[8] << 24);
    return true;
}

LinkStats DJIRonin::get_link_stats()
{
    LinkStats s;
    s.connected = is_connected();
    s.last_return_code = _last_rc;
    if (_can_conn)
    {
        SocketCAN* can = (SocketCAN*)_can_conn;
        s.interface_up = can->interface_up();
        s.bus_state = to_string(can->bus_state());
        s.can_error_frames = can->error_frames();
        s.can_ack_errors = can->ack_errors();
    }
    else
        s.bus_state = "UNKNOWN";
    if (_pack_thread)
        ((DataHandle*)_pack_thread)->fill_stats(s);
    return s;
}
