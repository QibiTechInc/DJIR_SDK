#include "Handle.h"

#include "SocketCAN.h"
#include "custom_crc16.h"
#include "custom_crc32.h"

#include <cstring>

namespace {

inline int16_t rd_i16(const std::vector<uint8_t>& d, size_t i)
{
    return (int16_t)(d[i] | (d[i + 1] << 8));
}

// Packet layout (see CmdCombine.cpp):
//   [0] SOF 0xAA  [1..2] ver/len  [3] cmd_type  [4] enc  [5..7] res
//   [8..9] seq    [10..11] CRC16  [12..n-5] DATA  [n-4..n-1] CRC32
const size_t HEADER_LEN = 12;
const size_t MIN_PACKET_LEN = HEADER_LEN + 4;
const size_t MAX_PACKET_LEN = 0x3FF;

}

DJIR_SDK::DataHandle::DataHandle(SocketCAN* can)
    : _can(can), _stopped(true)
{
}

DJIR_SDK::DataHandle::~DataHandle()
{
    stop();
}

void DJIR_SDK::DataHandle::start()
{
    if (_thread.joinable())
        return;
    _stopped = false;
    _thread = std::thread(&DataHandle::run, this);
}

void DJIR_SDK::DataHandle::stop()
{
    _stopped = true;
    if (_thread.joinable())
        _thread.join();
    _cv.notify_all();
}

uint16_t DJIR_SDK::DataHandle::seq_of(const std::vector<uint8_t>& data)
{
    return (uint16_t)(data[8] | (data[9] << 8));
}

bool DJIR_SDK::DataHandle::request(const std::vector<uint8_t>& cmd, std::vector<uint8_t>* reply, int timeout_ms)
{
    if (cmd.size() < MIN_PACKET_LEN)
        return false;
    uint16_t seq = seq_of(cmd);

    std::unique_lock<std::mutex> lk(_mutex);
    if (reply)
    {
        _replies.erase(seq);
        _waiting.insert(seq);
    }
    lk.unlock();

    bool sent = _can->send_cmd(cmd) > 0;

    lk.lock();
    if (sent) _tx_packets++; else _tx_failures++;
    if (!reply)
        return sent;
    if (!sent)
    {
        _waiting.erase(seq);
        return false;
    }

    bool ok = _cv.wait_for(lk, std::chrono::milliseconds(timeout_ms),
                           [&] { return _replies.count(seq) > 0 || _stopped; });
    _waiting.erase(seq);
    if (ok && _replies.count(seq))
    {
        *reply = std::move(_replies[seq]);
        _replies.erase(seq);
        return true;
    }
    _timeouts++;
    return false;
}

bool DJIR_SDK::DataHandle::get_push_data(GimbalPushData& out)
{
    std::lock_guard<std::mutex> lk(_mutex);
    out = _push;
    return _push.count > 0;
}

DJIR_SDK::CalibrationStatus DJIR_SDK::DataHandle::get_calibration_status()
{
    std::lock_guard<std::mutex> lk(_mutex);
    return _calib;
}

void DJIR_SDK::DataHandle::fill_stats(LinkStats& s)
{
    std::lock_guard<std::mutex> lk(_mutex);
    s.tx_packets = _tx_packets;
    s.tx_failures = _tx_failures;
    s.rx_packets = _rx_packets;
    s.crc_errors = _crc_errors;
    s.timeouts = _timeouts;
    s.any_rx = _any_rx;
    s.last_rx = _last_rx;
}

void DJIR_SDK::DataHandle::run()
{
    std::vector<uint8_t> pkt;
    size_t pack_len = 0;

    while (!_stopped)
    {
        // Blocks up to 50 ms waiting for the next CAN frame from the gimbal.
        std::vector<uint8_t> frame = _can->recv_frame(50);
        for (uint8_t b : frame)
        {
            if (pkt.empty())
            {
                if (b == 0xAA)
                    pkt.push_back(b);
                continue;
            }

            pkt.push_back(b);
            if (pkt.size() == 3)
            {
                pack_len = pkt[1] | ((pkt[2] & 0x03) << 8);
                if (pack_len < MIN_PACKET_LEN || pack_len > MAX_PACKET_LEN)
                    pkt.clear();
            }
            else if (pkt.size() == HEADER_LEN)
            {
                if (!check_head_crc(pkt))
                {
                    std::lock_guard<std::mutex> lk(_mutex);
                    _crc_errors++;
                    pkt.clear();
                }
            }
            else if (pkt.size() > HEADER_LEN && pkt.size() == pack_len)
            {
                if (check_pack_crc(pkt))
                    process_packet(pkt);
                else
                {
                    std::lock_guard<std::mutex> lk(_mutex);
                    _crc_errors++;
                }
                pkt.clear();
            }
        }
    }
}

void DJIR_SDK::DataHandle::process_packet(const std::vector<uint8_t>& data)
{
    std::lock_guard<std::mutex> lk(_mutex);
    _rx_packets++;
    _any_rx = true;
    _last_rx = std::chrono::steady_clock::now();

    uint8_t cmd_type = data[3];
    if (cmd_type & 0x20)
    {
        // Reply frame. Only keep it if somebody is waiting for this seq.
        uint16_t seq = seq_of(data);
        if (_waiting.count(seq))
        {
            _replies[seq] = data;
            _cv.notify_all();
        }
        return;
    }

    // Command frame pushed by the gimbal: DATA = CmdSet, CmdID, payload.
    if (data.size() < HEADER_LEN + 2 + 4)
        return;
    uint8_t cmd_set = data[12];
    uint8_t cmd_id = data[13];
    const size_t p = 14;                       // payload offset
    const size_t payload_len = data.size() - p - 4;

    if (cmd_set == 0x0E && cmd_id == 0x08 && payload_len >= 22)
    {
        // 2.3.4.9 Handheld Gimbal Parameter Push
        uint8_t ctrl = data[p];
        _push.angles_valid    = ctrl & 0x01;
        _push.limits_valid    = ctrl & 0x02;
        _push.stiffness_valid = ctrl & 0x04;
        _push.attitude.yaw   = rd_i16(data, p + 1);
        _push.attitude.roll  = rd_i16(data, p + 3);
        _push.attitude.pitch = rd_i16(data, p + 5);
        _push.joint.yaw      = rd_i16(data, p + 7);
        _push.joint.roll     = rd_i16(data, p + 9);
        _push.joint.pitch    = rd_i16(data, p + 11);
        _push.pitch_max = data[p + 13];
        _push.pitch_min = data[p + 14];
        _push.yaw_max   = data[p + 15];
        _push.yaw_min   = data[p + 16];
        _push.roll_max  = data[p + 17];
        _push.roll_min  = data[p + 18];
        _push.pitch_stiffness = data[p + 19];
        _push.yaw_stiffness   = data[p + 20];
        _push.roll_stiffness  = data[p + 21];
        _push.stamp = _last_rx;
        _push.count++;
    }
    else if (cmd_set == 0x0E && cmd_id == 0x10)
    {
        // 2.3.4.17 Gimbal Auto Calibration Status Push (TLV: id, len, value)
        size_t i = p;
        while (i + 2 <= p + payload_len)
        {
            uint8_t id = data[i], len = data[i + 1];
            if (i + 2 + len > p + payload_len)
                break;
            if (id == 0x00 && len >= 2)
            {
                _calib.state = (CalibrationState)data[i + 2];
                _calib.progress = data[i + 3];
                _calib.stamp = _last_rx;
                _calib.received = true;
            }
            i += 2 + len;
        }
    }
}

bool DJIR_SDK::DataHandle::check_head_crc(const std::vector<uint8_t>& data)
{
    crc16_t crc16 = crc16_init();
    crc16 = crc16_update(crc16, data.data(), 10);
    crc16 = crc16_finalize(crc16);
    uint16_t recv_crc = (uint16_t)(data[10] | (data[11] << 8));
    return crc16 == recv_crc;
}

bool DJIR_SDK::DataHandle::check_pack_crc(const std::vector<uint8_t>& data)
{
    size_t n = data.size();
    crc32_t crc32 = crc32_init();
    crc32 = crc32_update(crc32, data.data(), n - 4);
    crc32 = crc32_finalize(crc32);
    uint32_t recv_crc;
    memcpy(&recv_crc, &data[n - 4], 4);
    return crc32 == recv_crc;
}
