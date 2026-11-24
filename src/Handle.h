#ifndef HANDLE_H
#define HANDLE_H

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <map>
#include <mutex>
#include <set>
#include <thread>
#include <vector>

#include "DJIR_SDK.h"

namespace DJIR_SDK {

class SocketCAN;

/**
 * @brief Receive thread + request/reply matching for DJI R SDK packets.
 *
 * Replies are matched to requests by the 2-byte sequence number (bytes 8-9).
 * Unsolicited packets from the gimbal (parameter push 0x0E/0x08 and auto
 * calibration status 0x0E/0x10) are decoded and cached.
 */
class DataHandle
{
public:
    explicit DataHandle(SocketCAN* can);
    ~DataHandle();
    void start();
    void stop();

    /// Send a packet built by CmdCombine. If reply != nullptr, wait up to
    /// timeout_ms for the matching reply and store its full packet there.
    bool request(const std::vector<uint8_t>& cmd, std::vector<uint8_t>* reply, int timeout_ms);

    bool get_push_data(GimbalPushData& out);
    CalibrationStatus get_calibration_status();
    void fill_stats(LinkStats& stats);

private:
    void run();
    void process_packet(const std::vector<uint8_t>& data);
    static bool check_head_crc(const std::vector<uint8_t>& data);
    static bool check_pack_crc(const std::vector<uint8_t>& data);
    static uint16_t seq_of(const std::vector<uint8_t>& data);

    SocketCAN* _can;
    std::thread _thread;
    std::atomic<bool> _stopped;

    std::mutex _mutex;
    std::condition_variable _cv;
    std::set<uint16_t> _waiting;                        // seqs awaiting reply
    std::map<uint16_t, std::vector<uint8_t>> _replies;  // seq -> reply packet

    GimbalPushData _push;
    CalibrationStatus _calib;

    // statistics
    uint64_t _tx_packets = 0, _tx_failures = 0, _rx_packets = 0;
    uint64_t _crc_errors = 0, _timeouts = 0;
    std::chrono::steady_clock::time_point _last_rx;
    bool _any_rx = false;
};

}

#endif //HANDLE_H
