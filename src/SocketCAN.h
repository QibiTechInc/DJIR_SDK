#ifndef DJIR_SOCKETCAN_H
#define DJIR_SOCKETCAN_H

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace DJIR_SDK {

/**
 * @brief Minimal Linux SocketCAN transport for the DJI R SDK protocol.
 *
 * DJI R packets are split into classic 8-byte CAN frames sent with
 * standard id `send_id`; replies arrive on standard id `recv_id`.
 * The interface (e.g. can0) must already be up at 1 Mbit/s:
 *   sudo ip link set can0 up type can bitrate 1000000
 */
class SocketCAN
{
public:
    SocketCAN(uint32_t send_id, uint32_t recv_id, const std::string& ifname = "can0");
    ~SocketCAN();

    bool open();
    void close();
    bool is_open() const;

    /// Send a whole DJI packet, split into 8-byte frames.
    /// @return number of frames sent, 0 on failure.
    int send_cmd(const std::vector<uint8_t>& cmd);

    /// Wait up to timeout_ms for one frame from recv_id.
    /// @return frame payload, empty on timeout/error.
    std::vector<uint8_t> recv_frame(int timeout_ms);

private:
    uint32_t _send_id;
    uint32_t _recv_id;
    std::string _ifname;
    std::atomic<int> _fd;
    std::mutex _send_mutex;
};

}

#endif // DJIR_SOCKETCAN_H
