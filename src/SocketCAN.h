#ifndef DJIR_SOCKETCAN_H
#define DJIR_SOCKETCAN_H

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace DJIR_SDK {

/// CAN controller state, as reported by SocketCAN error frames.
enum class CanBusState {
    UNKNOWN = 0,
    ERROR_ACTIVE,   ///< normal operation
    ERROR_WARNING,  ///< error counters > 96
    ERROR_PASSIVE,  ///< error counters > 127 (e.g. nobody ACKs our frames)
    BUS_OFF         ///< controller stopped; needs `ip link set <if> down/up`
};

const char* to_string(CanBusState state);

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
    const std::string& ifname() const { return _ifname; }

    /// True if the network interface exists and is administratively UP.
    bool interface_up() const;

    /// Send a whole DJI packet, split into 8-byte frames.
    /// @return number of frames sent, 0 on failure.
    int send_cmd(const std::vector<uint8_t>& cmd);

    /// Wait up to timeout_ms for one data frame from recv_id.
    /// Error frames are consumed internally to update the bus state.
    /// @return frame payload, empty on timeout/error.
    std::vector<uint8_t> recv_frame(int timeout_ms);

    CanBusState bus_state() const { return _bus_state; }
    uint64_t error_frames() const { return _error_frames; }
    uint64_t ack_errors() const { return _ack_errors; }
    uint64_t tx_errors() const { return _tx_errors; }

private:
    void handle_error_frame(uint32_t can_id, const uint8_t* data);

    uint32_t _send_id;
    uint32_t _recv_id;
    std::string _ifname;
    std::atomic<int> _fd;
    std::mutex _send_mutex;

    std::atomic<CanBusState> _bus_state;
    std::atomic<uint64_t> _error_frames;
    std::atomic<uint64_t> _ack_errors;
    std::atomic<uint64_t> _tx_errors;
};

}

#endif // DJIR_SOCKETCAN_H
