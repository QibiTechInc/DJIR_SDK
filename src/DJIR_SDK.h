#ifndef DJI_R_SDK_H
#define DJI_R_SDK_H

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <stdlib.h>
#include <string>
#include <vector>

namespace DJIR_SDK {

enum class ReturnCode {
    EXECUTION_SUCCESSFUL = 0,
    PARSE_ERROR = 1,
    EXECUTION_FAILS = 2,
    UNDEFINED_ERROR = 0xFF,
    NO_REPLY = 0x100          ///< not from the gimbal: request timed out / not sent
};

const char* to_string(ReturnCode code);

enum class AxisType {
    YAW = 0,
    ROLL = 1,
    PITCH = 2
};

enum class MoveMode {
    INCREMENTAL_CONTROL = 0,
    ABSOLUTE_CONTROL = 1
};

enum class SpeedControl {
    DISABLED = 0,
    ENABLED = 1
};

enum class FocalControl {
    ENABLED = 0,
    DISABLED = 1
};

/// Angles in 0.1 degree units.
struct Angles {
    int16_t yaw = 0;
    int16_t roll = 0;
    int16_t pitch = 0;
};

/// Content of the gimbal parameter push (0x0E/0x08).
struct GimbalPushData {
    bool angles_valid = false;
    bool limits_valid = false;
    bool stiffness_valid = false;
    Angles attitude;
    Angles joint;
    uint8_t pitch_max = 0, pitch_min = 0;   ///< degrees
    uint8_t yaw_max = 0, yaw_min = 0;
    uint8_t roll_max = 0, roll_min = 0;
    uint8_t pitch_stiffness = 0, yaw_stiffness = 0, roll_stiffness = 0;  ///< 0..100
    std::chrono::steady_clock::time_point stamp;
    uint64_t count = 0;                      ///< number of pushes received
};

enum class CalibrationState : uint8_t {
    IDLE = 0,
    RUNNING = 1,
    COMPLETED = 2,
    FAILED = 3
};

struct CalibrationStatus {
    bool received = false;   ///< false until the gimbal has pushed any status
    CalibrationState state = CalibrationState::IDLE;
    uint8_t progress = 0;    ///< 0..100
    std::chrono::steady_clock::time_point stamp;
};

/// Link health, for diagnostics.
struct LinkStats {
    bool connected = false;
    bool interface_up = false;
    std::string bus_state;          ///< ERROR-ACTIVE, ERROR-PASSIVE, BUS-OFF, ...
    uint64_t tx_packets = 0;
    uint64_t tx_failures = 0;
    uint64_t rx_packets = 0;
    uint64_t crc_errors = 0;
    uint64_t timeouts = 0;          ///< requests with no reply
    uint64_t can_error_frames = 0;
    uint64_t can_ack_errors = 0;    ///< frames nobody ACKed (gimbal off / wiring)
    bool any_rx = false;
    std::chrono::steady_clock::time_point last_rx;
    ReturnCode last_return_code = ReturnCode::EXECUTION_SUCCESSFUL;
};


class DJIRonin
{
public:
    DJIRonin();
    ~DJIRonin();

    /**
     * @brief connect - Connect to DJI Ronin device via Linux SocketCAN
     * @param can_interface SocketCAN interface name (must be up at 1 Mbit/s)
     * @return True if the CAN socket was opened
     */
    bool connect(const std::string& can_interface = "can0");
    /**
     * @brief disconnect - Disconnect from DJI Ronin device
     * @return True if success
     */
    bool disconnect();
    bool is_connected() const;

    /// Timeout used when waiting for replies (default 200 ms).
    void set_reply_timeout(int timeout_ms);

    /**
     * @brief move_to - Handheld Gimbal Position Control (p.5, 2.3.4.1)
     * Uses the mode set by set_move_mode() and the axes enabled by
     * set_axis_enabled().
     * @param yaw Yaw angle, unit: 0.1° (range: -1800 to +1800)
     * @param roll Roll angle, unit: 0.1° (range: -1800 to +1800)
     * @param pitch Pitch angle, unit: 0.1° (range: -1800 to +1800)
     * @param time_ms Command execution time, unit: ms (100 .. 25500).
     * Time is used to set motion speed when gimbal is executing this command.
     * @return True if the gimbal accepted the command
     */
    bool move_to(int16_t yaw, int16_t roll, int16_t pitch, uint16_t time_ms);

    /**
     * @brief move_axes - Position control with explicit mode and per-axis
     * enable. Disabled axes are ignored by the gimbal (keep their position).
     */
    bool move_axes(int16_t yaw, int16_t roll, int16_t pitch, uint16_t time_ms,
                   MoveMode mode, bool yaw_enabled, bool roll_enabled, bool pitch_enabled);

    /**
     * @brief set_axis_enabled - Enable/disable an axis for move_to()
     * (the "axis valid" bits of 2.3.4.1).
     */
    bool set_axis_enabled(AxisType axis, bool enabled);

    /// @deprecated Misnamed: the protocol bits mean "axis invalid", not
    /// "invert". set_inverted_axis(axis, true) == set_axis_enabled(axis, false).
    bool set_inverted_axis(AxisType axis, bool invert);

    /**
     * @brief set_move_mode Set move mode
     * @param type INCREMENTAL or ABSOLUTE
     * @return True if success
     */
    bool set_move_mode(MoveMode type);

    /**
     * @brief set_speed - Handheld Gimbal Speed Control (p.6, 2.3.4.2)
     * @details This command can only control for 0.5s each time it is issued
     * due to safety reasons. If users require continuous speed, they can send
     * this command periodically. Send 0,0,0 to stop immediately.
     * @param yaw Unit: 0.1°/s (range: -3600 to 3600)
     * @param roll Unit: 0.1°/s
     * @param pitch Unit: 0.1°/s
     * @return True if the gimbal accepted the command
     */
    bool set_speed(int16_t yaw, int16_t roll, int16_t pitch);

    /**
     * @brief set_speed_mode - Handheld Gimbal Speed Control (p.6, 2.3.4.2)
     * @param speed_type ENABLED: take over speed control, DISABLED: release it
     * @param focal_type Enable or Disable the impact of camera focal length
     * into consideration
     * @return True if success
     */
    bool set_speed_mode(SpeedControl speed_type, FocalControl focal_type);

    /**
     * @brief get_current_position - Attitude angles (p.6, 2.3.4.3)
     * Same as get_attitude_angles(). Unit: 0.1°.
     */
    bool get_current_position(int16_t& yaw, int16_t& roll, int16_t& pitch);

    /// Attitude (world-frame) angles, unit 0.1°.
    bool get_attitude_angles(int16_t& yaw, int16_t& roll, int16_t& pitch);
    /// Joint (motor encoder) angles, unit 0.1°.
    bool get_joint_angles(int16_t& yaw, int16_t& roll, int16_t& pitch);

    /// 2.3.4.15: execute Recenter once.
    bool recenter();

    /// 2.3.4.16: start/stop gimbal auto calibration (motor self-tuning).
    /// Progress is pushed back; read it with get_calibration_status().
    bool set_auto_calibration(bool start, bool single_attitude_mode = false);
    CalibrationStatus get_calibration_status();

    /// 2.3.4.8: enable/disable the periodic parameter push (0x0E/0x08).
    bool set_push_enabled(bool enabled);
    /// Latest parameter push. Returns false if none was received yet.
    bool get_push_data(GimbalPushData& data);

    /// 2.3.4.10: module version, 0xAABBCCDD == AA.BB.CC.DD.
    bool get_version(uint32_t& version);

    LinkStats get_link_stats();
    ReturnCode last_return_code() const { return _last_rc; }

private:
    /// Send a command and wait for the reply payload (DATA after CmdSet/CmdID).
    bool request(uint8_t cmd_set, uint8_t cmd_id, const std::vector<uint8_t>& payload,
                 std::vector<uint8_t>* reply_payload);
    bool get_angles(uint8_t type, int16_t& yaw, int16_t& roll, int16_t& pitch);

    void* _can_conn;
    void* _pack_thread;
    uint8_t _position_ctrl_byte;
    uint8_t _speed_ctrl_byte;
    void* _cmd_cmb;
    std::mutex _cmd_mutex;
    int _reply_timeout_ms;
    std::atomic<ReturnCode> _last_rc;
};


}


#endif //DJI_R_SDK_H
