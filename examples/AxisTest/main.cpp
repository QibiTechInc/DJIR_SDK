// Rotates the gimbal one axis at a time (yaw, then roll, then pitch).
//
// For each axis: +amplitude, back to start, -amplitude, back to start.
// Uses INCREMENTAL_CONTROL, so every move is relative to the current pose.
//
// Usage: DJIR_AxisTest [can_interface] [amplitude_deg] [move_time_ms]
//   defaults: can0 15 1000

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#include "DJIR_SDK.h"
using namespace DJIR_SDK;

static void print_pos(DJIRonin& gimbal, const char* label)
{
    int16_t yaw = 0, roll = 0, pitch = 0;
    if (gimbal.get_current_position(yaw, roll, pitch))
        std::cout << "  " << label
                  << "  yaw=" << yaw / 10.0
                  << "  roll=" << roll / 10.0
                  << "  pitch=" << pitch / 10.0 << " [deg]" << std::endl;
    else
        std::cout << "  " << label << "  (no position reply)" << std::endl;
}

// Send one incremental move on a single axis and wait for it to finish.
static void step(DJIRonin& gimbal, AxisType axis, int16_t delta, uint16_t time_ms)
{
    int16_t yaw = 0, roll = 0, pitch = 0;
    switch (axis)
    {
    case AxisType::YAW:   yaw = delta;   break;
    case AxisType::ROLL:  roll = delta;  break;
    case AxisType::PITCH: pitch = delta; break;
    }

    if (!gimbal.move_to(yaw, roll, pitch, time_ms))
        std::cout << "  move_to failed" << std::endl;

    // Wait for the motion plus a short settle time.
    std::this_thread::sleep_for(std::chrono::milliseconds(time_ms + 500));

    char label[32];
    snprintf(label, sizeof(label), "%+6.1f deg ->", delta / 10.0);
    print_pos(gimbal, label);
}

int main(int argc, char** argv)
{
    std::string can_interface = (argc > 1) ? argv[1] : "can0";
    double amplitude_deg      = (argc > 2) ? std::atof(argv[2]) : 15.0;
    int move_time_ms          = (argc > 3) ? std::atoi(argv[3]) : 1000;

    if (amplitude_deg <= 0 || amplitude_deg > 90)
    {
        std::cerr << "amplitude_deg must be in (0, 90]" << std::endl;
        return 1;
    }
    if (move_time_ms < 100)
        move_time_ms = 100;

    int16_t amp = (int16_t)(amplitude_deg * 10.0);  // unit: 0.1 deg

    DJIRonin gimbal;
    if (!gimbal.connect(can_interface))
    {
        std::cerr << "Failed to open " << can_interface << ". Bring it up with:\n"
                  << "  sudo ip link set " << can_interface << " up type can bitrate 1000000"
                  << std::endl;
        return 1;
    }

    gimbal.set_move_mode(MoveMode::INCREMENTAL_CONTROL);

    std::cout << "Connected via " << can_interface
              << ", amplitude " << amplitude_deg << " deg, move time "
              << move_time_ms << " ms" << std::endl;
    print_pos(gimbal, "start      ");

    const struct { AxisType axis; const char* name; } axes[] = {
        { AxisType::YAW,   "YAW"   },
        { AxisType::ROLL,  "ROLL"  },
        { AxisType::PITCH, "PITCH" },
    };

    for (const auto& a : axes)
    {
        std::cout << "\n=== " << a.name << " ===" << std::endl;
        step(gimbal, a.axis, +amp, (uint16_t)move_time_ms);
        step(gimbal, a.axis, -amp, (uint16_t)move_time_ms);
        step(gimbal, a.axis, -amp, (uint16_t)move_time_ms);
        step(gimbal, a.axis, +amp, (uint16_t)move_time_ms);
    }

    std::cout << std::endl;
    print_pos(gimbal, "end        ");
    gimbal.disconnect();
    return 0;
}
