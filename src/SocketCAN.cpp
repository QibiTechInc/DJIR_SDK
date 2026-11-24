#include "SocketCAN.h"

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstring>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

DJIR_SDK::SocketCAN::SocketCAN(uint32_t send_id, uint32_t recv_id, const std::string& ifname)
    : _send_id(send_id), _recv_id(recv_id), _ifname(ifname), _fd(-1)
{
}

DJIR_SDK::SocketCAN::~SocketCAN()
{
    close();
}

bool DJIR_SDK::SocketCAN::open()
{
    if (is_open())
        return true;

    int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0)
    {
        perror("SocketCAN: socket");
        return false;
    }

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, _ifname.c_str(), IFNAMSIZ - 1);
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0)
    {
        fprintf(stderr, "SocketCAN: interface '%s' not found: %s\n",
                _ifname.c_str(), strerror(errno));
        ::close(fd);
        return false;
    }

    // Only receive standard data frames from the gimbal's id.
    struct can_filter filter;
    filter.can_id = _recv_id;
    filter.can_mask = CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG;
    setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter));

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0)
    {
        perror("SocketCAN: bind");
        ::close(fd);
        return false;
    }

    _fd = fd;
    return true;
}

void DJIR_SDK::SocketCAN::close()
{
    int fd = _fd.exchange(-1);
    if (fd >= 0)
        ::close(fd);
}

bool DJIR_SDK::SocketCAN::is_open() const
{
    return _fd >= 0;
}

int DJIR_SDK::SocketCAN::send_cmd(const std::vector<uint8_t>& cmd)
{
    int fd = _fd;
    if (fd < 0)
        return 0;

    std::lock_guard<std::mutex> lock(_send_mutex);
    int frames = 0;
    for (size_t off = 0; off < cmd.size(); off += CAN_MAX_DLEN)
    {
        struct can_frame frame;
        memset(&frame, 0, sizeof(frame));
        frame.can_id = _send_id;
        frame.can_dlc = (uint8_t)std::min<size_t>(CAN_MAX_DLEN, cmd.size() - off);
        memcpy(frame.data, cmd.data() + off, frame.can_dlc);

        // Retry briefly if the TX queue is full.
        int retries = 50;
        ssize_t n;
        while ((n = write(fd, &frame, sizeof(frame))) < 0 &&
               (errno == ENOBUFS || errno == EAGAIN) && retries-- > 0)
            usleep(1000);

        if (n != (ssize_t)sizeof(frame))
        {
            perror("SocketCAN: write");
            return 0;
        }
        frames++;
    }
    return frames;
}

std::vector<uint8_t> DJIR_SDK::SocketCAN::recv_frame(int timeout_ms)
{
    int fd = _fd;
    if (fd < 0)
        return std::vector<uint8_t>();

    struct pollfd pfd;
    pfd.fd = fd;
    pfd.events = POLLIN;
    pfd.revents = 0;
    if (poll(&pfd, 1, timeout_ms) <= 0 || !(pfd.revents & POLLIN))
        return std::vector<uint8_t>();

    struct can_frame frame;
    ssize_t n = read(fd, &frame, sizeof(frame));
    if (n != (ssize_t)sizeof(frame))
        return std::vector<uint8_t>();
    if ((frame.can_id & CAN_SFF_MASK) != _recv_id || (frame.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG)))
        return std::vector<uint8_t>();

    return std::vector<uint8_t>(frame.data, frame.data + frame.can_dlc);
}
