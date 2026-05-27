#include "osc_send.hpp"

#include <osc/OscOutboundPacketStream.h>
#include <ip/UdpSocket.h>
#include <ip/IpEndpointName.h>

#include <array>
#include <memory>
#include <stdexcept>
#include <string>

namespace fitra::hmd_sender {

namespace {
constexpr int kBufferBytes = 256;  // /fitra/hmd_pose fits easily in 64 byte
constexpr const char* kAddress = "/fitra/hmd_pose";
}  // namespace

struct OscSender::Impl {
    std::unique_ptr<UdpTransmitSocket> sock;
    std::array<char, kBufferBytes>     buf{};
};

OscSender::OscSender() : impl_(new Impl()) {}

OscSender::~OscSender() { delete impl_; }

bool OscSender::open(const std::string& host, std::uint16_t port) {
    try {
        IpEndpointName ep(host.c_str(), static_cast<int>(port));
        if (ep.address == IpEndpointName::ANY_ADDRESS) {
            last_error_ = "failed to resolve host: " + host;
            return false;
        }
        impl_->sock = std::make_unique<UdpTransmitSocket>(ep);
        last_error_.clear();
        return true;
    } catch (const std::exception& e) {
        last_error_ = std::string("open failed: ") + e.what();
        impl_->sock.reset();
        return false;
    }
}

bool OscSender::send(const HmdSample& s) {
    if (!impl_->sock) {
        last_error_ = "socket not open";
        return false;
    }
    try {
        osc::OutboundPacketStream p(impl_->buf.data(), kBufferBytes);
        p << osc::BeginMessage(kAddress)
          << static_cast<osc::int32>(s.valid ? 1 : 0)
          << s.timestamp_s
          << s.x << s.y << s.z
          << s.qx << s.qy << s.qz << s.qw
          << osc::EndMessage;
        impl_->sock->Send(p.Data(), p.Size());
        return true;
    } catch (const std::exception& e) {
        last_error_ = std::string("send failed: ") + e.what();
        return false;
    }
}

}  // namespace fitra::hmd_sender
