// slimevr_loopback — dump OSC datagrams received on a UDP port.
//
// Pair with `./cpp/build/main ... --slimevr-out --slimevr-port N` for a
// SlimeVR-Server-free smoke test: confirms that the publisher is sending
// well-formed VMC bundles at the requested rate and lets the operator
// eyeball position drift across body movements.
//
// The decoder is intentionally minimal -- it understands the subset that
// `slimevr::OscWriter` emits: bundles of messages with typetag chars from
// {`s`, `f`, `i`}. Unknown bytes are reported as a hex dump rather than
// parsed, so structural drift is loud rather than silent.

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <string>
#include <string_view>
#include <vector>

namespace {

std::atomic<bool> g_stop{false};
void on_signal(int) { g_stop.store(true); }

std::uint32_t be_load_u32(const std::uint8_t* p) {
    return (static_cast<std::uint32_t>(p[0]) << 24) |
           (static_cast<std::uint32_t>(p[1]) << 16) |
           (static_cast<std::uint32_t>(p[2]) <<  8) |
            static_cast<std::uint32_t>(p[3]);
}

float be_load_f32(const std::uint8_t* p) {
    std::uint32_t u = be_load_u32(p);
    float f;
    std::memcpy(&f, &u, sizeof(f));
    return f;
}

std::uint64_t be_load_u64(const std::uint8_t* p) {
    return (static_cast<std::uint64_t>(be_load_u32(p)) << 32) |
            static_cast<std::uint64_t>(be_load_u32(p + 4));
}

// Read a 4-byte-aligned NUL-terminated string starting at `p`. On success,
// returns the string view and advances `p` to the first byte past the
// padding. Returns false if `p` would overrun `end`.
bool read_osc_string(const std::uint8_t*& p, const std::uint8_t* end,
                     std::string_view& out) {
    const std::uint8_t* start = p;
    while (p < end && *p != 0) ++p;
    if (p >= end) return false;
    out = std::string_view{reinterpret_cast<const char*>(start),
                           static_cast<std::size_t>(p - start)};
    ++p;  // skip NUL
    std::size_t total = static_cast<std::size_t>(p - start);
    std::size_t pad = (4 - (total % 4)) % 4;
    if (p + pad > end) return false;
    p += pad;
    return true;
}

void dump_message(const std::uint8_t* buf, std::size_t n) {
    const std::uint8_t* p = buf;
    const std::uint8_t* end = buf + n;
    std::string_view address;
    if (!read_osc_string(p, end, address)) {
        std::printf("    [malformed message: bad address]\n");
        return;
    }
    std::string_view typetag;
    if (!read_osc_string(p, end, typetag)) {
        std::printf("    [malformed message: bad typetag]\n");
        return;
    }
    std::printf("    %.*s  %.*s ",
                static_cast<int>(address.size()), address.data(),
                static_cast<int>(typetag.size()), typetag.data());

    // Skip the leading ','.
    if (typetag.empty() || typetag[0] != ',') {
        std::printf("[bad typetag: no leading ',']\n");
        return;
    }
    for (std::size_t i = 1; i < typetag.size(); ++i) {
        char t = typetag[i];
        if (t == 's') {
            std::string_view s;
            if (!read_osc_string(p, end, s)) {
                std::printf("[bad string arg]"); break;
            }
            std::printf("\"%.*s\" ", static_cast<int>(s.size()), s.data());
        } else if (t == 'f') {
            if (p + 4 > end) { std::printf("[bad float arg]"); break; }
            float v = be_load_f32(p); p += 4;
            std::printf("%.4f ", static_cast<double>(v));
        } else if (t == 'i') {
            if (p + 4 > end) { std::printf("[bad int arg]"); break; }
            std::int32_t v = static_cast<std::int32_t>(be_load_u32(p)); p += 4;
            std::printf("%d ", v);
        } else {
            std::printf("[unknown typetag '%c']", t);
            break;
        }
    }
    std::printf("\n");
}

void dump_packet(const std::uint8_t* buf, std::size_t n, std::uint64_t pkt_num) {
    if (n >= 8 && std::memcmp(buf, "#bundle", 7) == 0 && buf[7] == 0) {
        if (n < 16) { std::printf("[%llu] short bundle\n", static_cast<unsigned long long>(pkt_num)); return; }
        std::uint64_t ts = be_load_u64(buf + 8);
        std::printf("[%llu] bundle ts=0x%016llx (%zu bytes)\n",
                    static_cast<unsigned long long>(pkt_num),
                    static_cast<unsigned long long>(ts), n);
        const std::uint8_t* p = buf + 16;
        const std::uint8_t* end = buf + n;
        while (p + 4 <= end) {
            std::uint32_t sz = be_load_u32(p);
            p += 4;
            if (sz == 0 || p + sz > end) {
                std::printf("    [malformed element size=%u]\n", sz);
                break;
            }
            dump_message(p, sz);
            p += sz;
        }
    } else {
        std::printf("[%llu] non-bundle packet (%zu bytes)\n",
                    static_cast<unsigned long long>(pkt_num), n);
        dump_message(buf, n);
    }
}

void print_help() {
    std::puts(
        "slimevr_loopback — dump OSC datagrams from a UDP port\n"
        "\n"
        "Pair with `./cpp/build/main ... --slimevr-out` for a SlimeVR-Server-\n"
        "free smoke test of the VMC publisher.\n"
        "\n"
        "Usage:\n"
        "  slimevr_loopback [--port N] [--seconds N] [--bind ADDR]\n"
        "\n"
        "Options:\n"
        "  --port N        UDP port (default 39539)\n"
        "  --bind ADDR     bind address (default 0.0.0.0)\n"
        "  --seconds N     auto-exit after N seconds (default 30, 0 = run\n"
        "                  until Ctrl-C)\n"
        "  --help          show this help\n");
}

}  // namespace

int main(int argc, char** argv) {
    int         port    = 39539;
    int         seconds = 30;
    std::string bind_addr = "0.0.0.0";

    for (int i = 1; i < argc; ++i) {
        std::string_view a{argv[i]};
        auto need = [&](const char* flag) -> const char* {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "missing argument for %s\n", flag);
                std::exit(EXIT_FAILURE);
            }
            return argv[++i];
        };
        if      (a == "--help" || a == "-h") { print_help(); return EXIT_SUCCESS; }
        else if (a == "--port")              { port = std::atoi(need("--port")); }
        else if (a == "--bind")              { bind_addr = need("--bind"); }
        else if (a == "--seconds")           { seconds = std::atoi(need("--seconds")); }
        else {
            std::fprintf(stderr, "unknown arg: %s\n", argv[i]);
            print_help();
            return EXIT_FAILURE;
        }
    }
    if (port <= 0 || port > 65535) {
        std::fprintf(stderr, "--port must be in [1, 65535]\n");
        return EXIT_FAILURE;
    }
    if (seconds < 0) {
        std::fprintf(stderr, "--seconds must be >= 0\n");
        return EXIT_FAILURE;
    }

    int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        std::fprintf(stderr, "socket() failed: %s\n", std::strerror(errno));
        return EXIT_FAILURE;
    }
    sockaddr_in sa{};
    sa.sin_family = AF_INET;
    sa.sin_port   = htons(static_cast<std::uint16_t>(port));
    if (::inet_pton(AF_INET, bind_addr.c_str(), &sa.sin_addr) != 1) {
        std::fprintf(stderr, "inet_pton failed for %s\n", bind_addr.c_str());
        ::close(fd);
        return EXIT_FAILURE;
    }
    if (::bind(fd, reinterpret_cast<sockaddr*>(&sa), sizeof(sa)) < 0) {
        std::fprintf(stderr, "bind() failed on %s:%d: %s\n",
                     bind_addr.c_str(), port, std::strerror(errno));
        ::close(fd);
        return EXIT_FAILURE;
    }

    std::printf("listening on %s:%d (timeout=%d sec, Ctrl-C to exit)\n",
                bind_addr.c_str(), port, seconds);

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    std::vector<std::uint8_t> buf(65536);
    auto start = std::chrono::steady_clock::now();
    std::uint64_t pkt_count = 0;

    while (!g_stop.load()) {
        if (seconds > 0) {
            auto elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - start).count();
            if (elapsed >= static_cast<double>(seconds)) break;
        }
        pollfd pfd{fd, POLLIN, 0};
        int pr = ::poll(&pfd, 1, 200);
        if (pr <= 0) continue;
        ssize_t n = ::recv(fd, buf.data(), buf.size(), 0);
        if (n <= 0) continue;
        dump_packet(buf.data(), static_cast<std::size_t>(n), pkt_count++);
    }

    auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
    std::printf("received %llu packets in %.2f s (%.1f pkt/s)\n",
                static_cast<unsigned long long>(pkt_count), elapsed,
                pkt_count / std::max(elapsed, 1.0e-9));
    ::close(fd);
    return EXIT_SUCCESS;
}
