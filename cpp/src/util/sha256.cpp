#include "util/sha256.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace fitra::util {

namespace {

constexpr std::array<std::uint32_t, 64> kRound{
    0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U,
    0x3956c25bU, 0x59f111f1U, 0x923f82a4U, 0xab1c5ed5U,
    0xd807aa98U, 0x12835b01U, 0x243185beU, 0x550c7dc3U,
    0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U, 0xc19bf174U,
    0xe49b69c1U, 0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU,
    0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU,
    0x983e5152U, 0xa831c66dU, 0xb00327c8U, 0xbf597fc7U,
    0xc6e00bf3U, 0xd5a79147U, 0x06ca6351U, 0x14292967U,
    0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU, 0x53380d13U,
    0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U,
    0xa2bfe8a1U, 0xa81a664bU, 0xc24b8b70U, 0xc76c51a3U,
    0xd192e819U, 0xd6990624U, 0xf40e3585U, 0x106aa070U,
    0x19a4c116U, 0x1e376c08U, 0x2748774cU, 0x34b0bcb5U,
    0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU, 0x682e6ff3U,
    0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U,
    0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U,
};

constexpr std::uint32_t rotr(std::uint32_t x, unsigned n) {
    return (x >> n) | (x << (32U - n));
}

class Sha256 {
public:
    void update(const std::uint8_t* data, std::size_t size) {
        total_bytes_ += size;
        while (size > 0) {
            const std::size_t n = std::min(size, block_.size() - block_size_);
            std::memcpy(block_.data() + block_size_, data, n);
            block_size_ += n;
            data += n;
            size -= n;
            if (block_size_ == block_.size()) {
                transform(block_.data());
                block_size_ = 0;
            }
        }
    }

    std::array<std::uint8_t, 32> finish() {
        const std::uint64_t bits = total_bytes_ * 8U;
        block_[block_size_++] = 0x80U;
        if (block_size_ > 56) {
            std::fill(block_.begin() + static_cast<std::ptrdiff_t>(block_size_),
                      block_.end(), 0U);
            transform(block_.data());
            block_size_ = 0;
        }
        std::fill(block_.begin() + static_cast<std::ptrdiff_t>(block_size_),
                  block_.begin() + 56, 0U);
        for (int i = 0; i < 8; ++i) {
            block_[63 - i] = static_cast<std::uint8_t>(bits >> (8 * i));
        }
        transform(block_.data());

        std::array<std::uint8_t, 32> out{};
        for (std::size_t i = 0; i < state_.size(); ++i) {
            out[i * 4 + 0] = static_cast<std::uint8_t>(state_[i] >> 24);
            out[i * 4 + 1] = static_cast<std::uint8_t>(state_[i] >> 16);
            out[i * 4 + 2] = static_cast<std::uint8_t>(state_[i] >> 8);
            out[i * 4 + 3] = static_cast<std::uint8_t>(state_[i]);
        }
        return out;
    }

private:
    void transform(const std::uint8_t* p) {
        std::array<std::uint32_t, 64> w{};
        for (std::size_t i = 0; i < 16; ++i) {
            w[i] = (static_cast<std::uint32_t>(p[i * 4 + 0]) << 24)
                 | (static_cast<std::uint32_t>(p[i * 4 + 1]) << 16)
                 | (static_cast<std::uint32_t>(p[i * 4 + 2]) << 8)
                 | static_cast<std::uint32_t>(p[i * 4 + 3]);
        }
        for (std::size_t i = 16; i < w.size(); ++i) {
            const auto s0 = rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18)
                          ^ (w[i - 15] >> 3);
            const auto s1 = rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19)
                          ^ (w[i - 2] >> 10);
            w[i] = w[i - 16] + s0 + w[i - 7] + s1;
        }

        auto a = state_[0];
        auto b = state_[1];
        auto c = state_[2];
        auto d = state_[3];
        auto e = state_[4];
        auto f = state_[5];
        auto g = state_[6];
        auto h = state_[7];
        for (std::size_t i = 0; i < w.size(); ++i) {
            const auto s1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
            const auto ch = (e & f) ^ ((~e) & g);
            const auto t1 = h + s1 + ch + kRound[i] + w[i];
            const auto s0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
            const auto maj = (a & b) ^ (a & c) ^ (b & c);
            const auto t2 = s0 + maj;
            h = g;
            g = f;
            f = e;
            e = d + t1;
            d = c;
            c = b;
            b = a;
            a = t1 + t2;
        }
        state_[0] += a;
        state_[1] += b;
        state_[2] += c;
        state_[3] += d;
        state_[4] += e;
        state_[5] += f;
        state_[6] += g;
        state_[7] += h;
    }

    std::array<std::uint32_t, 8> state_{
        0x6a09e667U, 0xbb67ae85U, 0x3c6ef372U, 0xa54ff53aU,
        0x510e527fU, 0x9b05688cU, 0x1f83d9abU, 0x5be0cd19U,
    };
    std::array<std::uint8_t, 64> block_{};
    std::size_t block_size_ = 0;
    std::uint64_t total_bytes_ = 0;
};

std::string hex_digest(const std::array<std::uint8_t, 32>& digest) {
    std::ostringstream out;
    out << std::hex << std::setfill('0');
    for (const auto b : digest) out << std::setw(2) << static_cast<unsigned>(b);
    return out.str();
}

}  // namespace

std::string sha256_hex(const std::uint8_t* data, std::size_t size) {
    Sha256 sha;
    sha.update(data, size);
    return hex_digest(sha.finish());
}

std::string sha256_file(const std::filesystem::path& path) {
    std::string digest;
    (void)sha256_file_interruptible(path, [] { return true; }, digest);
    return digest;
}

bool sha256_file_interruptible(const std::filesystem::path& path,
                               const std::function<bool()>& should_continue,
                               std::string& digest_out) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::runtime_error("cannot open for SHA-256: " + path.string());
    Sha256 sha;
    std::array<char, 1024 * 1024> buf{};
    while (in) {
        if (!should_continue()) return false;
        in.read(buf.data(), static_cast<std::streamsize>(buf.size()));
        const auto n = in.gcount();
        if (n > 0) {
            sha.update(reinterpret_cast<const std::uint8_t*>(buf.data()),
                       static_cast<std::size_t>(n));
        }
    }
    if (!in.eof()) throw std::runtime_error("cannot read for SHA-256: " + path.string());
    if (!should_continue()) return false;
    digest_out = hex_digest(sha.finish());
    return true;
}

}  // namespace fitra::util
