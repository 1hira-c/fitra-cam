#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <string>

namespace fitra::util {

// Small dependency-free SHA-256 used by the corpus spool commit protocol.
// Throws std::runtime_error when a file cannot be read.
std::string sha256_hex(const std::uint8_t* data, std::size_t size);
std::string sha256_file(const std::filesystem::path& path);
// Returns false when should_continue becomes false at a 1 MiB read boundary.
// I/O failures still throw so callers can distinguish pause from corruption.
bool sha256_file_interruptible(const std::filesystem::path& path,
                               const std::function<bool()>& should_continue,
                               std::string& digest_out);

}  // namespace fitra::util
