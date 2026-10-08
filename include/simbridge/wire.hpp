// Binary wire format for sending bus samples to other processes.
//
// Every packet starts with a 12 byte little endian header:
//   u32 magic ("SBRG")  u16 version  u16 message type  u32 payload length
// followed by the payload. Doubles are IEEE 754, little endian.
// tools/autonomy_listener.py decodes the same layout in Python.
#pragma once

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

#include "simbridge/messages.hpp"

namespace simbridge::wire {

inline constexpr uint32_t kMagic = 0x47524253;  // bytes on the wire: 'S' 'B' 'R' 'G'
inline constexpr uint16_t kVersion = 1;
inline constexpr std::size_t kHeaderSize = 12;

enum class MsgType : uint16_t { EntityState = 1, Detection = 2 };

class WireError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

std::vector<uint8_t> encode(const EntityState& msg);
std::vector<uint8_t> encode(const Detection& msg);

// Validates the header and returns the message type.
MsgType peek_type(const uint8_t* data, std::size_t size);

EntityState decode_entity_state(const uint8_t* data, std::size_t size);
Detection decode_detection(const uint8_t* data, std::size_t size);

}  // namespace simbridge::wire
