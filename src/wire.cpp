#include "simbridge/wire.hpp"

#include <cstring>
#include <string>

namespace simbridge::wire {

namespace {

class Writer {
public:
    void u16(uint16_t v) { put(v, 2); }
    void u32(uint32_t v) { put(v, 4); }
    void f64(double v) {
        uint64_t bits;
        std::memcpy(&bits, &v, sizeof bits);
        put(bits, 8);
    }
    void bytes(const std::string& s) { buf.insert(buf.end(), s.begin(), s.end()); }
    std::vector<uint8_t> buf;

private:
    void put(uint64_t v, int n) {
        for (int i = 0; i < n; ++i) buf.push_back(static_cast<uint8_t>(v >> (8 * i)));
    }
};

class Reader {
public:
    Reader(const uint8_t* data, std::size_t size) : p_(data), end_(data + size) {}
    uint16_t u16() { return static_cast<uint16_t>(get(2)); }
    uint32_t u32() { return static_cast<uint32_t>(get(4)); }
    double f64() {
        const uint64_t bits = get(8);
        double v;
        std::memcpy(&v, &bits, sizeof v);
        return v;
    }
    std::string bytes(std::size_t n) {
        need(n);
        std::string s(reinterpret_cast<const char*>(p_), n);
        p_ += n;
        return s;
    }
    bool done() const { return p_ == end_; }

private:
    void need(std::size_t n) const {
        if (static_cast<std::size_t>(end_ - p_) < n) throw WireError("packet truncated");
    }
    uint64_t get(int n) {
        need(static_cast<std::size_t>(n));
        uint64_t v = 0;
        for (int i = 0; i < n; ++i) v |= static_cast<uint64_t>(p_[i]) << (8 * i);
        p_ += n;
        return v;
    }
    const uint8_t* p_;
    const uint8_t* end_;
};

std::vector<uint8_t> frame(MsgType type, const std::vector<uint8_t>& payload) {
    Writer w;
    w.u32(kMagic);
    w.u16(kVersion);
    w.u16(static_cast<uint16_t>(type));
    w.u32(static_cast<uint32_t>(payload.size()));
    w.buf.insert(w.buf.end(), payload.begin(), payload.end());
    return w.buf;
}

Reader open_payload(const uint8_t* data, std::size_t size, MsgType expected) {
    if (peek_type(data, size) != expected) throw WireError("unexpected message type");
    return Reader(data + kHeaderSize, size - kHeaderSize);
}

}  // namespace

std::vector<uint8_t> encode(const EntityState& m) {
    if (m.name.size() > 0xFFFF) throw WireError("entity name too long");
    Writer w;
    w.u32(m.id);
    w.f64(m.t);
    w.f64(m.pos.x);
    w.f64(m.pos.y);
    w.f64(m.pos.z);
    w.f64(m.vel.x);
    w.f64(m.vel.y);
    w.f64(m.vel.z);
    w.f64(m.heading_rad);
    w.f64(m.speed_mps);
    w.u16(static_cast<uint16_t>(m.name.size()));
    w.bytes(m.name);
    return frame(MsgType::EntityState, w.buf);
}

std::vector<uint8_t> encode(const Detection& m) {
    Writer w;
    w.u32(m.sensor_id);
    w.u32(m.host_id);
    w.u32(m.target_id);
    w.f64(m.t);
    w.f64(m.range_m);
    w.f64(m.bearing_rad);
    return frame(MsgType::Detection, w.buf);
}

MsgType peek_type(const uint8_t* data, std::size_t size) {
    Reader r(data, size);
    if (r.u32() != kMagic) throw WireError("bad magic");
    const uint16_t version = r.u16();
    if (version != kVersion) throw WireError("unsupported wire version " + std::to_string(version));
    const uint16_t type = r.u16();
    const uint32_t len = r.u32();
    if (len != size - kHeaderSize) throw WireError("payload length mismatch");
    if (type != 1 && type != 2) throw WireError("unknown message type " + std::to_string(type));
    return static_cast<MsgType>(type);
}

EntityState decode_entity_state(const uint8_t* data, std::size_t size) {
    Reader r = open_payload(data, size, MsgType::EntityState);
    EntityState m;
    m.id = r.u32();
    m.t = r.f64();
    m.pos = Vec3{r.f64(), r.f64(), r.f64()};
    m.vel = Vec3{r.f64(), r.f64(), r.f64()};
    m.heading_rad = r.f64();
    m.speed_mps = r.f64();
    m.name = r.bytes(r.u16());
    if (!r.done()) throw WireError("trailing bytes after entity state");
    return m;
}

Detection decode_detection(const uint8_t* data, std::size_t size) {
    Reader r = open_payload(data, size, MsgType::Detection);
    Detection m;
    m.sensor_id = r.u32();
    m.host_id = r.u32();
    m.target_id = r.u32();
    m.t = r.f64();
    m.range_m = r.f64();
    m.bearing_rad = r.f64();
    if (!r.done()) throw WireError("trailing bytes after detection");
    return m;
}

}  // namespace simbridge::wire
