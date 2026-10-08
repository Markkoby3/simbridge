// legacy::BlockSim stands in for an older simulation framework that programs
// still depend on. Its API is intentionally unlike SimBridge's:
//
//   * imperial units: feet for position, knots for speed
//   * NED style axes: north, east, altitude
//   * compass headings in degrees: 0 = north, increasing clockwise
//   * an integer millisecond clock
//   * its own block ids, assigned sequentially, unrelated to scenario ids
//
// Nothing here knows SimBridge exists. The LegacyBlocksAdapter does all the
// translation, the same way a real adapter wraps a framework you cannot change.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace legacy {

struct BlockState {
    int block_id = 0;
    std::string label;
    double north_ft = 0, east_ft = 0, alt_ft = 0;
    double heading_deg = 0;  // compass
    double speed_kts = 0;
};

class BlockSim {
public:
    BlockSim(double max_turn_deg_per_s, double max_accel_kts_per_s);

    int add_block(const std::string& label, double north_ft, double east_ft, double alt_ft, double heading_deg,
                  double speed_kts);
    void set_heading_deg(int block_id, double heading_deg);
    void set_speed_kts(int block_id, double speed_kts);
    void tick_ms(int64_t ms);

    int64_t clock_ms() const { return clock_ms_; }
    const std::vector<BlockState>& blocks() const { return blocks_; }

private:
    struct Command {
        double heading_deg = 0, speed_kts = 0;
    };
    double max_turn_dps_;
    double max_accel_ktps_;
    std::vector<BlockState> blocks_;
    std::vector<Command> commands_;
    int64_t clock_ms_ = 0;
};

}  // namespace legacy
