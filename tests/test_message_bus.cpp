#include <gtest/gtest.h>

#include <atomic>
#include <thread>
#include <vector>

#include "simbridge/message_bus.hpp"
#include "simbridge/messages.hpp"

using namespace simbridge;

TEST(MessageBus, DeliversToEverySubscriber) {
    MessageBus bus;
    int a = 0, b = 0;
    bus.subscribe<Detection>("det", [&](const Detection& d) { a += static_cast<int>(d.target_id); });
    bus.subscribe<Detection>("det", [&](const Detection& d) { b += static_cast<int>(d.target_id); });
    Detection d;
    d.target_id = 7;
    bus.publish("det", d);
    EXPECT_EQ(a, 7);
    EXPECT_EQ(b, 7);
    EXPECT_EQ(bus.published_count(), 1u);
}

TEST(MessageBus, TopicsAreIsolated) {
    MessageBus bus;
    int hits = 0;
    bus.subscribe<Detection>("a", [&](const Detection&) { ++hits; });
    bus.publish("b", Detection{});
    EXPECT_EQ(hits, 0);
}

TEST(MessageBus, RejectsTypeMismatch) {
    MessageBus bus;
    bus.declare<EntityState>("state");
    EXPECT_THROW(bus.publish("state", Detection{}), TopicTypeError);
    EXPECT_THROW(bus.subscribe<VehicleCommand>("state", [](const VehicleCommand&) {}), TopicTypeError);
}

TEST(MessageBus, HistoryReplaysLastNToLateJoiners) {
    MessageBus bus;
    bus.declare<VehicleCommand>("cmd", TopicQos{2});
    for (uint32_t i = 1; i <= 5; ++i) {
        VehicleCommand c;
        c.entity_id = i;
        bus.publish("cmd", c);
    }
    std::vector<uint32_t> seen;
    bus.subscribe<VehicleCommand>("cmd", [&](const VehicleCommand& c) { seen.push_back(c.entity_id); });
    EXPECT_EQ(seen, (std::vector<uint32_t>{4, 5}));
}

TEST(MessageBus, VolatileTopicKeepsNoHistory) {
    MessageBus bus;
    bus.publish("cmd", VehicleCommand{});
    int hits = 0;
    bus.subscribe<VehicleCommand>("cmd", [&](const VehicleCommand&) { ++hits; });
    EXPECT_EQ(hits, 0);
}

TEST(MessageBus, UnsubscribeStopsDelivery) {
    MessageBus bus;
    int hits = 0;
    const auto id = bus.subscribe<Detection>("det", [&](const Detection&) { ++hits; });
    bus.publish("det", Detection{});
    EXPECT_TRUE(bus.unsubscribe(id));
    EXPECT_FALSE(bus.unsubscribe(id));
    bus.publish("det", Detection{});
    EXPECT_EQ(hits, 1);
    EXPECT_EQ(bus.subscriber_count("det"), 0u);
}

TEST(MessageBus, CallbackMayPublishWithoutDeadlock) {
    MessageBus bus;
    int relayed = 0;
    bus.subscribe<Detection>("in", [&](const Detection& d) { bus.publish("out", d); });
    bus.subscribe<Detection>("out", [&](const Detection&) { ++relayed; });
    bus.publish("in", Detection{});
    EXPECT_EQ(relayed, 1);
}

TEST(MessageBus, ConcurrentPublishersLoseNothing) {
    MessageBus bus;
    std::atomic<int> received{0};
    bus.subscribe<Detection>("det", [&](const Detection&) { received.fetch_add(1); });
    constexpr int kThreads = 4, kEach = 10000;
    std::vector<std::thread> threads;
    for (int t = 0; t < kThreads; ++t) {
        threads.emplace_back([&] {
            for (int i = 0; i < kEach; ++i) bus.publish("det", Detection{});
        });
    }
    for (auto& th : threads) th.join();
    EXPECT_EQ(received.load(), kThreads * kEach);
    EXPECT_EQ(bus.published_count(), static_cast<uint64_t>(kThreads * kEach));
}
