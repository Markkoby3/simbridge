// A typed, topic based publish/subscribe bus modeled on DDS concepts.
//
//   * Topics are named and bound to exactly one message type. Publishing or
//     subscribing with a different type throws TopicTypeError, the same
//     guarantee a DDS participant gives when type names disagree.
//   * A topic's QoS can keep the last N samples. Late subscribers receive that
//     history on subscribe, like DDS TRANSIENT_LOCAL durability with KEEP_LAST(N).
//   * Publishing is thread safe. Callbacks run on the publishing thread and are
//     invoked outside the bus lock, so a callback may publish or subscribe.
//
// Lifetime note: declare every topic from the host process before loading
// plugins, so no topic object is created by code that lives in a plugin library.
#pragma once

#include <atomic>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <typeindex>
#include <typeinfo>
#include <unordered_map>
#include <utility>
#include <vector>

namespace simbridge {

class TopicTypeError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

struct TopicQos {
    // 0 = volatile (no history). N = keep and replay the last N samples.
    std::size_t history_depth = 0;
};

using SubscriptionId = uint64_t;

class MessageBus {
public:
    MessageBus() = default;
    MessageBus(const MessageBus&) = delete;
    MessageBus& operator=(const MessageBus&) = delete;

    // Create a topic (or update its QoS) bound to message type T.
    template <typename T>
    void declare(const std::string& topic, TopicQos qos = {}) {
        std::lock_guard<std::mutex> lock(mu_);
        Topic<T>& t = topic_locked<T>(topic);
        t.qos = qos;
        while (t.history.size() > qos.history_depth) t.history.pop_front();
    }

    template <typename T>
    SubscriptionId subscribe(const std::string& topic, std::function<void(const T&)> callback) {
        auto cb = std::make_shared<std::function<void(const T&)>>(std::move(callback));
        const SubscriptionId id = next_id_.fetch_add(1);
        std::deque<T> replay;
        {
            std::lock_guard<std::mutex> lock(mu_);
            Topic<T>& t = topic_locked<T>(topic);
            t.subs.emplace_back(id, cb);
            replay = t.history;
        }
        for (const T& sample : replay) (*cb)(sample);
        return id;
    }

    template <typename T>
    void publish(const std::string& topic, const T& msg) {
        std::vector<std::shared_ptr<std::function<void(const T&)>>> targets;
        {
            std::lock_guard<std::mutex> lock(mu_);
            Topic<T>& t = topic_locked<T>(topic);
            if (t.qos.history_depth > 0) {
                t.history.push_back(msg);
                if (t.history.size() > t.qos.history_depth) t.history.pop_front();
            }
            targets.reserve(t.subs.size());
            for (auto& s : t.subs) targets.push_back(s.second);
        }
        published_.fetch_add(1, std::memory_order_relaxed);
        for (auto& cb : targets) (*cb)(msg);
    }

    bool unsubscribe(SubscriptionId id) {
        std::lock_guard<std::mutex> lock(mu_);
        for (auto& entry : topics_) {
            if (entry.second->remove(id)) return true;
        }
        return false;
    }

    std::size_t subscriber_count(const std::string& topic) const {
        std::lock_guard<std::mutex> lock(mu_);
        auto it = topics_.find(topic);
        return it == topics_.end() ? 0 : it->second->count();
    }

    uint64_t published_count() const { return published_.load(std::memory_order_relaxed); }

private:
    struct TopicBase {
        explicit TopicBase(std::type_index t) : type(t) {}
        virtual ~TopicBase() = default;
        virtual bool remove(SubscriptionId id) = 0;
        virtual std::size_t count() const = 0;
        std::type_index type;
        TopicQos qos;
    };

    template <typename T>
    struct Topic final : TopicBase {
        Topic() : TopicBase(std::type_index(typeid(T))) {}
        bool remove(SubscriptionId id) override {
            for (auto it = subs.begin(); it != subs.end(); ++it) {
                if (it->first == id) {
                    subs.erase(it);
                    return true;
                }
            }
            return false;
        }
        std::size_t count() const override { return subs.size(); }
        std::vector<std::pair<SubscriptionId, std::shared_ptr<std::function<void(const T&)>>>> subs;
        std::deque<T> history;
    };

    // Caller must hold mu_.
    template <typename T>
    Topic<T>& topic_locked(const std::string& name) {
        auto it = topics_.find(name);
        if (it == topics_.end()) {
            auto created = std::make_unique<Topic<T>>();
            Topic<T>& ref = *created;
            topics_.emplace(name, std::move(created));
            return ref;
        }
        if (it->second->type != std::type_index(typeid(T))) {
            throw TopicTypeError("topic '" + name + "' is bound to a different message type");
        }
        return static_cast<Topic<T>&>(*it->second);
    }

    mutable std::mutex mu_;
    std::unordered_map<std::string, std::unique_ptr<TopicBase>> topics_;
    std::atomic<SubscriptionId> next_id_{1};
    std::atomic<uint64_t> published_{0};
};

}  // namespace simbridge
