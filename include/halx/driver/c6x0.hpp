#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>

#include <halx/core.hpp>
#include <halx/peripheral/can.hpp>

namespace halx::driver {

enum class C6x0Type {
  C610,
  C620,
};

enum class C6x0Id {
  ID_1,
  ID_2,
  ID_3,
  ID_4,
  ID_5,
  ID_6,
  ID_7,
  ID_8,
};

class C6x0Manager {
public:
  C6x0Manager(peripheral::CanBase &can) : can_{can} {
    auto filter_index = can_.attach_rx_queue({0x200, 0x7F0, false}, rx_queue_);
    if (!filter_index) {
      std::terminate();
    }
    filter_index_ = *filter_index;
  }

  ~C6x0Manager() { can_.detach_rx_filter(filter_index_); }

  void update() {
    while (auto msg = rx_queue_.pop()) {
      if (0x201 <= msg->id && msg->id < 0x201 + 8) {
        Params &motor = params_[msg->id - 0x201];
        int16_t position =
            static_cast<int16_t>(msg->data[0] << 8 | msg->data[1]);
        if (motor.prev_position_) {
          int16_t delta = position - *motor.prev_position_;
          if (delta > 4096) {
            delta -= 8192;
          } else if (delta < -4096) {
            delta += 8192;
          }
          motor.position_ += delta;
        } else {
          motor.position_ = position;
        }
        motor.prev_position_ = position;
        motor.rpm_ = static_cast<int16_t>(msg->data[2] << 8 | msg->data[3]);
        motor.current_raw_ =
            static_cast<int16_t>(msg->data[4] << 8 | msg->data[5]);
      }
    }
  }

  bool transmit() {
    peripheral::CanMessage msg{};
    msg.id = 0x200;
    msg.ide = false;
    msg.dlc = 8;
    for (size_t i = 0; i < 4; ++i) {
      msg.data[i * 2] = params_[i].current_ref_raw_ >> 8;
      msg.data[i * 2 + 1] = params_[i].current_ref_raw_;
    }
    if (!can_.transmit(msg, 5)) {
      return false;
    }
    msg.data.fill(0);
    msg.id = 0x1FF;
    for (size_t i = 0; i < 4; ++i) {
      msg.data[i * 2] = params_[i + 4].current_ref_raw_ >> 8;
      msg.data[i * 2 + 1] = params_[i + 4].current_ref_raw_;
    }
    return can_.transmit(msg, 5);
  }

  int64_t get_position(C6x0Id id) {
    return params_[std::to_underlying(id)].position_;
  }

  int16_t get_rpm(C6x0Id id) { return params_[std::to_underlying(id)].rpm_; }

  int16_t get_current_raw(C6x0Id id) {
    return params_[std::to_underlying(id)].current_raw_;
  }

  void set_current_ref_raw(C6x0Id id, int16_t current) {
    params_[std::to_underlying(id)].current_ref_raw_ = current;
  }

private:
  struct Params {
    int64_t position_ = 0;
    std::optional<int16_t> prev_position_;
    int16_t rpm_ = 0;
    int16_t current_raw_ = 0;
    int16_t current_ref_raw_ = 0;
  };

  peripheral::CanBase &can_;
  core::RingBuffer<peripheral::CanMessage> rx_queue_{64};
  size_t filter_index_;
  std::array<Params, 8> params_{};
};

class C6x0 {
public:
  C6x0(C6x0Manager &manager, C6x0Type type, C6x0Id id)
      : manager_{manager}, type_{type}, id_{id} {}

  ~C6x0() { set_current_ref(0); }

  int64_t get_position() { return manager_.get_position(id_); }

  int16_t get_rpm() { return manager_.get_rpm(id_); }

  float get_current() {
    switch (type_) {
    case C6x0Type::C610:
      return manager_.get_current_raw(id_);
    case C6x0Type::C620:
      return manager_.get_current_raw(id_) / 16384.0f * 20000.0f;
    }
  }

  void set_current_ref(float current) {
    switch (type_) {
    case C6x0Type::C610:
      manager_.set_current_ref_raw(id_,
                                   std::clamp(current, -10000.0f, 10000.0f));
      break;
    case C6x0Type::C620:
      manager_.set_current_ref_raw(
          id_, std::clamp(current, -20000.0f, 20000.0f) / 20000.0f * 16384.0f);
      break;
    }
  }

private:
  C6x0Manager &manager_;
  C6x0Type type_;
  C6x0Id id_;
};

} // namespace halx::driver