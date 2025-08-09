#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <numbers>
#include <optional>
#include <utility>

#include <halx/core.hpp>
#include <halx/peripheral/can.hpp>

namespace halx::drivers {

enum class CyberGearRunMode : uint8_t {
  OPERATION_CONTROL = 0x00,
  POSITION = 0x01,
  SPEED = 0x02,
  CURRENT = 0x03,
};

enum class CyberGearParameter : uint16_t {
  RUN_MODE = 0x7005,
  IQ_REF = 0x7006,
  SPD_REF = 0x700A,
  LIMIT_TORQUE = 0x700B,
  CUR_KP = 0x7010,
  CUR_KI = 0x7011,
  CUR_FILT_GAIN = 0x7014,
  LOC_REF = 0x7016,
  LIMIT_SPD = 0x7017,
  LIMIT_CUR = 0x7018,
  MECH_POS = 0x7019,
  IQF = 0x701A,
  MECH_VEL = 0x701B,
  VBUS = 0x701C,
  ROTATION = 0x701D,
  LOC_KP = 0x701E,
  SPD_KP = 0x701F,
  SPD_KI = 0x7020,
};

struct CyberGearFeedback {
  uint8_t motor_can_id;
  uint8_t fault;
  uint8_t mode;
  float angle;
  float velocity;
  float torque;
  float temperature;
};

class CyberGear {
public:
  CyberGear(peripheral::CanBase &can, uint8_t motor_can_id, uint8_t host_can_id)
      : can_{can}, motor_can_id_{motor_can_id}, master_can_id_{host_can_id} {
    auto filter_index = can_.attach_rx_queue(
        {static_cast<uint32_t>(motor_can_id_ << 8), 0x800FF00, true},
        rx_queue_);
    if (!filter_index) {
      std::terminate();
    }
    filter_index_ = *filter_index;
  }

  ~CyberGear() { can_.detach_rx_filter(filter_index_); }

  std::optional<CyberGearFeedback> set_operation_control(float torque,
                                                         float angle,
                                                         float velocity,
                                                         float kp, float kd) {
    uint16_t torque_int =
        std::clamp((torque + 12.0f) / 24.0f * 65535.0f, 0.0f, 65535.0f);
    uint16_t angle_int =
        std::clamp((angle + 4.0f * static_cast<float>(std::numbers::pi)) /
                       (8.0f * static_cast<float>(std::numbers::pi)) * 65535.0f,
                   0.0f, 65535.0f);
    uint16_t velocity_int =
        std::clamp((velocity + 30.0f) / 60.0f * 65535.0f, 0.0f, 65535.0f);
    uint16_t kp_int = std::clamp(kp / 500.0f * 65535.0f, 0.0f, 65535.0f);
    uint16_t kd_int = std::clamp(kd / 5.0f * 65535.0f, 0.0f, 65535.0f);

    CyberGearMessage msg;
    msg.type = CommunicationType::TYPE_1;
    msg.target_address = motor_can_id_;
    msg.data1[0] = (angle_int >> 8) & 0xFF;
    msg.data1[1] = angle_int & 0xFF;
    msg.data1[2] = (velocity_int >> 8) & 0xFF;
    msg.data1[3] = velocity_int & 0xFF;
    msg.data1[4] = (kp_int >> 8) & 0xFF;
    msg.data1[5] = kp_int & 0xFF;
    msg.data1[6] = (kd_int >> 8) & 0xFF;
    msg.data1[7] = kd_int & 0xFF;
    msg.data2 = torque_int;
    auto res = send_message(msg);
    if (!res) {
      return std::nullopt;
    }
    return to_cyber_gear_motor_feedback(*res);
  }

  std::optional<CyberGearFeedback> enable() {
    CyberGearMessage msg{
        CommunicationType::TYPE_3, motor_can_id_, {}, master_can_id_};
    auto res = send_message(msg);
    if (!res) {
      return std::nullopt;
    }
    return to_cyber_gear_motor_feedback(*res);
  }

  std::optional<CyberGearFeedback> stop(bool clear_fault) {
    CyberGearMessage msg{CommunicationType::TYPE_4,
                         motor_can_id_,
                         {static_cast<uint8_t>(clear_fault)},
                         master_can_id_};
    auto res = send_message(msg);
    if (!res) {
      return std::nullopt;
    }
    return to_cyber_gear_motor_feedback(*res);
  }

  std::optional<CyberGearFeedback> set_position_to_mechanical_zero() {
    CyberGearMessage msg{
        CommunicationType::TYPE_6, motor_can_id_, {1}, master_can_id_};
    auto res = send_message(msg);
    if (!res) {
      return std::nullopt;
    }
    return to_cyber_gear_motor_feedback(*res);
  }

  template <class T>
  std::optional<T> read_parameter(CyberGearParameter parameter) {
    uint16_t index = std::to_underlying(parameter);
    CyberGearMessage msg{
        CommunicationType::TYPE_17,
        motor_can_id_,
        {static_cast<uint8_t>(index), static_cast<uint8_t>(index >> 8)},
        master_can_id_};
    auto res = send_message(msg);
    if (!res) {
      return std::nullopt;
    }
    if (res->type != CommunicationType::TYPE_17) {
      return std::nullopt;
    }
    T data;
    std::memcpy(&data, &res->data1[4], sizeof(T));
    return data;
  }

  template <class T>
  std::optional<CyberGearFeedback> write_parameter(CyberGearParameter parameter,
                                                   T data) {
    uint16_t index = std::to_underlying(parameter);
    CyberGearMessage msg{
        CommunicationType::TYPE_18,
        motor_can_id_,
        {static_cast<uint8_t>(index), static_cast<uint8_t>(index >> 8)},
        master_can_id_};
    std::memcpy(&msg.data1[4], &data, sizeof(T));
    auto res = send_message(msg);
    if (!res) {
      return std::nullopt;
    }
    return to_cyber_gear_motor_feedback(*res);
  }

private:
  enum class CommunicationType : uint8_t {
    TYPE_0 = 0,
    TYPE_1 = 1,
    TYPE_2 = 2,
    TYPE_3 = 3,
    TYPE_4 = 4,
    TYPE_6 = 6,
    TYPE_7 = 7,
    TYPE_17 = 17,
    TYPE_18 = 18,
    TYPE_21 = 21,
    TYPE_22 = 22,
  };

  struct CyberGearMessage {
    CommunicationType type;
    uint8_t target_address;
    std::array<uint8_t, 8> data1;
    uint16_t data2;
  };

  peripheral::CanBase &can_;
  size_t filter_index_;
  core::RingBuffer<peripheral::CanMessage> rx_queue_{8};
  uint8_t motor_can_id_;
  uint8_t master_can_id_;

  static inline peripheral::CanMessage
  to_can_message(const CyberGearMessage &msg) {
    return {static_cast<uint32_t>(std::to_underlying(msg.type) << 24 |
                                  msg.data2 << 8 | msg.target_address),
            true, 8, msg.data1};
  }

  static inline std::optional<CyberGearMessage>
  to_cyber_gear_message(const peripheral::CanMessage &msg) {
    CyberGearMessage cyber_gear_msg;
    uint8_t type = (msg.id >> 24) & 0x1F;
    cyber_gear_msg.target_address = msg.id & 0xFF;
    cyber_gear_msg.data1 = msg.data;
    cyber_gear_msg.data2 = (msg.id >> 8) & 0xFFFF;

    switch (type) {
    case 0:
      cyber_gear_msg.type = CommunicationType::TYPE_0;
      break;
    case 1:
      cyber_gear_msg.type = CommunicationType::TYPE_1;
      break;
    case 2:
      cyber_gear_msg.type = CommunicationType::TYPE_2;
      break;
    case 3:
      cyber_gear_msg.type = CommunicationType::TYPE_3;
      break;
    case 4:
      cyber_gear_msg.type = CommunicationType::TYPE_4;
      break;
    case 6:
      cyber_gear_msg.type = CommunicationType::TYPE_6;
      break;
    case 7:
      cyber_gear_msg.type = CommunicationType::TYPE_7;
      break;
    case 17:
      cyber_gear_msg.type = CommunicationType::TYPE_17;
      break;
    case 18:
      cyber_gear_msg.type = CommunicationType::TYPE_18;
      break;
    case 21:
      cyber_gear_msg.type = CommunicationType::TYPE_21;
      break;
    case 22:
      cyber_gear_msg.type = CommunicationType::TYPE_22;
      break;
    default:
      return std::nullopt;
    }
    return cyber_gear_msg;
  }

  static inline std::optional<CyberGearFeedback>
  to_cyber_gear_motor_feedback(const CyberGearMessage &msg) {
    if (msg.type != CommunicationType::TYPE_2) {
      return std::nullopt;
    }

    CyberGearFeedback feedback;
    feedback.motor_can_id = msg.data2 & 0xFF;
    feedback.fault = (msg.data2 >> 8) & 0x3F;
    feedback.mode = (msg.data2 >> 14) & 0x3;
    feedback.angle = ((msg.data1[0] << 8) | msg.data1[1]) / 65535.0f * 8.0f *
                         std::numbers::pi -
                     4.0f * std::numbers::pi;
    feedback.velocity =
        ((msg.data1[2] << 8) | msg.data1[3]) / 65535.0f * 60.0f - 30.0f;
    feedback.torque =
        ((msg.data1[4] << 8) | msg.data1[5]) / 65535.0f * 24.0f - 12.0f;
    feedback.temperature = ((msg.data1[6] << 8) | msg.data1[7]) / 10.0f;
    return feedback;
  }

  std::optional<CyberGearMessage> send_message(const CyberGearMessage &msg) {
    rx_queue_.clear();
    if (!can_.transmit(to_can_message(msg), 5)) {
      return std::nullopt;
    }
    auto can_msg = rx_queue_.pop(5);
    if (!can_msg) {
      return std::nullopt;
    }
    return to_cyber_gear_message(*can_msg);
  }
};

} // namespace halx::drivers
