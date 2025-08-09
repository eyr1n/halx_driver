#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <optional>
#include <utility>

#include <halx/peripheral/uart.hpp>

namespace halx::driver {

enum class UnitreeGoMode : uint8_t {
  LOCK,
  FOC,
  CALIBRATION,
};

enum class UnitreeGoError : uint8_t {
  NORMAL,
  OVERHEATING,
  OVERCURRENT,
  OVERVOLTAGE,
  ENCODER_FAULT,
};

struct UnitreeGoCommand {
  uint8_t id;
  UnitreeGoMode mode;
  float torque;
  float radps;
  float rad;
  float kp;
  float kd;
};

struct UnitreeGoFeedback {
  uint8_t id;
  UnitreeGoMode mode;
  float torque;
  float radps;
  float rad;
  int8_t temp;
  UnitreeGoError error;
  int16_t force;
};

class UnitreeGo {
public:
  UnitreeGo(peripheral::UartBase &uart) : uart_{uart} {}

  std::optional<UnitreeGoFeedback> transmit(const UnitreeGoCommand &command) {
    auto tx_buf = to_uart_buf(command);
    std::array<uint8_t, 16> rx_buf;
    uart_.flush();
    if (!uart_.transmit(tx_buf.data(), tx_buf.size(), 10)) {
      return std::nullopt;
    }
    if (!uart_.receive(rx_buf.data(), rx_buf.size(), 10)) {
      return std::nullopt;
    }
    return to_unitree_go_feedback(rx_buf);
  }

private:
  peripheral::UartBase &uart_;

  static inline std::array<uint8_t, 17>
  to_uart_buf(const UnitreeGoCommand &command) {
    int16_t tau_set = std::clamp(command.torque, -127.99f, 127.99f) * 256.0f;
    int16_t omega_set = std::clamp(command.radps, -804.0f, 804.0f) /
                        (2.0f * std::numbers::pi) * 256.0f;
    int32_t theta_set = command.rad / (2.0f * std::numbers::pi) * 32768.0f;
    int16_t k_pos = std::clamp(command.kp, 0.0f, 25.599f) * 1280.0f;
    int16_t k_spd = std::clamp(command.kd, 0.0f, 25.599f) * 1280.0f;

    std::array<uint8_t, 17> buf;
    buf[0] = 0xFE;
    buf[1] = 0xEE;
    buf[2] = std::to_underlying(command.mode) << 4 | command.id;
    buf[3] = tau_set & 0xFF;
    buf[4] = (tau_set >> 8) & 0xFF;
    buf[5] = omega_set & 0xFF;
    buf[6] = (omega_set >> 8) & 0xFF;
    buf[7] = theta_set & 0xFF;
    buf[8] = (theta_set >> 8) & 0xFF;
    buf[9] = (theta_set >> 16) & 0xFF;
    buf[10] = (theta_set >> 24) & 0xFF;
    buf[11] = k_pos & 0xFF;
    buf[12] = (k_pos >> 8) & 0xFF;
    buf[13] = k_spd & 0xFF;
    buf[14] = (k_spd >> 8) & 0xFF;
    uint16_t crc = crc16_ccitt(buf.data(), 15);
    buf[15] = crc & 0xFF;
    buf[16] = (crc >> 8) & 0xFF;
    return buf;
  }

  static inline std::optional<UnitreeGoFeedback>
  to_unitree_go_feedback(const std::array<uint8_t, 16> &buf) {
    if (buf[0] != 0xFD || buf[1] != 0xEE) {
      return std::nullopt;
    }
    uint16_t crc = (buf[15] << 8) | buf[14];
    if (crc != crc16_ccitt(buf.data(), 14)) {
      return std::nullopt;
    }

    UnitreeGoFeedback feedback;
    feedback.id = buf[2] & 0x0F;
    feedback.mode = static_cast<UnitreeGoMode>((buf[2] >> 4) & 0x07);
    feedback.torque = static_cast<int16_t>((buf[4] << 8) | buf[3]) / 256.0f;
    feedback.radps = static_cast<int16_t>((buf[6] << 8) | buf[5]) / 256.0f *
                     2.0f * std::numbers::pi;
    feedback.rad = static_cast<int32_t>((buf[10] << 24) | (buf[9] << 16) |
                                        (buf[8] << 8) | buf[7]) /
                   32768.0f * 2.0f * std::numbers::pi;
    feedback.temp = buf[11];
    feedback.error = static_cast<UnitreeGoError>(buf[12] & 0x07);
    feedback.force = (buf[13] & 0x7F) | ((buf[12] >> 3) & 0x1F);
    return feedback;
  }

  static inline uint16_t crc16_ccitt(const uint8_t *data, size_t size) {
    uint16_t crc = 0;
    for (size_t i = 0; i < size; ++i) {
      crc ^= data[i];
      for (uint8_t j = 0; j < 8; ++j) {
        crc = crc & 1 ? (crc >> 1) ^ 0x8408 : crc >> 1;
      }
    }
    return crc;
  }
};

} // namespace halx::driver