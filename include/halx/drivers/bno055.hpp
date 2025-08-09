#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>

#include <Eigen/Geometry>

#include <halx/core.hpp>
#include <halx/peripheral/uart.hpp>

namespace halx::drivers {

enum class Bno055Register : uint8_t {
  // Page 0
  CHIP_ID = 0x00,
  ACC_ID = 0x01,
  MAG_ID = 0x02,
  GYR_ID = 0x03,
  SW_REV_ID_LSB = 0x04,
  SW_REV_ID_MSB = 0x05,
  BL_REV_ID = 0x06,
  PAGE_ID = 0x07,
  ACC_DATA_X_LSB = 0x08,
  ACC_DATA_X_MSB = 0x09,
  ACC_DATA_Y_LSB = 0x0A,
  ACC_DATA_Y_MSB = 0x0B,
  ACC_DATA_Z_LSB = 0x0C,
  ACC_DATA_Z_MSB = 0x0D,
  MAG_DATA_X_LSB = 0x0E,
  MAG_DATA_X_MSB = 0x0F,
  MAG_DATA_Y_LSB = 0x10,
  MAG_DATA_Y_MSB = 0x11,
  MAG_DATA_Z_LSB = 0x12,
  MAG_DATA_Z_MSB = 0x13,
  GYR_DATA_X_LSB = 0x14,
  GYR_DATA_X_MSB = 0x15,
  GYR_DATA_Y_LSB = 0x16,
  GYR_DATA_Y_MSB = 0x17,
  GYR_DATA_Z_LSB = 0x18,
  GYR_DATA_Z_MSB = 0x19,
  EUL_DATA_X_LSB = 0x1A,
  EUL_DATA_X_MSB = 0x1B,
  EUL_DATA_Y_LSB = 0x1C,
  EUL_DATA_Y_MSB = 0x1D,
  EUL_DATA_Z_LSB = 0x1E,
  EUL_DATA_Z_MSB = 0x1F,
  QUA_DATA_W_LSB = 0x20,
  QUA_DATA_W_MSB = 0x21,
  QUA_DATA_X_LSB = 0x22,
  QUA_DATA_X_MSB = 0x23,
  QUA_DATA_Y_LSB = 0x24,
  QUA_DATA_Y_MSB = 0x25,
  QUA_DATA_Z_LSB = 0x26,
  QUA_DATA_Z_MSB = 0x27,
  LIA_DATA_X_LSB = 0x28,
  LIA_DATA_X_MSB = 0x29,
  LIA_DATA_Y_LSB = 0x2A,
  LIA_DATA_Y_MSB = 0x2B,
  LIA_DATA_Z_LSB = 0x2C,
  LIA_DATA_Z_MSB = 0x2D,
  GRV_DATA_X_LSB = 0x2E,
  GRV_DATA_X_MSB = 0x2F,
  GRV_DATA_Y_LSB = 0x30,
  GRV_DATA_Y_MSB = 0x31,
  GRV_DATA_Z_LSB = 0x32,
  GRV_DATA_Z_MSB = 0x33,
  TEMP = 0x34,
  CALIB_STAT = 0x35,
  ST_RESULT = 0x36,
  INT_STA = 0x37,
  SYS_CLK_STATUS = 0x38,
  SYS_STATUS = 0x39,
  SYS_ERR = 0x3A,
  UNIT_SEL = 0x3B,
  OPR_MODE = 0x3D,
  PWR_MODE = 0x3E,
  SYS_TRIGGER = 0x3F,
  TEMP_SOURCE = 0x40,
  AXIS_MAP_CONFIG = 0x41,
  AXIS_MAP_SIGN = 0x42,
  ACC_OFFSET_X_LSB = 0x55,
  ACC_OFFSET_X_MSB = 0x56,
  ACC_OFFSET_Y_LSB = 0x57,
  ACC_OFFSET_Y_MSB = 0x58,
  ACC_OFFSET_Z_LSB = 0x59,
  ACC_OFFSET_Z_MSB = 0x5A,
  MAG_OFFSET_X_LSB = 0x5B,
  MAG_OFFSET_X_MSB = 0x5C,
  MAG_OFFSET_Y_LSB = 0x5D,
  MAG_OFFSET_Y_MSB = 0x5E,
  MAG_OFFSET_Z_LSB = 0x5F,
  MAG_OFFSET_Z_MSB = 0x60,
  GYR_OFFSET_X_LSB = 0x61,
  GYR_OFFSET_X_MSB = 0x62,
  GYR_OFFSET_Y_LSB = 0x63,
  GYR_OFFSET_Y_MSB = 0x64,
  GYR_OFFSET_Z_LSB = 0x65,
  GYR_OFFSET_Z_MSB = 0x66,
  ACC_RADIUS_LSB = 0x67,
  ACC_RADIUS_MSB = 0x68,
  MAG_RADIUS_LSB = 0x69,
  MAG_RADIUS_MSB = 0x6A,

  // Page 1
  ACC_CONFIG = 0x08,
  MAG_CONFIG = 0x09,
  GYR_CONFIG_0 = 0x0A,
  GYR_CONFIG_1 = 0x0B,
  ACC_SLEEP_CONFIG = 0x0C,
  GYR_SLEEP_CONFIG = 0x0D,
  INT_MSK = 0x0F,
  INT_EN = 0x10,
  ACC_AM_THRES = 0x11,
  ACC_INT_SETTINGS = 0x12,
  ACC_HG_DURATION = 0x13,
  ACC_HG_THRES = 0x14,
  ACC_NM_THRES = 0x15,
  ACC_NM_SET = 0x16,
  GYR_INT_SETTING = 0x17,
  GYR_HR_X_SET = 0x18,
  GYR_DUR_X = 0x19,
  GYR_HR_Y_SET = 0x1A,
  GYR_DUR_Y = 0x1B,
  GYR_HR_Z_SET = 0x1C,
  GYR_DUR_Z = 0x1D,
  GYR_AM_THRES = 0x1E,
  GYR_AM_SET = 0x1F
};

class Bno055 {
public:
  Bno055(peripheral::UartBase &uart) : uart_{uart} {}

  bool start(uint32_t timeout) {
    core::TimeoutHelper timeout_helper{timeout};
    while (!timeout_helper.is_timeout()) {
      std::array<uint8_t, 1> data{0x00};
      if (!write_register(Bno055Register::OPR_MODE, data)) {
        continue;
      }
      data[0] = 0x08;
      if (!write_register(Bno055Register::OPR_MODE, data)) {
        continue;
      }
      return true;
    }
    return false;
  }

  std::optional<Eigen::Quaternionf> get_quaternion() {
    auto res = read_register<8>(Bno055Register::QUA_DATA_W_LSB);
    if (!res) {
      return std::nullopt;
    }
    return Eigen::Quaternionf{
        static_cast<int16_t>(((*res)[1] << 8) | (*res)[0]) / 16384.0f,
        static_cast<int16_t>(((*res)[3] << 8) | (*res)[2]) / 16384.0f,
        static_cast<int16_t>(((*res)[5] << 8) | (*res)[4]) / 16384.0f,
        static_cast<int16_t>(((*res)[7] << 8) | (*res)[6]) / 16384.0f};
  }

  template <size_t N>
  bool write_register(Bno055Register address,
                      const std::array<uint8_t, N> &data) {
    std::array<uint8_t, 4> buf{0xAA, 0x00, std::to_underlying(address), N};
    uart_.flush();
    if (!uart_.transmit(buf.data(), buf.size(), 5)) {
      return false;
    }
    if (!uart_.transmit(data.data(), data.size(), 5)) {
      return false;
    }
    if (!uart_.receive(buf.data(), 2, 5)) {
      return false;
    }
    return buf[0] == 0xEE && buf[1] == 0x01;
  }

  template <size_t N>
  std::optional<std::array<uint8_t, N>> read_register(Bno055Register address) {
    std::array<uint8_t, 4> tx_buf{0xAA, 0x01, std::to_underlying(address), N};
    std::array<uint8_t, N> rx_buf;
    uart_.flush();
    if (!uart_.transmit(tx_buf.data(), tx_buf.size(), 5)) {
      return std::nullopt;
    }
    if (!uart_.receive(tx_buf.data(), 2, 5)) {
      return std::nullopt;
    }
    if (tx_buf[0] != 0xBB || tx_buf[1] != N) {
      return std::nullopt;
    }
    if (!uart_.receive(rx_buf.data(), rx_buf.size(), 5)) {
      return std::nullopt;
    }
    return rx_buf;
  }

private:
  peripheral::UartBase &uart_;
};

} // namespace halx::drivers
