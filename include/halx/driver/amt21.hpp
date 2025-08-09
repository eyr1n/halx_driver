#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <utility>

#include <halx/peripheral/uart.hpp>

namespace halx::drivers {

enum class Amt21Resolution : uint8_t {
  BIT_12 = 12,
  BIT_14 = 14,
};

class Amt21Manager {
public:
  Amt21Manager(peripheral::UartBase &uart) : uart_{uart} {}

  std::optional<std::array<uint8_t, 2>> send_command(uint8_t address,
                                                     uint8_t command) {
    uint8_t tx_buf = address | command;
    std::array<uint8_t, 2> rx_buf;
    uart_.flush();
    if (!uart_.transmit(&tx_buf, 1, 5)) {
      return std::nullopt;
    }
    if (!uart_.receive(rx_buf.data(), rx_buf.size(), 5)) {
      return std::nullopt;
    }
    if (!test_checksum(rx_buf[0], rx_buf[1])) {
      return std::nullopt;
    }
    return rx_buf;
  }

  bool send_extended_command(uint8_t address, uint8_t command) {
    std::array<uint8_t, 2> buf{static_cast<uint8_t>(address | 0x02), command};
    uart_.flush();
    if (!uart_.transmit(buf.data(), buf.size(), 5)) {
      return false;
    }
    return true;
  }

private:
  peripheral::UartBase &uart_;

  static inline bool test_checksum(uint8_t l, uint8_t h) {
    bool k1 = !(bit(h, 5) ^ bit(h, 3) ^ bit(h, 1) ^ bit(l, 7) ^ bit(l, 5) ^
                bit(l, 3) ^ bit(l, 1));
    bool k0 = !(bit(h, 4) ^ bit(h, 2) ^ bit(h, 0) ^ bit(l, 6) ^ bit(l, 4) ^
                bit(l, 2) ^ bit(l, 0));
    return (k1 == bit(h, 7)) && (k0 == bit(h, 6));
  }

  static inline bool bit(uint8_t x, uint8_t i) { return ((x >> i) & 1) == 1; }
};

class Amt21 {
public:
  Amt21(Amt21Manager &manager, Amt21Resolution resolution, uint8_t address)
      : manager_{manager}, resolution_{resolution}, address_{address} {}

  std::optional<uint16_t> read_position() {
    auto res = manager_.send_command(address_, 0x00);
    if (!res) {
      return std::nullopt;
    }
    return ((*res)[1] << 8 | (*res)[0]) &
           ((1 << std::to_underlying(resolution_)) - 1);
  }

  std::optional<int16_t> read_turns() {
    auto res = manager_.send_command(address_, 0x01);
    if (!res) {
      return std::nullopt;
    }
    int16_t turns = ((*res)[1] << 8 | (*res)[0]) & 0x3FFF;
    if (turns & 0x2000) {
      turns |= 0xC000;
    }
    return turns;
  }

  bool set_zero_point() {
    return manager_.send_extended_command(address_, 0x5E);
  }

  bool reset() { return manager_.send_extended_command(address_, 0x75); }

private:
  Amt21Manager &manager_;
  Amt21Resolution resolution_;
  uint8_t address_;
};

} // namespace halx::drivers