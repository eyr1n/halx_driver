#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <utility>

#include <halx/peripheral/uart.hpp>

namespace halx::driver {

class ScsManager {
public:
  ScsManager(peripheral::UartBase &uart) : uart_{uart} {}

  std::optional<uint8_t> ping(uint8_t id) {
    if (!send_message(id, Instruction::PING, 0, nullptr, 0)) {
      return std::nullopt;
    }
    return check_error(id);
  }

  [[nodiscard]] std::optional<uint8_t> read_data(uint8_t id, uint8_t address,
                                                 uint8_t *data, uint8_t size) {
    if (!send_message(id, Instruction::READ_DATA, address, &size, 1)) {
      return std::nullopt;
    }
    std::array<uint8_t, 6> buf;
    if (!uart_.receive(buf.data(), buf.size() - 1, 5)) {
      return std::nullopt;
    }
    if (buf[0] != 0xFF || buf[1] != 0xFF) {
      return std::nullopt;
    }
    if (buf[2] != id) {
      return std::nullopt;
    }
    if (buf[3] != size + 2) {
      return std::nullopt;
    }
    if (!uart_.receive(data, size, 5)) {
      return std::nullopt;
    }
    if (!uart_.receive(&buf[5], 1, 5)) {
      return std::nullopt;
    }
    uint8_t checksum = sum(&buf[2], buf.size() - 3) + sum(data, size);
    checksum = ~checksum;
    if (buf[5] != checksum) {
      return std::nullopt;
    }
    return buf[4];
  }

  std::optional<uint8_t> write_data(uint8_t id, uint8_t address,
                                    const uint8_t *data, uint8_t size) {
    if (!send_message(id, Instruction::WRITE_DATA, address, data, size)) {
      return std::nullopt;
    }
    return check_error(id);
  }

  std::optional<uint8_t> reg_write(uint8_t id, uint8_t address,
                                   const uint8_t *data, uint8_t size) {
    if (!send_message(id, Instruction::REG_WRITE, address, data, size)) {
      return std::nullopt;
    }
    return check_error(id);
  }

  std::optional<uint8_t> action(uint8_t id) {
    if (!send_message(id, Instruction::ACTION, 0, nullptr, 0)) {
      return std::nullopt;
    }
    return check_error(id);
  }

  std::optional<uint8_t> recovery(uint8_t id) {
    if (!send_message(id, Instruction::RECOVERY, 0, nullptr, 0)) {
      return std::nullopt;
    }
    return check_error(id);
  }

  std::optional<uint8_t> reset(uint8_t id) {
    if (!send_message(id, Instruction::RESET, 0, nullptr, 0)) {
      return std::nullopt;
    }
    return check_error(id);
  }

private:
  enum class Instruction : uint8_t {
    PING = 0x01,
    READ_DATA = 0x02,
    WRITE_DATA = 0x03,
    REG_WRITE = 0x04,
    ACTION = 0x05,
    SYNC_READ = 0x82,
    SYNC_WRITE = 0x83,
    RECOVERY = 0x06,
    RESET = 0x0A,
  };

  peripheral::UartBase &uart_;

  bool send_message(uint8_t id, Instruction instruction, uint8_t address,
                    const uint8_t *data, uint8_t size) {
    std::array<uint8_t, 5> buf{0xFF, 0xFF, id, static_cast<uint8_t>(size + 3),
                               std::to_underlying(instruction)};
    uart_.flush();
    if (!uart_.transmit(buf.data(), buf.size(), 5)) {
      return false;
    }
    if (!uart_.transmit(&address, 1, 5)) {
      return false;
    }
    uint8_t checksum = sum(&buf[2], buf.size() - 2) + address;
    if (data) {
      if (!uart_.transmit(data, size, 5)) {
        return false;
      }
      for (uint8_t i = 0; i < size; i++) {
        checksum += data[i];
      }
    }
    checksum = ~checksum;
    return uart_.transmit(&checksum, 1, 5);
  }

  std::optional<uint8_t> check_error(uint8_t id) {
    if (id == 0xFE) {
      return 0;
    }
    std::array<uint8_t, 6> buf;
    if (!uart_.receive(buf.data(), buf.size(), 5)) {
      return std::nullopt;
    }
    if (buf[0] != 0xFF || buf[1] != 0xFF) {
      return std::nullopt;
    }
    if (buf[2] != id) {
      return std::nullopt;
    }
    if (buf[3] != 2) {
      return std::nullopt;
    }
    uint8_t checksum = sum(&buf[2], buf.size() - 3);
    checksum = ~checksum;
    if (buf[5] != checksum) {
      return std::nullopt;
    }
    return buf[4];
  }

  static inline uint8_t sum(const uint8_t *data, uint8_t size) {
    uint8_t sum = 0;
    for (uint8_t i = 0; i < size; ++i) {
      sum += data[i];
    }
    return sum;
  }
};

} // namespace halx::driver
