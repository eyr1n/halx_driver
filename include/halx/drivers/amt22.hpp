#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <utility>

#include <stm32cubemx_helper/context.hpp>
#include <stm32cubemx_helper/device.hpp>

#include <halx/core.hpp>
#include <halx/rtos.hpp>

namespace halx::drivers {

enum class Amt22Resolution : uint8_t {
  BIT_12 = 12,
  BIT_14 = 14,
};

template <SPI_HandleTypeDef *Handle> class Amt22 {
public:
  Amt22(GPIO_TypeDef *cs_port, uint16_t cs_pin, Amt22Resolution resolution)
      : cs_port_{cs_port}, cs_pin_{cs_pin}, resolution_{resolution} {
    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);
    stm32cubemx_helper::set_context<Handle, Amt22>(this);
    HAL_SPI_RegisterCallback(
        Handle, HAL_SPI_TX_RX_COMPLETE_CB_ID, [](SPI_HandleTypeDef *hspi) {
          auto amt22 = stm32cubemx_helper::get_context<Handle, Amt22>();
          amt22->tx_rx_sem_.release();
        });
  }

  ~Amt22() {
    HAL_SPI_Abort_IT(Handle);
    HAL_SPI_UnRegisterCallback(Handle, HAL_SPI_TX_RX_COMPLETE_CB_ID);
    stm32cubemx_helper::set_context<Handle, Amt22>(nullptr);
    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);
  }

  std::optional<uint16_t> read_position() {
    std::array<uint8_t, 2> command{0x00, 0x00};
    auto res = send_command(command);
    if (!res) {
      return std::nullopt;
    }
    return ((*res)[0] << 8 | (*res)[1]) &
           ((1 << std::to_underlying(resolution_)) - 1);
  }

  std::optional<int16_t> read_turns() {
    std::array<uint8_t, 4> command{0x00, 0xA0, 0x00, 0x00};
    auto res = send_command(command);
    if (!res) {
      return std::nullopt;
    }
    int16_t turns = ((*res)[2] << 8 | (*res)[3]) & 0x3FFF;
    if (turns & 0x2000) {
      turns |= 0xC000;
    }
    return turns;
  }

  bool set_zero_point() {
    std::array<uint8_t, 2> command{0x00, 0x70};
    return send_command(command).has_value();
  }

  bool reset() {
    std::array<uint8_t, 2> command{0x00, 0x60};
    return send_command(command).has_value();
  }

private:
  GPIO_TypeDef *cs_port_;
  uint16_t cs_pin_;
  rtos::Semaphore tx_rx_sem_{1, 1};
  Amt22Resolution resolution_;

  template <size_t N>
  std::optional<std::array<uint8_t, N>>
  send_command(const std::array<uint8_t, N> &command) {
    std::array<uint8_t, N> buf;
    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);
    for (size_t i = 0; i < N; ++i) {
      tx_rx_sem_.acquire(0);
      if (HAL_SPI_TransmitReceive_IT(Handle, &command[i], &buf[i],
                                     sizeof(uint8_t))) {
        HAL_SPI_Abort_IT(Handle);
        return std::nullopt;
      }
      if (!tx_rx_sem_.acquire(1)) {
        HAL_SPI_Abort_IT(Handle);
        return std::nullopt;
      }
    }
    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);
    for (size_t i = 0; i < N; i += 2) {
      if (!test_checksum(buf[i], buf[i + 1])) {
        return std::nullopt;
      }
    }
    return buf;
  }

  static inline bool test_checksum(uint8_t l, uint8_t h) {
    bool k1 = !(bit(h, 5) ^ bit(h, 3) ^ bit(h, 1) ^ bit(l, 7) ^ bit(l, 5) ^
                bit(l, 3) ^ bit(l, 1));
    bool k0 = !(bit(h, 4) ^ bit(h, 2) ^ bit(h, 0) ^ bit(l, 6) ^ bit(l, 4) ^
                bit(l, 2) ^ bit(l, 0));
    return (k1 == bit(h, 7)) && (k0 == bit(h, 6));
  }

  static inline bool bit(uint8_t x, uint8_t i) { return ((x >> i) & 1) == 1; }
};

} // namespace halx::drivers