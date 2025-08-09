#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <halx/core.hpp>

namespace halx::driver {

struct Rgb {
  float r;
  float g;
  float b;
};

struct Hsv {
  float h;
  float s;
  float v;
};

inline Rgb to_rgb(const Hsv &hsv) {
  float c = hsv.v * hsv.s;
  float x = c * (1.0f - std::fabs(std::fmod(hsv.h / 60.0f, 2.0f) - 1.0f));
  float m = hsv.v - c;

  if (hsv.h >= 0 && hsv.h < 60) {
    return {(c + m), (x + m), (0 + m)};
  } else if (hsv.h >= 60 && hsv.h < 120) {
    return {(x + m), (c + m), (0 + m)};
  } else if (hsv.h >= 120 && hsv.h < 180) {
    return {(0 + m), (c + m), (x + m)};
  } else if (hsv.h >= 180 && hsv.h < 240) {
    return {(0 + m), (x + m), (c + m)};
  } else if (hsv.h >= 240 && hsv.h < 300) {
    return {(x + m), (0 + m), (c + m)};
  } else {
    return {(c + m), (0 + m), (x + m)};
  }
}

// period: 1.25us
// freq: 800kHz
// counter period: 25-1
// dma circular mode
class Ws2812b {
public:
  Ws2812b(TIM_HandleTypeDef *htim, uint32_t channel, size_t size)
      : htim_{htim}, channel_{channel}, size_{size},
        buf_(RESET_CODE + size * 24, 0) {
    HAL_TIM_PWM_Start_DMA(htim_, channel_,
                          reinterpret_cast<uint32_t *>(buf_.data()),
                          buf_.size());
  }

  void set_rgb(size_t index, const Rgb &rgb) {
    uint8_t r = rgb.r * 255.0f;
    uint8_t g = rgb.g * 255.0f;
    uint8_t b = rgb.b * 255.0f;

    // G
    size_t offset = RESET_CODE + index * 24;
    for (size_t i = 0; i < 8; ++i) {
      if (g & (1 << (7 - i))) {
        buf_[offset + i] = ONE_CODE;
      } else {
        buf_[offset + i] = ZERO_CODE;
      }
    }

    // R
    for (size_t i = 0; i < 8; ++i) {
      if (r & (1 << (7 - i))) {
        buf_[offset + 8 + i] = ONE_CODE;
      } else {
        buf_[offset + 8 + i] = ZERO_CODE;
      }
    }

    // B
    for (size_t i = 0; i < 8; ++i) {
      if (b & (1 << (7 - i))) {
        buf_[offset + 16 + i] = ONE_CODE;
      } else {
        buf_[offset + 16 + i] = ZERO_CODE;
      }
    }
  }

  void set_rgb_all(const Rgb &rgb) {
    for (size_t i = 0; i < size_; ++i) {
      set_rgb(i, rgb);
    }
  }

  size_t size() { return size_; }

private:
  static constexpr uint8_t RESET_CODE = 50;
  static constexpr uint8_t ZERO_CODE = 8;
  static constexpr uint8_t ONE_CODE = 16;

  TIM_HandleTypeDef *htim_;
  uint32_t channel_;
  size_t size_;
  std::vector<uint32_t> buf_;
};

} // namespace halx::driver