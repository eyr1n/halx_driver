#pragma once

#include <cstdint>

#include <halx/core.hpp>

namespace halx::drivers {

class IncrementalEncoder {
public:
  IncrementalEncoder(TIM_HandleTypeDef *htim) : htim_{htim} {
    HAL_TIM_Encoder_Start(htim_, TIM_CHANNEL_ALL);
  }

  ~IncrementalEncoder() { HAL_TIM_Encoder_Stop(htim_, TIM_CHANNEL_ALL); }

  void update() {
    int16_t delta = __HAL_TIM_GET_COUNTER(htim_);
    __HAL_TIM_SET_COUNTER(htim_, 0);
    position_ += delta;
  }

  int64_t get_position() { return position_; }

private:
  TIM_HandleTypeDef *htim_;
  int64_t position_ = 0;
};

} // namespace halx::drivers
