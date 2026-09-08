#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>

namespace stm32_bridge {

struct ControlCmd {
  int16_t v_mm_s{0};     // mm/s
  int16_t w_mrad_s{0};   // 0.001 rad/s
};

struct ControlProtocol {
  uint8_t head1{0xAA};
  uint8_t head2{0x55};
  uint8_t tail{0xCC};
};

struct FeedbackProtocol {
  uint8_t head1{0xAA};
  uint8_t head2{0x55};
  uint8_t tail1{0x0D};
  uint8_t tail2{0x0A};
};

struct EncoderFeedback {
  int16_t left{0};
  int16_t right{0};
  bool valid{false};
};

uint8_t checksum8_sum(const uint8_t* data, size_t len);

// ROS->STM32 pack: AA 55 Vh Vl Wh Wl chk CC
std::vector<uint8_t> packControl(const ControlCmd& cmd, const ControlProtocol& proto);

// STM32->ROS unpack: AA 55 Lh Ll Rh Rl chk 0D 0A
bool unpackFeedback(const std::vector<uint8_t>& frame, EncoderFeedback& fb, const FeedbackProtocol& proto);

} // namespace
