#include "stm32_bridge/protocol.hpp"

namespace stm32_bridge {

uint8_t checksum8_sum(const uint8_t* data, size_t len) {
	uint8_t sum = 0;
	for (size_t i = 0; i < len; ++i) {
		sum += data[i];
	}
	return sum;
}

std::vector<uint8_t> packControl(const ControlCmd& cmd, const ControlProtocol& proto) {
	std::vector<uint8_t> frame(8, 0);
	frame[0] = proto.head1;
	frame[1] = proto.head2;
	frame[2] = static_cast<uint8_t>((cmd.v_mm_s >> 8) & 0xFF);
	frame[3] = static_cast<uint8_t>(cmd.v_mm_s & 0xFF);
	frame[4] = static_cast<uint8_t>((cmd.w_mrad_s >> 8) & 0xFF);
	frame[5] = static_cast<uint8_t>(cmd.w_mrad_s & 0xFF);
	frame[6] = checksum8_sum(frame.data() + 2, 4);
	frame[7] = proto.tail;
	return frame;
}

bool unpackFeedback(const std::vector<uint8_t>& frame, EncoderFeedback& fb, const FeedbackProtocol& proto) {
	if (frame.size() < 9) {
		fb.valid = false;
		return false;
	}
	if (!(frame[0] == proto.head1 && frame[1] == proto.head2)) {
		fb.valid = false;
		return false;
	}
	if (!(frame[7] == proto.tail1 && frame[8] == proto.tail2)) {
		fb.valid = false;
		return false;
	}

	const uint8_t crc = checksum8_sum(frame.data() + 2, 4);
	if (crc != frame[6]) {
		fb.valid = false;
		return false;
	}

	fb.left = static_cast<int16_t>((static_cast<uint16_t>(frame[2]) << 8) | frame[3]);
	fb.right = static_cast<int16_t>((static_cast<uint16_t>(frame[4]) << 8) | frame[5]);
	fb.valid = true;
	return true;
}

}  // namespace stm32_bridge
