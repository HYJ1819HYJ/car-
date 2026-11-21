#ifndef STM32_BRIDGE_JOYSTICK_GUARD_HPP
#define STM32_BRIDGE_JOYSTICK_GUARD_HPP

#include <algorithm>
#include <cerrno>
#include <linux/joystick.h>
#include <unistd.h>

namespace stm32_bridge {

// A nonblocking joystick produces EAGAIN while its current state is unchanged.
// EOF and read errors instead invalidate the stored deadman and axis states.
inline bool readJoystickSafely(int fd, double (&axes)[8], int (&buttons)[11]) {
  for (;;) {
    struct js_event event;
    const ssize_t n = ::read(fd, &event, sizeof(event));
    if (n == static_cast<ssize_t>(sizeof(event))) {
      const unsigned char type = event.type & ~JS_EVENT_INIT;
      if (type == JS_EVENT_BUTTON && event.number < 11) buttons[event.number] = event.value;
      if (type == JS_EVENT_AXIS && event.number < 8) axes[event.number] = static_cast<double>(event.value) / 32767.0;
      continue;
    }
    if (n < 0 && errno == EINTR) continue;
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return true;
    std::fill(axes, axes + 8, 0.0);
    std::fill(buttons, buttons + 11, 0);
    return false;
  }
}

}  // namespace stm32_bridge

#endif  // STM32_BRIDGE_JOYSTICK_GUARD_HPP
