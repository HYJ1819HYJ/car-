
import time


def control_frame(v_mm_s: int, w_mrad_s: int) -> bytes:
    """生成 STM32 接收的 AA 55 V W SUM CC 控制帧。"""
    payload = v_mm_s.to_bytes(2, "big", signed=True) + w_mrad_s.to_bytes(
        2, "big", signed=True
    )
    return bytes([0xAA, 0x55]) + payload + bytes([sum(payload) & 0xFF, 0xCC])


def main() -> None:
    try:
        import serial  # type: ignore[import-not-found]
    except ImportError as exc:
        raise SystemExit("缺少 pyserial，请安装 python3-serial 后再运行") from exc

    with serial.Serial("/dev/ttyUSB0", 115200, timeout=1) as ser:
        print("串口已打开")
        try:
            frame = control_frame(150, 0)
            print("发送:", frame.hex())
            ser.write(frame)
            ser.flush()
            time.sleep(0.5)

            ret = ser.read(20)
            print("收到:", ret.hex() if ret else "无返回")
        finally:
            # STM32 会持续向电机重发最后速度，测试结束前必须明确发送零速。
            stop = control_frame(0, 0)
            ser.write(stop)
            ser.flush()
            time.sleep(0.05)
            print("已发送零速停止帧:", stop.hex())


if __name__ == "__main__":
    main()
