# K210 MAIX Bit — 色块追踪脚本 (MaixPy)
#
# 功能: 检测指定颜色色块 → UART 发送坐标给 ESP32-S3
# 协议: "T,cx,cy,w,h\n" (有目标) / "N\n" (无目标)
#
# 使用方法:
#   1. 用 MaixPy IDE 连接 MAIX Bit
#   2. 用 Threshold Editor 校准目标颜色的 LAB 阈值
#   3. 修改下方 TARGET_THRESHOLD
#   4. 运行此脚本

import sensor
import image
import lcd
import time
from machine import UART

# ── 配置 ──
BAUD_RATE = 115200
UART_NUM = UART.UART1   # MAIX Bit UART1: TX=IO6, RX=IO7
MIN_PIXELS = 200         # 最小色块像素数 (过滤噪声)

# LAB 颜色阈值 (l_lo, l_hi, a_lo, a_hi, b_lo, b_hi)
# 用 MaixPy IDE → Tools → Machine Vision → Threshold Editor 校准
# 以下为红色示例值，需根据实际目标调整
TARGET_THRESHOLD = (30, 80, 15, 127, 15, 127)

# ── 初始化 ──
lcd.init()
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)  # 320x240
sensor.set_vflip(0)
sensor.set_hmirror(0)
sensor.run(1)
sensor.skip_frames(time=2000)  # 等待 AWB/AEC 稳定

uart = UART(UART_NUM, baudrate=BAUD_RATE, bits=8, parity=None, stop=1,
            tx=6, rx=7)

clock = time.clock()

print("Water Turret K210 Tracker Ready")
print("Threshold:", TARGET_THRESHOLD)
print("UART:", BAUD_RATE, "baud")

# ── 主循环 ──
while True:
    clock.tick()
    img = sensor.snapshot()

    blobs = img.find_blobs(
        [TARGET_THRESHOLD],
        pixels_threshold=MIN_PIXELS,
        area_threshold=MIN_PIXELS,
        merge=True,
        margin=10
    )

    if blobs:
        # 取最大色块
        b = max(blobs, key=lambda x: x.pixels())
        cx = b.cx()
        cy = b.cy()
        w = b.w()
        h = b.h()

        # 绘制追踪框
        img.draw_rectangle(b.rect(), color=(0, 255, 0))
        img.draw_cross(cx, cy, color=(0, 255, 0))
        img.draw_string(cx + 5, cy - 10, "(%d,%d)" % (cx, cy),
                        color=(0, 255, 0), scale=1)

        # 发送坐标
        uart.write("T,%d,%d,%d,%d\n" % (cx, cy, w, h))
    else:
        uart.write("N\n")

    # 显示 FPS
    fps = clock.fps()
    img.draw_string(2, 2, "%.1f fps" % fps, color=(255, 255, 0), scale=2)
    lcd.display(img)
