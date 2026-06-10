/* water_turret — 自动追踪水枪炮台主程序
 *
 * 架构: K210 (MaixPy 色块追踪) → UART → ESP32-S3 (PID 舵机控制 + 继电器)
 *
 * UART 协议:
 *   K210 → ESP32: "T,cx,cy,w,h\n" (有目标) / "N\n" (无目标)
 *   帧坐标系: 320×240 (QVGA), 中心 (160,120)
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <stdlib.h>
#include <string.h>

LOG_MODULE_REGISTER(water_turret, LOG_LEVEL_INF);

/* ── 帧参数 ── */
#define FRAME_W  320
#define FRAME_H  240
#define CENTER_X (FRAME_W / 2)
#define CENTER_Y (FRAME_H / 2)

/* ── 运行时可调参数 ── */
static struct {
	int32_t h_center;       /* μs - 水平中点 */
	int32_t v_center;       /* μs - 垂直中点 */
	int32_t slew_rate;      /* μs/cycle - 最大脉宽变化速率 */
	int32_t interval_ms;    /* ms - 控制循环周期 */
	int32_t ema_num;        /* EMA 分子 */
	int32_t ema_den;        /* EMA 分母 */
	int32_t pid_kp;         /* ×0.01 */
	int32_t pid_ki;         /* ×0.01 */
	int32_t pid_kd;         /* ×0.01 */
	int32_t pid_imax;       /* 积分饱和限制 */
	int32_t fire_cooldown_ms;
	int32_t fire_pulse_ms;
	int32_t deadzone;       /* 像素 - 死区半径 */
	int32_t fire_threshold; /* 像素 - 误差小于此值时自动开火 */
} g_tune = {
	.h_center       = 1500,
	.v_center       = 1500,
	.slew_rate      = 80,
	.interval_ms    = 20,
	.ema_num        = 3,
	.ema_den        = 20,
	.pid_kp         = 40,
	.pid_ki         = 5,
	.pid_kd         = 60,
	.pid_imax       = 3000,
	.fire_cooldown_ms = 1000,
	.fire_pulse_ms  = 200,
	.deadzone       = 10,
	.fire_threshold = 25,
};

/* ── 设备树节点 ── */
static const struct pwm_dt_spec servo_h = PWM_DT_SPEC_GET(DT_ALIAS(servo_h));
static const struct pwm_dt_spec servo_v = PWM_DT_SPEC_GET(DT_ALIAS(servo_v));
static const struct gpio_dt_spec relay = GPIO_DT_SPEC_GET(DT_ALIAS(relay_fire), gpios);
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct device *k210_uart = DEVICE_DT_GET(DT_ALIAS(k210_uart));

/* ── UART 接收缓冲 ── */
#define UART_BUF_SIZE 64
static char uart_buf[UART_BUF_SIZE];
static int uart_buf_pos;

/* ISR → 主循环消息队列 (解决 sscanf 不可在 ISR 调用的问题) */
static K_MSGQ_DEFINE(uart_msgq, UART_BUF_SIZE, 8, 1);

/* ── 追踪目标 (原子更新) ── */
static atomic_t target_cx = ATOMIC_INIT(-1);  /* -1 = 无目标 */
static atomic_t target_cy = ATOMIC_INIT(-1);
static atomic_t target_valid = ATOMIC_INIT(0);

/* ── 状态 ── */
static struct gpio_callback button_cb;
static struct k_work fire_work;
static struct k_timer relay_timer;
static atomic_t relay_active = ATOMIC_INIT(0);
static int64_t last_fire_time;
static atomic_t fire_count = ATOMIC_INIT(0);

/* ── 模式 ── */
enum turret_mode {
	MODE_IDLE = 0,      /* 舵机静止在中点 */
	MODE_TRACKING = 1,  /* 自动追踪 */
};
static volatile enum turret_mode g_mode = MODE_TRACKING;

/* ── 辅助函数 ── */

static inline int32_t clamp_i32(int32_t val, int32_t lo, int32_t hi)
{
	if (val < lo) return lo;
	if (val > hi) return hi;
	return val;
}

static int servo_set_pulse(const struct pwm_dt_spec *spec, uint32_t pulse_ns)
{
	return pwm_set_pulse_dt(spec, pulse_ns);
}

/* ── UART 解析: "T,cx,cy,w,h\n" 或 "N\n" ── */
static void parse_uart_line(const char *line)
{
	if (line[0] == 'T' && line[1] == ',') {
		int cx, cy, w, h;
		int n = sscanf(line + 2, "%d,%d,%d,%d", &cx, &cy, &w, &h);
		if (n >= 2) {
			atomic_set(&target_cx, cx);
			atomic_set(&target_cy, cy);
			atomic_set(&target_valid, 1);
		}
	} else if (line[0] == 'N') {
		atomic_set(&target_valid, 0);
	}
}

/* ── UART 中断接收 (仅缓冲行，不在 ISR 中解析) ── */
static void uart_isr(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	if (!uart_irq_update(dev)) {
		return;
	}

	while (uart_irq_rx_ready(dev)) {
		uint8_t c;
		int ret = uart_fifo_read(dev, &c, 1);
		if (ret <= 0) {
			break;
		}

		if (c == '\n' || c == '\r') {
			if (uart_buf_pos > 0) {
				uart_buf[uart_buf_pos] = '\0';
				/* 将完整行送入消息队列，主循环解析 */
				k_msgq_put(&uart_msgq, uart_buf, K_NO_WAIT);
				uart_buf_pos = 0;
			}
		} else {
			if (uart_buf_pos < UART_BUF_SIZE - 1) {
				uart_buf[uart_buf_pos++] = (char)c;
			} else {
				/* 溢出，丢弃本行 */
				uart_buf_pos = 0;
			}
		}
	}
}

/* ── 继电器关闭回调 (timer expiry) ── */
static void relay_timer_expiry(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	gpio_pin_set_dt(&relay, 0);
	atomic_set(&relay_active, 0);
}

/* ── 继电器开火 (非阻塞) ── */
static void fire_once(const char *src)
{
	/* 防止重入: 已在开火中则跳过 */
	if (atomic_get(&relay_active)) {
		return;
	}

	int64_t now = k_uptime_get();
	if ((now - last_fire_time) < g_tune.fire_cooldown_ms) {
		return;
	}

	atomic_set(&relay_active, 1);
	last_fire_time = now;

	int n = atomic_inc(&fire_count) + 1;
	LOG_INF("FIRE #%d (%s)", n, src);
	gpio_pin_set_dt(&relay, 1);
	/* 非阻塞: timer 到期后自动关闭继电器 */
	k_timer_start(&relay_timer, K_MSEC(g_tune.fire_pulse_ms), K_NO_WAIT);
}

static void fire_work_handler(struct k_work *w)
{
	ARG_UNUSED(w);
	fire_once("button");
}

static void button_isr(const struct device *port, struct gpio_callback *cb,
		       gpio_port_pins_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);
	k_work_submit(&fire_work);
}

/* ── Shell 命令 ── */

static int cmd_tune_set(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 3) {
		shell_print(sh, "Usage: tune set <param> <value>");
		return -EINVAL;
	}

	const char *name = argv[1];
	int32_t val = atoi(argv[2]);

	if (strcmp(name, "h_center") == 0) { g_tune.h_center = val; }
	else if (strcmp(name, "v_center") == 0) { g_tune.v_center = val; }
	else if (strcmp(name, "slew_rate") == 0) { g_tune.slew_rate = val; }
	else if (strcmp(name, "interval") == 0) { g_tune.interval_ms = val; }
	else if (strcmp(name, "ema_num") == 0) { g_tune.ema_num = val; }
	else if (strcmp(name, "ema_den") == 0) { g_tune.ema_den = val; }
	else if (strcmp(name, "pid_kp") == 0) { g_tune.pid_kp = val; }
	else if (strcmp(name, "pid_ki") == 0) { g_tune.pid_ki = val; }
	else if (strcmp(name, "pid_kd") == 0) { g_tune.pid_kd = val; }
	else if (strcmp(name, "pid_imax") == 0) { g_tune.pid_imax = val; }
	else if (strcmp(name, "deadzone") == 0) { g_tune.deadzone = val; }
	else if (strcmp(name, "fire_threshold") == 0) { g_tune.fire_threshold = val; }
	else if (strcmp(name, "fire_cooldown") == 0) { g_tune.fire_cooldown_ms = val; }
	else if (strcmp(name, "fire_pulse") == 0) { g_tune.fire_pulse_ms = val; }
	else {
		shell_error(sh, "Unknown param: %s", name);
		return -EINVAL;
	}

	shell_print(sh, "OK: %s = %d", name, val);
	return 0;
}

static int cmd_tune_show(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "h_center=%d v_center=%d", g_tune.h_center, g_tune.v_center);
	shell_print(sh, "slew_rate=%d interval=%d", g_tune.slew_rate, g_tune.interval_ms);
	shell_print(sh, "ema=%d/%d", g_tune.ema_num, g_tune.ema_den);
	shell_print(sh, "pid: kp=%d ki=%d kd=%d imax=%d",
		    g_tune.pid_kp, g_tune.pid_ki, g_tune.pid_kd, g_tune.pid_imax);
	shell_print(sh, "deadzone=%d fire_threshold=%d", g_tune.deadzone, g_tune.fire_threshold);
	shell_print(sh, "fire: cooldown=%d pulse=%d", g_tune.fire_cooldown_ms, g_tune.fire_pulse_ms);
	return 0;
}

static int cmd_mode(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_print(sh, "mode: %d (0=idle, 1=track)", g_mode);
		return 0;
	}
	g_mode = atoi(argv[1]);
	shell_print(sh, "mode → %d", g_mode);
	return 0;
}

static int cmd_fire(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	fire_once("shell");
	shell_print(sh, "fired");
	return 0;
}

static int cmd_center(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	servo_set_pulse(&servo_h, PWM_USEC(g_tune.h_center));
	servo_set_pulse(&servo_v, PWM_USEC(g_tune.v_center));
	shell_print(sh, "servos → center (%d, %d)", g_tune.h_center, g_tune.v_center);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_tune,
	SHELL_CMD(set, NULL, "tune set <param> <val>", cmd_tune_set),
	SHELL_CMD(show, NULL, "Show all params", cmd_tune_show),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(tune, &sub_tune, "Tune parameters", NULL);
SHELL_CMD_REGISTER(mode, NULL, "Get/set mode (0=idle,1=track)", cmd_mode);
SHELL_CMD_REGISTER(fire, NULL, "Manual fire", cmd_fire);
SHELL_CMD_REGISTER(center, NULL, "Move servos to center", cmd_center);

/* ── 主程序 ── */
int main(void)
{
	int ret;

	LOG_INF("=== Water Turret — K210 + ESP32-S3 ===");

	/* ── UART1 for K210 ── */
	if (!device_is_ready(k210_uart)) {
		LOG_ERR("K210 UART not ready");
		return -ENODEV;
	}
	uart_irq_callback_set(k210_uart, uart_isr);
	uart_irq_rx_enable(k210_uart);
	LOG_INF("K210 UART ready (115200)");

	/* ── 按钮 ── */
	if (!gpio_is_ready_dt(&button)) {
		LOG_ERR("button not ready");
		return -ENODEV;
	}
	gpio_pin_configure_dt(&button, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
	k_work_init(&fire_work, fire_work_handler);
	gpio_init_callback(&button_cb, button_isr, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb);

	/* ── 继电器 ── */
	if (!gpio_is_ready_dt(&relay)) {
		LOG_ERR("relay not ready");
		return -ENODEV;
	}
	gpio_pin_configure_dt(&relay, GPIO_OUTPUT_INACTIVE);
	k_timer_init(&relay_timer, relay_timer_expiry, NULL);
	LOG_INF("relay ready: GPIO38");

	/* ── 舵机 ── */
	if (!pwm_is_ready_dt(&servo_h)) {
		LOG_ERR("servo_h not ready");
		return -ENODEV;
	}
	if (!pwm_is_ready_dt(&servo_v)) {
		LOG_ERR("servo_v not ready");
		return -ENODEV;
	}
	servo_set_pulse(&servo_h, PWM_USEC(g_tune.h_center));
	servo_set_pulse(&servo_v, PWM_USEC(g_tune.v_center));
	LOG_INF("servos ready: H=%d V=%d (us)", g_tune.h_center, g_tune.v_center);

	LOG_INF("init complete, mode=TRACKING");

	/* ── PID 状态 ── */
	int32_t h_pulse = g_tune.h_center * 1000; /* ns */
	int32_t v_pulse = g_tune.v_center * 1000;
	int32_t ema_cx = CENTER_X;
	int32_t ema_cy = CENTER_Y;
	int32_t integral_h = 0, integral_v = 0;
	int32_t prev_err_h = 0, prev_err_v = 0;
	int32_t no_target_count = 0;

	/* ── 主控制循环 ── */
	while (1) {
		k_msleep(g_tune.interval_ms);

		/* 从消息队列读取 UART 行并解析 (线程安全) */
		char line[UART_BUF_SIZE];
		while (k_msgq_get(&uart_msgq, line, K_NO_WAIT) == 0) {
			parse_uart_line(line);
		}

		if (g_mode == MODE_IDLE) {
			continue;
		}

		/* 读取目标坐标 */
		int valid = atomic_get(&target_valid);
		int32_t cx = atomic_get(&target_cx);
		int32_t cy = atomic_get(&target_cy);

		if (!valid || cx < 0) {
			no_target_count++;
			if (no_target_count > 50) {
				/* 丢失目标超过 1 秒，PID 重置 */
				integral_h = 0;
				integral_v = 0;
				prev_err_h = 0;
				prev_err_v = 0;
			}
			continue;
		}
		no_target_count = 0;

		/* EMA 滤波 */
		ema_cx = (g_tune.ema_num * cx + (g_tune.ema_den - g_tune.ema_num) * ema_cx)
			 / g_tune.ema_den;
		ema_cy = (g_tune.ema_num * cy + (g_tune.ema_den - g_tune.ema_num) * ema_cy)
			 / g_tune.ema_den;

		/* 计算误差 (像素) */
		int32_t err_h = ema_cx - CENTER_X;  /* 正 = 目标在右 */
		int32_t err_v = ema_cy - CENTER_Y;  /* 正 = 目标在下 */

		/* 死区 */
		if (abs(err_h) < g_tune.deadzone) err_h = 0;
		if (abs(err_v) < g_tune.deadzone) err_v = 0;

		/* PID 计算 (× 0.01 精度) */
		integral_h = clamp_i32(integral_h + err_h, -g_tune.pid_imax, g_tune.pid_imax);
		integral_v = clamp_i32(integral_v + err_v, -g_tune.pid_imax, g_tune.pid_imax);

		int32_t delta_h = (g_tune.pid_kp * err_h
				   + g_tune.pid_ki * integral_h
				   + g_tune.pid_kd * (err_h - prev_err_h)) / 100;
		int32_t delta_v = (g_tune.pid_kp * err_v
				   + g_tune.pid_ki * integral_v
				   + g_tune.pid_kd * (err_v - prev_err_v)) / 100;

		prev_err_h = err_h;
		prev_err_v = err_v;

		/* Slew rate 限制 (μs/cycle) */
		delta_h = clamp_i32(delta_h, -g_tune.slew_rate, g_tune.slew_rate);
		delta_v = clamp_i32(delta_v, -g_tune.slew_rate, g_tune.slew_rate);

		/* 更新脉宽 (ns) - 注意: 目标在右 → 舵机需向右转 → 脉宽减小 */
		h_pulse -= delta_h * 1000;
		v_pulse += delta_v * 1000;  /* 目标在下 → 舵机下倾 → 脉宽增大 */

		/* 限幅 */
		h_pulse = clamp_i32(h_pulse, PWM_USEC(500), PWM_USEC(2500));
		v_pulse = clamp_i32(v_pulse, PWM_USEC(500), PWM_USEC(2500));

		/* 输出到舵机 */
		servo_set_pulse(&servo_h, (uint32_t)h_pulse);
		servo_set_pulse(&servo_v, (uint32_t)v_pulse);

		/* 自动开火: 误差足够小时触发 */
		if (abs(err_h) < g_tune.fire_threshold &&
		    abs(err_v) < g_tune.fire_threshold) {
			fire_once("auto");
		}
	}

	return 0;
}
