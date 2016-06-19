#include "ets_sys.h"
#include "osapi.h"
#include "gpio.h"
#include "os_type.h"
#include "uart_hw.h"
#include "ip_addr.h"
#include "espconn.h"
#include <errno.h>
#include <stdlib.h>

#define USE_GPIO1

static volatile unsigned pwm_duty[2];

static void pwm_timer_fn(void *arg)
{
	static unsigned cycle = 0;

	if (++cycle == 10)
		cycle = 0;

	if (cycle < pwm_duty[0])
		GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, 1);
	else
		GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, 1);

	if (cycle < pwm_duty[1])
		GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, 4);
	else
		GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, 4);
}


static volatile int pwm_steer;
static os_timer_t steer_off_timer;

static void steer_timer_fn(void *arg)
{
	os_timer_arm_us(&steer_off_timer,
			1000 + (pwm_steer + 10) * 1000 / 19, 0);
	GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, 2);
}

static void steer_off_timer_fn(void *arg)
{
	GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, 2);
}


#ifdef USE_GPIO1

static void nullPutchar(char c)
{
}

#else

static void stdoutUartTxd(char c)
{
	//Wait until there is room in the FIFO
	while (((READ_PERI_REG(UART_STATUS(0))>>UART_TXFIFO_CNT_S)&UART_TXFIFO_CNT)>=126);
	//Send the character
	WRITE_PERI_REG(UART_FIFO(0), c);
}

static void stdoutPutchar(char c)
{
	//convert \n -> \r\n
	if (c=='\n') stdoutUartTxd('\r');
	stdoutUartTxd(c);
}

void stdoutInit(void)
{
	//Enable TxD pin
	PIN_PULLUP_DIS(PERIPHS_IO_MUX_U0TXD_U);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, FUNC_U0TXD);

	//Set baud rate and other serial parameters to 115200,n,8,1
	uart_div_modify(0, UART_CLK_FREQ/BIT_RATE_115200);
	WRITE_PERI_REG(UART_CONF0(0), (STICK_PARITY_DIS)|(ONE_STOP_BIT << UART_STOP_BIT_NUM_S)| \
				(EIGHT_BITS << UART_BIT_NUM_S));

	//Reset tx & rx fifo
	SET_PERI_REG_MASK(UART_CONF0(0), UART_RXFIFO_RST|UART_TXFIFO_RST);
	CLEAR_PERI_REG_MASK(UART_CONF0(0), UART_RXFIFO_RST|UART_TXFIFO_RST);
	//Clear pending interrupts
	WRITE_PERI_REG(UART_INT_CLR(0), 0xffff);

	//Install our own putchar handler
	os_install_putc1((void *)stdoutPutchar);
}

#endif

static void udp_rx_cb(void *arg, char *data, unsigned short length)
{
	char buf[100], *pthrust, *psteer;
	int thrust, steer;

	if (length >= sizeof(buf)) {
		memcpy(buf, data, 10);
		buf[10] = 0;
		ets_printf("%s: oversized packet: %d bytes [%s...]\n",
			   __func__, length, buf);
		return;
	}
	memcpy(buf, data, length);
	buf[length] = 0;
	errno = 0;
	thrust = strtol(buf, &pthrust, 0);
	if (errno == 0)
		steer = strtol(pthrust, &psteer, 0);

	if (errno != 0 || pthrust == buf || psteer == pthrust) {
		ets_printf("%s: couldn't parse packet: [%s]\n",
			   __func__, buf);
		return;
	}
	if (thrust > 10 || thrust < -10 ||
	    steer > 10 || steer < -10) {
		ets_printf("%s: bad parameters: thrust: %d, steer: %d\n",
			   __func__, thrust, steer);
		return;
	}
	ets_printf("%s: setting thrust: %d, steer: %d\n", __func__, thrust, steer);

	if (thrust >= 0) {
		pwm_duty[0] = 0;
		pwm_duty[1] = thrust;
	} else {
		pwm_duty[1] = 0;
		pwm_duty[0] = -thrust;
	}
	pwm_steer = steer;
}

static void udp_init(void)
{
	static esp_udp udp = {
		.local_port = 1025,
	};

	static struct espconn udpconn = {
		.type = ESPCONN_UDP,
		.proto.udp = &udp,
	};

	espconn_regist_recvcb(&udpconn, udp_rx_cb);
	espconn_create(&udpconn);
}


void user_init()
{
	static os_timer_t pwm_timer;
	static os_timer_t steer_timer;

	system_timer_reinit();

	/* set up GPIO0 */
	PIN_PULLUP_DIS(PERIPHS_IO_MUX_GPIO0_U);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0);
	GPIO_REG_WRITE(GPIO_ENABLE_W1TS_ADDRESS, 1);
	GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, 1);

	/* set up GPIO2 */
	PIN_PULLUP_DIS(PERIPHS_IO_MUX_GPIO2_U);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_GPIO2);
	GPIO_REG_WRITE(GPIO_ENABLE_W1TS_ADDRESS, 4);
	GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, 4);
#ifdef USE_GPIO1
	/* set up GPIO1 */
	PIN_PULLUP_DIS(PERIPHS_IO_MUX_U0TXD_U);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, FUNC_GPIO1);
	GPIO_REG_WRITE(GPIO_ENABLE_W1TS_ADDRESS, 2);
	GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, 2);
	os_install_putc1((void *)nullPutchar);
#else
	stdoutInit();
#endif
	udp_init();

	os_timer_setfn(&pwm_timer, pwm_timer_fn, NULL);
	os_timer_arm_us(&pwm_timer, 100, 1);

	os_timer_setfn(&steer_timer, steer_timer_fn, NULL);
	os_timer_setfn(&steer_off_timer, steer_off_timer_fn, NULL);
	os_timer_arm(&steer_timer, 10, 1);
}
