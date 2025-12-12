#include "uart_test.hpp"
#include <termios.h>

using namespace time_literals;

AttitudeUart::AttitudeUart(const char *uart_device) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	strncpy(_uart_path, uart_device, sizeof(_uart_path));
}

AttitudeUart::~AttitudeUart()
{
	if (_uart_fd >= 0) {
		close(_uart_fd);
	}
}

bool AttitudeUart::init()
{
	// 打开串口
	_uart_fd = open(_uart_path, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_uart_fd < 0) {
		PX4_ERR("Failed to open UART: %s", _uart_path);
		return false;
	}

	struct termios uart_cfg;
	tcgetattr(_uart_fd, &uart_cfg);

	cfsetispeed(&uart_cfg, B115200);
	cfsetospeed(&uart_cfg, B115200);

	uart_cfg.c_cflag &= ~PARENB;
	uart_cfg.c_cflag &= ~CSTOPB;
	uart_cfg.c_cflag &= ~CSIZE;
	uart_cfg.c_cflag |= CS8;
	uart_cfg.c_cflag |= CREAD | CLOCAL;
	uart_cfg.c_cflag &= ~CRTSCTS;

	tcsetattr(_uart_fd, TCSANOW, &uart_cfg);

	PX4_INFO("UART opened: %s", _uart_path);

	// 每 20ms 调度一次
	ScheduleOnInterval(1_s);

	return true;
}

void AttitudeUart::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	vehicle_attitude_s att;

	if (_att_sub.updated() && _att_sub.update(&att)) {

		// 输出四元数
		char buf[128];
		int len = snprintf(buf, sizeof(buf),
				   "ATT q: %.4f %.4f %.4f %.4f\n",
				   (double)att.q[0],
				   (double)att.q[1],
				   (double)att.q[2],
				   (double)att.q[3]);
		// PX4_INFO("received: %s", buf);
		if (_uart_fd >= 0) {
			if(write(_uart_fd, buf, len) > 0){
				PX4_INFO("UART write failed");
			}
		}
	}
}

/*-------------------------------------------
 *    ModuleBase hooks
 *------------------------------------------*/

AttitudeUart *AttitudeUart::instantiate(int argc, char *argv[])
{
	const char *uart_path = "/dev/ttyS4";

	if (argc > 1) {
		uart_path = argv[1];
	}

	AttitudeUart *instance = new AttitudeUart(uart_path);

	if (!instance) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (!instance->init()) {
		delete instance;
		return nullptr;
	}

	return instance;
}

int AttitudeUart::task_spawn(int argc, char *argv[])
{
	AttitudeUart *instance = new AttitudeUart();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int AttitudeUart::custom_command(int argc, char *argv[])
{
	return print_usage("Unknown command");
}

int AttitudeUart::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
	### Description
	Publish vehicle attitude through UART periodically.
	)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("attitude_uart", "driver");
	PRINT_MODULE_USAGE_COMMAND("start [uart_dev]");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int uart_test_main(int argc, char *argv[])
{
	return AttitudeUart::main(argc, argv);
}
