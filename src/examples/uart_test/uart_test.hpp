#pragma once
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_attitude.h>

class AttitudeUart : public ModuleBase<AttitudeUart>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	AttitudeUart(const char *uart_device = "/dev/pts/4");
	~AttitudeUart() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	static AttitudeUart *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	int _uart_fd{-1};
	char _uart_path[32];

	uORB::Subscription _att_sub{ORB_ID(vehicle_attitude)};
};
