/****************************************************************************
 *
 *   Copyright (c) 2015, 2016 Airmind Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name Airmind nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file vl53l0x.cpp
 * @author Wrong Chao
 * @author Wrong Chao <wrongchao480@gmail.com>
 *
 * Driver for the vl53l0x connected via I2C.
 */

#include <px4_workqueue.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_getopt.h>

#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <vector>

#include <perf/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/distance_sensor.h>

#include <board_config.h>

#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"
#include "vl53l0x_i2c_platform.h"
#include "vl53l0x.hpp"

/* Configuration Constants */
#define VL53L0X_I2C_BUS 		PX4_I2C_BUS_EXPANSION
#define VL53L0X_I2C_BASEADDR 	0x29 /* 7-bit address. 8-bit address is 0x52 */
#define VL53L0X_DEVICE_PATH	"/dev/vl53l0x"

/* Device limits */
#define VL53L0X_MIN_DISTANCE 	(0.02f)
#define VL53L0X_MAX_DISTANCE 	(2.00f)

#define VL53L0X_CONVERSION_INTERVAL 	40000 /* 60ms for one sonar */
#define TICKS_BETWEEN_SUCCESIVE_FIRES 	40000 /* 30ms between each sonar measurement (watch out for interference!) */

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

class VL53L0X : public device::I2C
{
public:
	VL53L0X(uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING, int bus = VL53L0X_I2C_BUS,
		  int address = VL53L0X_I2C_BASEADDR);
	virtual ~VL53L0X();

	virtual int 		init();

	virtual ssize_t		read(device::file_t *filp, char *buffer, size_t buflen);

	virtual int         transfer_data(const uint8_t *send, unsigned send_len, uint8_t *recv, unsigned recv_len);

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void				print_info();

protected:
	virtual int			probe();

private:
	VL53L0X_Dev_t _vl53l0x_dev;
	uint8_t _rotation;
	float				_min_distance;
	float				_max_distance;
	work_s				_work;
	ringbuffer::RingBuffer		*_reports;
	bool				_sensor_ok;
	int					_measure_ticks;
	int					_class_instance;
	int					_orb_class_instance;

	orb_advert_t		_distance_sensor_topic;

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;

	uint8_t				_cycle_counter;	/* counter in cycle to change i2c adresses */
	int					_cycling_rate;	/* */
	uint8_t				_index_counter;	/* temporary sonar i2c address */
	std::vector<uint8_t>	addr_ind; 	/* temp sonar i2c address vector */
	std::vector<float>
	_latest_sonar_measurements; /* vector to store latest sonar measurements in before writing to report */


	/**
	* Test whether the device supported by the driver is present at a
	* specific address.
	*
	* @param address	The I2C bus address to probe.
	* @return			True if the device is present.
	*/
	int					probe_address(uint8_t address);

	/**
	* Initialise the automatic measurement state machine and start it.
	*
	* @note This function is called at open and error time.  It might make sense
	*       to make it more aggressive about resetting the bus in case of errors.
	*/
	void				start();

	/**
	* Stop the automatic measurement state machine.
	*/
	void				stop();

	/**
	* Set the min and max distance thresholds if you want the end points of the sensors
	* range to be brought in at all, otherwise it will use the defaults VL53L0X_MIN_DISTANCE
	* and VL53L0X_MAX_DISTANCE
	*/
	void				set_minimum_distance(float min);
	void				set_maximum_distance(float max);
	float				get_minimum_distance();
	float				get_maximum_distance();

	/**
	* VL53L0X device
	**/
	VL53L0X_Error       init_device(uint8_t address);
	VL53L0X_Error 		vl53l0x_measure_init(VL53L0X_Dev_t *pMyDevice);

	/**
	* Perform a poll cycle; collect from the previous measurement
	* and start a new one.
	*/
	void				cycle();
	int					collect();
	/**
	* Static trampoline from the workq context; because we don't have a
	* generic workq wrapper yet.
	*
	* @param arg		Instance pointer for the driver that is polling.
	*/
	static void			cycle_trampoline(void *arg);


};

/*
 * Driver 'main' command.
 */
extern "C" { __EXPORT int vl53l0x_main(int argc, char *argv[]);}

VL53L0X::VL53L0X(uint8_t rotation, int bus, int address) :
	I2C("vl53l0x", VL53L0X_DEVICE_PATH, bus, address, 100000),
	_rotation(rotation),
	_min_distance(VL53L0X_MIN_DISTANCE),
	_max_distance(VL53L0X_MAX_DISTANCE),
	_reports(nullptr),
	_sensor_ok(false),
	_measure_ticks(0),
	_class_instance(-1),
	_orb_class_instance(-1),
	_distance_sensor_topic(nullptr),
	_sample_perf(perf_alloc(PC_ELAPSED, "VL53L0X_read")),
	_comms_errors(perf_alloc(PC_COUNT, "VL53L0X_comms_errors")),
	_cycle_counter(0),	/* initialising counter for cycling function to zero */
	_cycling_rate(0),	/* initialising cycling rate (which can differ depending on one sonar or multiple) */
	_index_counter(0) 	/* initialising temp sonar i2c address to zero */

{
	/* enable debug() calls */
	_debug_enabled = false;

	/* work_cancel in the dtor will explode if we don't do this... */
	memset(&_work, 0, sizeof(_work));
}

VL53L0X::~VL53L0X()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	if (_class_instance != -1) {
		unregister_class_devname(RANGE_FINDER_BASE_DEVICE_PATH, _class_instance);
	}

	/* free perf counters */
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
VL53L0X::init()
{
	int ret = PX4_ERROR;

	/* do I2C init (and probe) first */
	if (I2C::init() != OK) {
		return ret;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(distance_sensor_s));

	_index_counter = VL53L0X_I2C_BASEADDR;	/* set temp sonar i2c address to base adress */
	set_device_address(_index_counter);		/* set I2c port to temp sonar i2c adress */

	if (_reports == nullptr) {
		return ret;
	}

	_class_instance = register_class_devname(RANGE_FINDER_BASE_DEVICE_PATH);

	/* get a publish handle on the range finder topic */
	struct distance_sensor_s ds_report = {};

	_distance_sensor_topic = orb_advertise_multi(ORB_ID(distance_sensor), &ds_report,
				 &_orb_class_instance, ORB_PRIO_LOW);

	if (_distance_sensor_topic == nullptr) {
		DEVICE_LOG("failed to create distance_sensor object. Did you start uOrb?");
	}

	/* check for connected rangefinders on each i2c port:
	   We start from i2c base address (0x70 = 112) and count downwards
	   So second iteration it uses i2c address 111, third iteration 110 and so on*/
	DEVICE_LOG("add sonar\n");
	for (unsigned counter = 0; counter <= 0; counter++) {
		_index_counter = VL53L0X_I2C_BASEADDR + counter * 2;	/* set temp sonar i2c address to base adress + counter * 2 */
		set_device_address(_index_counter);			/* set I2c port to temp sonar i2c adress */
		VL53L0X_Error ret2 = init_device(_index_counter);
		DEVICE_LOG("vl53l0x add error: %d\n", ret2);

		if (ret2 == VL53L0X_ERROR_NONE) { /* sonar is present -> store address_index in array */
			addr_ind.push_back(_index_counter);
			DEVICE_LOG("sonar added, addredd:%d", _index_counter);
			_latest_sonar_measurements.push_back(200);
		}
	}

	_index_counter = VL53L0X_I2C_BASEADDR;
	set_device_address(_index_counter); /* set i2c port back to base adress for rest of driver */

	/* if only one sonar detected, no special timing is required between firing, so use default */
	if (addr_ind.size() == 1) {
		_cycling_rate = VL53L0X_CONVERSION_INTERVAL;

	} else {
		_cycling_rate = TICKS_BETWEEN_SUCCESIVE_FIRES;
	}

	/* show the connected sonars in terminal */
	for (unsigned i = 0; i < addr_ind.size(); i++) {
		DEVICE_LOG("sonar %d with address %d added", (i + 1), addr_ind[i]);
	}

	DEVICE_DEBUG("Number of sonars connected: %zu", addr_ind.size());

	if (addr_ind.size() == 0) {
		return PX4_ERROR;
	}

	ret = OK;
	/* sensor is ok, but we don't really know if it is within range */
	_sensor_ok = true;

	start();

	return ret;
}

VL53L0X_Error
VL53L0X::init_device(uint8_t address) {
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	// VL53L0X_DeviceInfo_t vl53l0x_dev_info;
	VL53L0X_Dev_t *pMyDevice = &_vl53l0x_dev;
	pMyDevice->I2cDevAddr = address;
	pMyDevice->comms_type = 1;
	pMyDevice->comms_speed_khz = 100;

	DEVICE_DEBUG("VL53L0X_DataInit");
	Status = VL53L0X_DataInit(pMyDevice); // Data initialization
	if (Status != VL53L0X_ERROR_NONE) {
		DEVICE_LOG("vl53l0x data init error:%d", Status);
		return Status;
	}
	// DEVICE_DEBUG("VL53L0X_GetDeviceInfo");
	// Status = VL53L0X_GetDeviceInfo(pMyDevice, &vl53l0x_dev_info);
	// if (Status != VL53L0X_ERROR_NONE) {
	// 	DEVICE_LOG("vl53l0x get device info error: %d", Status);
	// 	return Status;
	// }
	// if ((vl53l0x_dev_info.ProductRevisionMajor != 1) && (vl53l0x_dev_info.ProductRevisionMinor != 1)) {
	// 	Status = VL53L0X_ERROR_NOT_SUPPORTED;
	// 	return Status;
	// }

	Status = vl53l0x_measure_init(pMyDevice);
	if (Status != VL53L0X_ERROR_NONE) {
		DEVICE_LOG("vl53l0x measure init error:%d", Status);
		return Status;
	}
	return Status;
}

VL53L0X_Error
VL53L0X::vl53l0x_measure_init(VL53L0X_Dev_t *pMyDevice) {
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;

	//FixPoint1616_t LimitCheckCurrent;
	// uint32_t refSpadCount;
	// uint8_t isApertureSpads;
	uint8_t VhvSettings;
	uint8_t PhaseCal;
	// Device Initialization
	DEVICE_DEBUG("VL53L0X_StaticInit");
	Status = VL53L0X_StaticInit(pMyDevice);
	if (Status != VL53L0X_ERROR_NONE) {
		DEVICE_LOG("static init error:%d", Status);
		return Status;
	}
	// Device Initialization
	DEVICE_DEBUG("VL53L0X_PerformRefCalibration");
	Status = VL53L0X_PerformRefCalibration(pMyDevice, &VhvSettings, &PhaseCal);
	if (Status != VL53L0X_ERROR_NONE) {
		DEVICE_LOG("perform refcalibration error:%d", Status);
		return Status;
	}

	// needed if a coverglass is used and no calibration has been performed
	// DEVICE_DEBUG("VL53L0X_PerformRefSpadManagement");
	// Status = VL53L0X_PerformRefSpadManagement(pMyDevice, &refSpadCount, &isApertureSpads);
	// if (Status != VL53L0X_ERROR_NONE) {
	// 	DEVICE_LOG("performrefspadmanagement error:%d", Status);
	// 	return Status;
	// }

	// no need to do this when we use VL53L0X_PerformSingleRangingMeasurement
	DEVICE_DEBUG("VL53L0X_SetDeviceMode");
	Status = VL53L0X_SetDeviceMode(pMyDevice, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
	if (Status != VL53L0X_ERROR_NONE) {
		DEVICE_LOG("set device mode error:%d", Status);
		return Status;
	}
	// if (Status == VL53L0X_ERROR_NONE) {
	// 	DEVICE_DEBUG("VL53L0X_SetLimitCheckEnable: %d", VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE);
	// 	Status = VL53L0X_SetLimitCheckEnable(pMyDevice,
	// 		VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
	// }
	// if (Status == VL53L0X_ERROR_NONE) {
	// 	DEVICE_DEBUG("VL53L0X_SetLimitCheckEnable: %d", VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE);
	// 	Status = VL53L0X_SetLimitCheckEnable(pMyDevice,
	// 		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
	// }

	// if (Status == VL53L0X_ERROR_NONE) {
	// 	DEVICE_DEBUG("VL53L0X_SetLimitCheckValue: %d", VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE);
	// 	Status = VL53L0X_SetLimitCheckValue(pMyDevice,
	// 		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
	// 		(FixPoint1616_t)(0.25 * 65536));
	// }
	// if (Status == VL53L0X_ERROR_NONE) {
	// 	DEVICE_DEBUG("VL53L0X_SetLimitCheckValue: %d",VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE);
	// 	Status = VL53L0X_SetLimitCheckValue(pMyDevice,
	// 		VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
	// 		(FixPoint1616_t)(18 * 65536));
	// }
	// if (Status == VL53L0X_ERROR_NONE) {
	// 	DEVICE_DEBUG("VL53L0X_SetMeasurementTimingBudgetMicroSeconds");
	// 	Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pMyDevice,
	// 		200000);
	// }
	// start ranging measure
	if (Status == VL53L0X_ERROR_NONE) {
		DEVICE_DEBUG("VL53L0X_StartMeasurement");
		Status = VL53L0X_StartMeasurement(pMyDevice);
	}
	if (Status != VL53L0X_ERROR_NONE) {
		return Status;
	}

	return Status;
}

int
VL53L0X::transfer_data(const uint8_t *send, unsigned send_len, uint8_t *recv, unsigned recv_len)
{
	return transfer(send, send_len, recv, recv_len);
}

int
VL53L0X::probe()
{
	return OK;
}

void
VL53L0X::set_minimum_distance(float min)
{
	_min_distance = min;
}

void
VL53L0X::set_maximum_distance(float max)
{
	_max_distance = max;
}

float
VL53L0X::get_minimum_distance()
{
	return _min_distance;
}

float
VL53L0X::get_maximum_distance()
{
	return _max_distance;
}

ssize_t
VL53L0X::read(device::file_t *filp, char *buffer, size_t buflen)
{

	unsigned count = buflen / sizeof(struct distance_sensor_s);
	struct distance_sensor_s *rbuf = reinterpret_cast<struct distance_sensor_s *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (_measure_ticks > 0) {

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (_reports->get(rbuf)) {
				ret += sizeof(*rbuf);
				rbuf++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement - run one conversion */
	do {
		_reports->flush();

		/* run the collection phase */
		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* state machine will have generated a report, copy it out */
		if (_reports->get(rbuf)) {
			ret = sizeof(*rbuf);
		}

	} while (0);

	return ret;
}

int
VL53L0X::collect()
{
	VL53L0X_Error status = VL53L0X_ERROR_NONE;
	VL53L0X_RangingMeasurementData_t vl53l0x_data;
	int	ret = -EIO;

	DEVICE_DEBUG("VL53L0X_GetRangingMeasurementData");
	status = VL53L0X_GetRangingMeasurementData(&_vl53l0x_dev, &vl53l0x_data);
	if (status != VL53L0X_ERROR_NONE) {
		DEVICE_DEBUG("error:Call of VL53L0X_PerformSingleRangingMeasurement: %d\n", status);
		return ret;
	}

	float distance_m = float(vl53l0x_data.RangeMilliMeter) * 1e-3f;

	struct distance_sensor_s report;
	report.timestamp = hrt_absolute_time();
	report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND;
	report.orientation = _rotation;
	report.current_distance = distance_m;
	report.min_distance = get_minimum_distance();
	report.max_distance = get_maximum_distance();
	report.covariance = 0.0f;
	/* TODO: set proper ID */
	report.id = 0;

	/* publish it, if we are the primary */
	if (_distance_sensor_topic != nullptr) {
		orb_publish(ORB_ID(distance_sensor), _distance_sensor_topic, &report);
	}

	_reports->force(&report);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	ret = OK;

	perf_end(_sample_perf);
	return ret;
}

void
VL53L0X::start()
{

	_reports->flush();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&VL53L0X::cycle_trampoline, this, 5);
}

void
VL53L0X::stop()
{
	work_cancel(HPWORK, &_work);
}

void
VL53L0X::cycle_trampoline(void *arg)
{

	VL53L0X *dev = (VL53L0X *)arg;

	dev->cycle();

}

void
VL53L0X::cycle()
{
	_index_counter = addr_ind[_cycle_counter]; /*sonar from previous iteration collect is now read out */
	set_device_address(_index_counter);

	/* perform collection */
	if (OK != collect()) {
		DEVICE_DEBUG("collection error");
		/* if error restart the measurement state machine */
		start();
		return;
	}

	/* change i2c adress to next sonar */
	_cycle_counter = _cycle_counter + 1;

	if (_cycle_counter >= addr_ind.size()) {
		_cycle_counter = 0;
	}


	/* schedule a fresh cycle call when we are ready to measure again */
	work_queue(HPWORK,
		   &_work,
		   (worker_t)&VL53L0X::cycle_trampoline,
		   this,
		   USEC2TICK(_cycling_rate));
}

void
VL53L0X::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	_reports->print_info("report queue");
}

/**
 * Local functions in support of the shell command.
 */
namespace  vl53l0x
{

VL53L0X	*g_dev;

void	start(uint8_t rotation);
void	stop();
void	test();
void	reset();
void	info();


int
vl53l0x_transfer(const uint8_t *send, unsigned send_len, uint8_t *recv, unsigned recv_len) {
	if(g_dev != nullptr) {
		usleep(10000);
		send = send == 0 ? nullptr : send;
		recv = recv == 0 ? nullptr : recv;
		int ret = g_dev->transfer_data(send, send_len, recv, recv_len);
		return ret;
	}
	return PX4_ERROR;
}

/**
 * Start the driver.
 */
void
start(uint8_t rotation)
{

	if (g_dev != nullptr) {
		errx(1, "already started");
	}

	/* create the driver */
	g_dev = new VL53L0X(rotation, VL53L0X_I2C_BUS);

	if (g_dev == nullptr) {
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	exit(0);

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Stop the driver
 */
void stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	} else {
		errx(1, "driver not running");
	}

	exit(0);
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{
	struct distance_sensor_s report;
	ssize_t sz;

	int fd = open(VL53L0X_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed (try 'VL53L0X start' if the driver is not running", VL53L0X_DEVICE_PATH);
	}

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		err(1, "immediate read failed");
	}

	warnx("single read");
	warnx("measurement: %0.2f m", (double)report.current_distance);
	warnx("time:        %llu", report.timestamp);

	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info()
{
	if (g_dev == nullptr) {
		errx(1, "driver not running");
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}

} /* namespace */

int
vl53l0x_main(int argc, char *argv[])
{
	// check for optional arguments
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;
	uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;


	while ((ch = px4_getopt(argc, argv, "R:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (uint8_t)atoi(myoptarg);
			PX4_INFO("Setting distance sensor orientation to %d", (int)rotation);
			break;

		default:
			PX4_WARN("Unknown option!");
		}
	}

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[myoptind], "start")) {
		vl53l0x::start(rotation);
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[myoptind], "stop")) {
		vl53l0x::stop();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[myoptind], "test")) {
		vl53l0x::test();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[myoptind], "reset")) {
		vl53l0x::reset();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[myoptind], "info") || !strcmp(argv[myoptind], "status")) {
		vl53l0x::info();
	}

	PX4_ERR("unrecognized command, try 'start', 'test', 'reset' or 'info'");
	return PX4_ERROR;
}
