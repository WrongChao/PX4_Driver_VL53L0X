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

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/distance_sensor.h>

#include <board_config.h>

#include <vl53l0x_api.h>
#include <vl53l0x_platform.h>
#include <vl53l0x_i2c_platform.h>

/* Configuration Constants */
#define VL53L0X_I2C_BUS 		PX4_I2C_BUS_EXPANSION
#define VL53L0X_I2C_BASEADDR 	0x52 /* 7-bit address. 8-bit address is 0xE0 */
#define VL53L0X_DEVICE_PATH	"/dev/vl53l0x"

/* MB12xx Registers addresses */

#define SRF02_TAKE_RANGE_REG	0x51		/* Measure range Register */

/* Device limits */
#define VL53L0X_MIN_DISTANCE 	(0.05f)
#define VL53L0X_MAX_DISTANCE 	(2.00f)

#define VL53L0X_CONVERSION_INTERVAL 	35000 /* 60ms for one sonar */
#define TICKS_BETWEEN_SUCCESIVE_FIRES 	20000 /* 30ms between each sonar measurement (watch out for interference!) */

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
	virtual int			ioctl(device::file_t *filp, int cmd, unsigned long arg);

	virtual int         i2c_transfer(const uint8_t *send, unsigned send_len, uint8_t *recv, unsigned recv_len);

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
extern "C" { __EXPORT int VL53L0X_main(int argc, char *argv[]);}

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

	// XXX we should find out why we need to wait 200 ms here
	usleep(200000);

	/* check for connected rangefinders on each i2c port:
	   We start from i2c base address (0x70 = 112) and count downwards
	   So second iteration it uses i2c address 111, third iteration 110 and so on*/
	for (unsigned counter = 0; counter <= MB12XX_MAX_RANGEFINDERS; counter++) {
		_index_counter = VL53L0X_I2C_BASEADDR + counter * 2;	/* set temp sonar i2c address to base adress + counter * 2 */
		set_device_address(_index_counter);			/* set I2c port to temp sonar i2c adress */
		VL53L0X_Error ret = init_device(_index_counter);

		if (ret2 == VL53L0X_ERROR_NONE) { /* sonar is present -> store address_index in array */
			addr_ind.push_back(_index_counter);
			DEVICE_DEBUG("sonar added");
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

	ret = OK;
	/* sensor is ok, but we don't really know if it is within range */
	_sensor_ok = true;

	return ret;
}

VL53L0X_Error
init_device(uint8_t address) {
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	VL53L0X_DeviceInfo_t vl53l0x_dev_info;
	VL53L0X_Dev_t *pMyDevice = &_vl53l0x_dev;
	pMyDevice->I2cDevAddr = address;
	pMyDevice->comms_type = 1;
	pMyDevice->comms_speed_khz = 400;

	Status = VL53L0X_DataInit(pMyDevice); // Data initialization
	if (Status != VL53L0X_ERROR_NONE) {
		print_pal_error(Status);
		return Status;
	}
	Status = VL53L0X_GetDeviceInfo(pMyDevice, &vl53l0x_dev_info);
	if (Status != VL53L0X_ERROR_NONE) {
		print_pal_error(Status);
		return Status;
	}
	if ((vl53l0x_dev_info.ProductRevisionMajor != 1) && (vl53l0x_dev_info.ProductRevisionMinor != 1)) {
		Status = VL53L0X_ERROR_NOT_SUPPORTED;
		print_pal_error(Status);
		return Status;
	}

	Status = vl53l0x_measure_init(pMyDevice);
	vl53l0x_status = Status;
	if (Status != VL53L0X_ERROR_NONE) {
		print_pal_error(Status);
		return Status;
	}
	return Status;
}

VL53L0X_Error
vl53l0x_measure_init(VL53L0X_Dev_t *pMyDevice) {
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;

	//FixPoint1616_t LimitCheckCurrent;
	uint32_t refSpadCount;
	uint8_t isApertureSpads;
	uint8_t VhvSettings;
	uint8_t PhaseCal;
	// Device Initialization
	Status = VL53L0X_StaticInit(pMyDevice);
	if (Status != VL53L0X_ERROR_NONE) {
		//printf ("Call of VL53L0X_StaticInit\n");
		print_pal_error(Status);
		return Status;
	}
	// Device Initialization
	Status = VL53L0X_PerformRefCalibration(pMyDevice, &VhvSettings, &PhaseCal);
	if (Status != VL53L0X_ERROR_NONE) {
		//printf ("Call of VL53L0X_PerformRefCalibration\n");
		print_pal_error(Status);
		return Status;
	}

	// needed if a coverglass is used and no calibration has been performed
	Status = VL53L0X_PerformRefSpadManagement(pMyDevice, &refSpadCount, &isApertureSpads);
	if (Status != VL53L0X_ERROR_NONE) {
		//printf ("Call of VL53L0X_PerformRefSpadManagement\n");
		//printf ("refSpadCount = %d, isApertureSpads = %d\n", refSpadCount, isApertureSpads);
		print_pal_error(Status);
		return Status;
	}

	// no need to do this when we use VL53L0X_PerformSingleRangingMeasurement
	Status = VL53L0X_SetDeviceMode(pMyDevice, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING); // Setup in single ranging mode
	if (Status != VL53L0X_ERROR_NONE) {
		//printf ("Call of VL53L0X_SetDeviceMode\n");
		print_pal_error(Status);
		return Status;
	}
	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetLimitCheckEnable(pMyDevice,
			VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
	}
	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetLimitCheckEnable(pMyDevice,
			VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
	}

	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetLimitCheckValue(pMyDevice,
			VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
			(FixPoint1616_t)(0.25 * 65536));
	}
	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetLimitCheckValue(pMyDevice,
			VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
			(FixPoint1616_t)(18 * 65536));
	}
	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pMyDevice,
			200000);
	}
	// start ranging measure
	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_StartMeasurement(pMyDevice);
	}
	if (Status != VL53L0X_ERROR_NONE) {
		print_pal_error(Status);
		return Status;
	}

	return Status;
}

int
VL53L0X::i2c_transfer(const uint8_t *send, unsigned send_len, uint8_t *recv, unsigned recv_len)
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

int
VL53L0X::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_measure_ticks = 0;
				return OK;

			/* external signalling (DRDY) not supported */
			case SENSOR_POLLRATE_EXTERNAL:

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_ticks = USEC2TICK(_cycling_rate);

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();

					}

					return OK;
				}

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* convert hz to tick interval via microseconds */
					int ticks = USEC2TICK(1000000 / arg);

					/* check against maximum rate */
					if (ticks < USEC2TICK(_cycling_rate)) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_measure_ticks = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_measure_ticks == 0) {
			return SENSOR_POLLRATE_MANUAL;
		}

		return (1000 / _measure_ticks);

	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			ATOMIC_ENTER;

			if (!_reports->resize(arg)) {
				ATOMIC_LEAVE;
				return -ENOMEM;
			}

			ATOMIC_LEAVE;

			return OK;
		}

	case SENSORIOCRESET:
		/* XXX implement this */
		return -EINVAL;

	default:
		/* give it to the superclass */
		return I2C::ioctl(filp, cmd, arg);
	}
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

	status = VL53L0X_GetRangingMeasurementData(&_vl53l0x_dev, &vl53l0x_data);
	if (status != VL53L0X_ERROR_NONE) {
		//printf("error:Call of VL53L0X_PerformSingleRangingMeasurement\n");
		return ret;
	}

	uint16_t distance_cm = val[0] << 8 | val[1];
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

	/* notify about state change */
	struct subsystem_info_s info = {};
	info.present = true;
	info.enabled = true;
	info.ok = true;
	info.subsystem_type = subsystem_info_s::SUBSYSTEM_TYPE_RANGEFINDER;

	static orb_advert_t pub = nullptr;

	if (pub != nullptr) {
		orb_publish(ORB_ID(subsystem_info), pub, &info);

	} else {
		pub = orb_advertise(ORB_ID(subsystem_info), &info);

	}
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

	/* Is there a collect->measure gap? Yes, and the timing is set equal to the cycling_rate
	   Otherwise the next sonar would fire without the first one having received its reflected sonar pulse */

	if (_measure_ticks > USEC2TICK(_cycling_rate)) {

		/* schedule a fresh cycle call when we are ready to measure again */
		work_queue(HPWORK,
			   &_work,
			   (worker_t)&VL53L0X::cycle_trampoline,
			   this,
			   _measure_ticks - USEC2TICK(_cycling_rate));
		return;
	}
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
transfer(const uint8_t *send, unsigned send_len, uint8_t *recv, unsigned recv_len) {
	if(g_dev != nullptr) {
		send = send == 0 ? nullptr : send;
		recv = recv == 0 ? nullptr : recv;
		return g_dev->i2c_transfer(send, send_len, recv, recv_len);
	}
	return PX4_ERROR;
}

/**
 * Start the driver.
 */
void
start(uint8_t rotation)
{
	int fd;

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

	/* set the poll rate to default, starts automatic data collection */
	fd = open(VL53L0X_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		goto fail;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
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
	int ret;

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

	/* start the sensor polling at 2Hz */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2)) {
		errx(1, "failed to set 2Hz poll rate");
	}

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 5; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 2000);

		if (ret != 1) {
			errx(1, "timed out waiting for sensor data");
		}

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			err(1, "periodic read failed");
		}

		warnx("periodic read %u", i);
		warnx("valid %u", (float)report.current_distance > report.min_distance
		      && (float)report.current_distance < report.max_distance ? 1 : 0);
		warnx("measurement: %0.3f", (double)report.current_distance);
		warnx("time:        %llu", report.timestamp);
	}

	/* reset the sensor polling to default rate */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT)) {
		errx(1, "failed to set default poll rate");
	}

	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(VL53L0X_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "failed ");
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		err(1, "driver reset failed");
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "driver poll restart failed");
	}

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
