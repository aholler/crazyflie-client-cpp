//
// Copyright (c) 2014 Alexander Holler <holler@ahsoftware.de>
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to
// the following conditions:
//
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
// CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
// SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

/// \author Alexander Holler

#include <iostream>
#include <cmath> // abs(float)
#include <chrono>
#include <thread>
#include <exception> // set_terminate
#include <cstdlib> // std::abort
#include <csignal>
#include <set>
#include <limits>

#include <linux/limits.h> // PATH_MAX
#include <sys/stat.h> // S_I*
#include <sys/types.h> // mkdir
#include <unistd.h> // readlink

#include <boost/foreach.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <cflie/CCrazyflie.h>

#include "Joystick.hpp"
#include "version.h"

static std::terminate_handler old_terminate;

static void deleter_terminate(void)
{
	// program terminated through an error (e.g. unhandled exception)
	if (old_terminate)
		old_terminate();
	else {
		// std::exception_ptr doesn't seem to exist with gcc 4.7.3 but
		// that isn't a problem because this code here is not reached
		// with gcc, because it has an standard old_terminate which
		// verbosely prints the error.
		//std::exception_ptr e(std::current_exception());
		//std::cout << e.what() << std::endl;
		std::abort();
	}
}

static void (*old_signal_handler)(int) = nullptr;

static bool interrupted = false;

void deleter_sig_handler(int)
{
	// program halted normally (e.g. CTRL-C)
	interrupted = true;
}

class Deleter {
        public:
                Deleter() {
                        old_signal_handler = std::signal(SIGINT, deleter_sig_handler); // catch CTRL-C
                        if (old_signal_handler ==  SIG_ERR)
                                old_signal_handler = nullptr;
                        old_terminate = std::set_terminate(deleter_terminate);
                }
                ~Deleter() {
                        // program halted normally
                }
};

static Deleter deleter; // Make sure the singleton gets deleted to kill the radio

class Settings {
	public:
		float min_thrust;
		float max_thrust;
		float trim_roll;
		float trim_pitch;
		float max_yaw;
		float max_rp_angle;
		float slew_rate;
		float slew_limit;
		bool x_mode;
		std::string link_uri;
		std::string input_device;
		std::string device_config_mapping;
		void print(void) const;
		void load(const std::string& path);
		void save(const std::string& path) const;
};

void Settings::print(void) const
{
	std::cout << "link_uri: '" << link_uri << '\'' << std::endl;
	std::cout << "input_device: '" << input_device << '\'' << std::endl;
	std::cout << "device_config_mapping: '" << device_config_mapping << '\'' << std::endl;
	std::cout << "min_thrust: " << min_thrust << std::endl;
	std::cout << "max_thrust: " << max_thrust << std::endl;
	std::cout << "trim_roll: " << trim_roll << std::endl;
	std::cout << "trim_pitch: " << trim_pitch << std::endl;
	std::cout << "max_yaw " << max_yaw << std::endl;
	std::cout << "max_rp_angle " << max_rp_angle << std::endl;
	std::cout << "slew_rate " << slew_rate << std::endl;
	std::cout << "slew_limit " << slew_limit << std::endl;
	std::cout << "x-mode " << x_mode << std::endl;
}

void Settings::load(const std::string& path)
{
	boost::property_tree::ptree ptree;
	try {
		read_json(path + "/config.json", ptree);
	} catch(...) {
		// just ignore all errors
	}
	link_uri = ptree.get<std::string>("link_uri", "radio://0/10/250K");
	input_device = ptree.get<std::string>("input_device", "");
	if (!input_device.empty())
		device_config_mapping = ptree.get<std::string>("device_config_mapping." + input_device, "");
	min_thrust = ptree.get<unsigned>("min_thrust", 25);
	max_thrust = ptree.get<unsigned>("max_thrust", 80);
	trim_roll = ptree.get<float>("trim_roll", 0);
	trim_pitch = ptree.get<float>("trim_pitch", 0);
	max_yaw = ptree.get<unsigned>("max_yaw", 200);
	max_rp_angle = ptree.get<unsigned>("max_rp", 30);
	slew_rate = ptree.get<unsigned>("slew_rate", 30);
	slew_limit = ptree.get<unsigned>("slew_limit", 45);
	x_mode = ptree.get<bool>("client_side_xmode", false);
}

void Settings::save(const std::string& path) const
{
	// Read, write instead of just write makes sure
	// only known values will be changed, leaving everthing
	// else unmodified.
	boost::property_tree::ptree ptree;
	try {
		read_json(path + "/config.json", ptree);
	} catch(...) {
		// just ignore all errors
	}
	ptree.put("link_uri", link_uri);
	//ptree.put("input_device", input_device); // only save/change it if we are saving the mapping too
	ptree.put("min_thrust", min_thrust);
	ptree.put("max_thrust", max_thrust);
	ptree.put("trim_roll", trim_roll);
	ptree.put("trim_pitch", trim_pitch);
	ptree.put("max_yaw", max_yaw);
	ptree.put("max_rp", max_rp_angle);
	ptree.put("slew_rate", slew_rate);
	ptree.put("slew_limit", slew_limit);
	ptree.put("client_side_xmode", x_mode);
	::mkdir(path.c_str(), S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH);
	try {
		write_json(path + "/config.json", ptree);
	} catch(...) {
		// just ignore all errors
	}
}


enum Joystick_button_function {
	Button_alt_hold,
	Button_trim_pitch,
	Button_trim_roll,
	Button_killswitch,
	Button_exit,
};

enum Joystick_axis_function {
	Axis_yaw,
	Axis_thrust,
	Axis_roll,
	Axis_pitch,
};

class Joystick_button_setting
{
	public:
		Joystick_button_setting(unsigned n)
			: number(n)
		{}
		unsigned number;
		Joystick_button_function function;
		float scale;
		bool operator==(const Joystick_button_setting& s) const {
			return number == s.number;
		}
		bool operator<(const Joystick_button_setting& s) const {
			return number < s.number;
		}
};

class Joystick_axis_setting
{
	public:
		Joystick_axis_setting(unsigned n)
			: number(n)
		{}
		unsigned number;
		Joystick_axis_function function;
		float scale;
		bool operator==(const Joystick_axis_setting& s) const {
			return number == s.number;
		}
		bool operator<(const Joystick_axis_setting& s) const {
			return number < s.number;
		}
};

static std::set<Joystick_axis_setting> joystick_axis_settings;
static std::set<Joystick_button_setting> joystick_button_settings;

static void load_input(const std::string& path, const std::string& mapping)
{
	boost::property_tree::ptree ptree;
	try {
		read_json(path + "/input/" + mapping + ".json", ptree);
	} catch(...) {
		std::cerr << "Error reading '" + path + "/input/" + mapping + ".json'" << std::endl;
		return;
	}
	BOOST_FOREACH(boost::property_tree::ptree::value_type &v, ptree.get_child("inputconfig.inputdevice.axis")) {
		std::string type (v.second.get<std::string>("type", "not defined"));
		unsigned number(v.second.get<unsigned>("id", 666));
		if (number == 666)
			continue;
		std::string key (v.second.get<std::string>("key", "not defined"));
		if (key == "not defined")
			continue;
		if (type == "Input.AXIS") {
			float scale(v.second.get<float>("scale", std::numeric_limits<float>::max()));
			if (scale == std::numeric_limits<float>::max())
				continue;
			Joystick_axis_function f(Axis_thrust);
			if (key == "thrust")
				f = Axis_thrust;
			else if (key == "yaw")
				f = Axis_yaw;
			else if (key == "roll")
				f = Axis_roll;
			else if (key == "pitch")
				f = Axis_pitch;
			else
				continue;
			Joystick_axis_setting s(number);
			s.function = f;
			s.scale = scale;
			joystick_axis_settings.insert(s);
		}
		else if (type == "Input.BUTTON") {
			float scale(v.second.get<float>("scale", std::numeric_limits<float>::max()));
			if (scale == std::numeric_limits<float>::max())
				continue;
			Joystick_button_function f(Button_exit);
			if (key == "pitchcal")
				f = Button_trim_pitch;
			else if (key == "rollcal")
				f = Button_trim_roll;
			else if (key == "estop")
				f = Button_killswitch;
			else if (key == "exit")
				f = Button_exit;
			else if (key == "althold")
				f = Button_alt_hold;
			else
				continue;
			Joystick_button_setting s(number);
			s.function = f;
			s.scale = scale;
			joystick_button_settings.insert(s);
		}
	}
}


//#define MAX_THRUST 65000
#define MAX_THRUST std::numeric_limits<uint16_t>::max()

static float deadband(float value, float threshold)
{
	if (abs(value) < threshold)
		value = 0;
	else if (value > 0)
		value -= threshold;
	else if (value < 0)
		value += threshold;
	return value/(1-threshold);
}

static uint16_t p2t(float percentage)
{
	// Convert a percentage to raw thrust
	return uint16_t(MAX_THRUST * (percentage / 100.0));
}

static void set_pitch(CCrazyflie &cflieCopter, float pitch, float roll, float max_rp_angle, float trim_pitch, float trim_roll, bool x_mode)
{
	// pitch, scale -1.0
	pitch *= max_rp_angle;
	pitch += trim_pitch;
	if (x_mode) {
		roll *= max_rp_angle;
		roll += trim_roll;
		roll = 0.707 * (roll - pitch);
		pitch = 0.707 * (roll + pitch);
		cflieCopter.setRoll(roll); // degree
	}
	cflieCopter.setPitch(pitch); // degree
}

static void set_roll(CCrazyflie &cflieCopter, float pitch, float roll, float max_rp_angle, float trim_pitch, float trim_roll, bool x_mode)
{
	roll *= max_rp_angle;
	roll += trim_roll;
	if (x_mode) {
		pitch *= max_rp_angle;
		pitch += trim_pitch;
		roll = 0.707 * (roll - pitch);
		pitch = 0.707 * (roll + pitch);
		cflieCopter.setPitch(pitch); // degree
	}
	cflieCopter.setRoll(roll); // degree
}

static void set_yaw(CCrazyflie &cflieCopter, float yaw, float max_yaw)
{
	cflieCopter.setYaw(deadband(yaw, 0.2) * max_yaw); // degree
}

static void set_thrust(CCrazyflie &cflieCopter, float thrust, bool alt_hold, float min_thrust, float max_thrust, float& old_thrust)
{
	if (alt_hold)
		// In hoover mode the meaning of thrust is different.
		// From the forum:
		// "A value of 32767 means do not change the target altitude,
		// 0 means decrease the target altitude fast,
		// 65535 means increase the target altitude fast."
		thrust = int(deadband(thrust, 0.2) * std::numeric_limits<int16_t>::max() + std::numeric_limits<int16_t>::max() + .5); // Convert to uint16
	else
		thrust = min_thrust + thrust * (max_thrust - min_thrust);
	// TODO: slew
	old_thrust = thrust;
	cflieCopter.setThrust(thrust);
}

static void set_althold(CCrazyflie &cflieCopter, bool button, bool& alt_hold, float thrust, float min_thrust, float max_thrust, float& old_thrust)
{
	if (button == alt_hold)
		return;
	cflieCopter.setParameterValue("flightmode.althold", (uint8_t)button);
	alt_hold = button;
	set_thrust(cflieCopter, thrust, alt_hold, min_thrust, max_thrust, old_thrust);
}

static void show_temperature(CCrazyflie &cflieCopter)
{
	static float temperature(0);
	float t(cflieCopter.temperature());

	if (abs(temperature - t) > 1) {
		temperature = t;
		std::cout << "Temperature: " << temperature << std::endl;
	}
}

static void show_battery_level(CCrazyflie &cflieCopter)
{
	static float batteryLevel(0);
	float l(cflieCopter.batteryLevel());

	if (abs(batteryLevel - l) > 0.1) {
		batteryLevel = l;
		std::cout << "Battery: " << batteryLevel << std::endl;
	}
}

static void show_pressure(CCrazyflie &cflieCopter)
{
	static float pressure(0);
	float p(cflieCopter.pressure());

	if (abs(pressure - p) > 0.1) {
		pressure = p;
		std::cout << "Pressure: " << pressure << std::endl;
	}
}

static void show_firmware_revision(CCrazyflie &cflieCopter)
{
	uint32_t fw_rev0;
	uint16_t fw_rev1;
	uint8_t fw_dirty;
	cflieCopter.getParameterValue("firmware.revision0", fw_rev0);
	cflieCopter.getParameterValue("firmware.revision1", fw_rev1);
	cflieCopter.getParameterValue("firmware.modified", fw_dirty);
	uint64_t fw_rev(fw_rev0);
	fw_rev <<= 16;
	fw_rev += fw_rev1;
	std::cout << "Crazyflie firmware revision " << std::hex << fw_rev << std::dec;
	if (fw_dirty)
		std::cout << "-dirty";
	std::cout << std::endl;
}


int main(int, char **)
{
	std::cout << std::endl <<
		"Crazyflie headless client" << std::endl <<
		"(C) 2014 Alexander Holler" << std::endl << std::endl <<
		"Version " VERSION << std::endl << std::endl <<
		"Uses a heavily modified libcflie" << std::endl <<
		"(C) 2013 Jan Winkler" << std::endl << std::endl;

	old_signal_handler = std::signal(SIGINT, deleter_sig_handler); // catch CTRL-C
	if (old_signal_handler ==  SIG_ERR)
		old_signal_handler = nullptr;

	Settings settings;

	// Get the path of the executable, this is Linux specific!
	std::string path;
	{
	char buf[PATH_MAX];
	ssize_t len = ::readlink("/proc/self/exe", buf, sizeof(buf));
	if (len == -1) {
		std::cerr << "Can't read my path from /proc/self/exe!" << std::endl;
		exit(1);
	}
	path = std::string(buf, len);
	}
	path = path.substr(0, path.rfind('/')) + "/conf";
	std::cout << "Configuration path: '" << path << '\'' << std::endl;

	settings.load(path);
	settings.print();
	load_input(path, settings.device_config_mapping);

	Joystick joystick;
	joystick.open();
	settings.input_device = joystick.get_name();

	CCrazyRadio crRadio(settings.link_uri);

	if (!crRadio.startRadio()) {
		std::cerr << "Could not connect to dongle. Did you plug it in?" << std::endl;
		return 1;
	}

	CCrazyflie cflieCopter(&crRadio);
	cflieCopter.setSendSetpoints(true);

	uint16_t min_thrust_raw(p2t(settings.min_thrust));
	uint16_t max_thrust_raw(p2t(settings.max_thrust));
	float old_thrust(0);
	bool alt_hold(false);

	enum {
		d_roll,
		d_pitch,
		d_thrust,
		d_yaw,
		d_maxdata,
	};
	float data[d_maxdata] = {0, 0, 0, 0};

	// Wait until initialized
	while (!interrupted && cflieCopter.cycle() && !cflieCopter.isInitialized())
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	std::cout << "Initialized" << std::endl;

	show_firmware_revision(cflieCopter);

	bool has_pressure_sensor(false);
	try {
		uint8_t imu_sensors_HMC5883L;
		cflieCopter.getParameterValue("imu_sensors.HMC5883L", imu_sensors_HMC5883L);
		if (imu_sensors_HMC5883L)
			has_pressure_sensor = true;
	} catch(...) {
	}
	std::cout << "Pressure sensor available " << has_pressure_sensor << std::endl;
	// Start Logging
	cflieCopter.enableBatteryLogging();
	if (has_pressure_sensor)
		cflieCopter.enableBarometerLogging();

	// Start the main loop
	while (!interrupted && cflieCopter.cycle()) {
		std::this_thread::sleep_for(std::chrono::milliseconds(5));

		show_battery_level(cflieCopter);
		if (has_pressure_sensor) {
			show_temperature(cflieCopter);
			show_pressure(cflieCopter);
		}

		js_event evt;
		if (!joystick.try_get_event(evt))
			continue;
		if ((evt.type & ~JS_EVENT_INIT) == JS_EVENT_BUTTON) {
			auto i = joystick_button_settings.find(Joystick_button_setting(evt.number));
			if (i == joystick_button_settings.end())
				continue;
			if (!evt.value) {
				// Button up
				if (i->function == Button_alt_hold && has_pressure_sensor) {
					std::cout << "Stop hovering" << std::endl;
					set_althold(cflieCopter, evt.value, alt_hold, data[d_thrust], min_thrust_raw, max_thrust_raw, old_thrust);
				}
				continue;
			}
			// Button down
			if (i->function == Button_exit) {
				std::cout << "Exit" << std::endl;
				break;
			}
			else if (i->function == Button_alt_hold && has_pressure_sensor) {
				std::cout << "Hovering" << std::endl;
				set_althold(cflieCopter, evt.value, alt_hold, data[d_thrust], min_thrust_raw, max_thrust_raw, old_thrust);
			}
			else if (i->function == Button_trim_pitch) {
				settings.trim_pitch += i->scale;
				std::cout << "trim_pitch " << settings.trim_pitch << std::endl;
			}
			else if (i->function == Button_trim_roll) {
				settings.trim_roll += i->scale;
				std::cout << "trim_roll " << settings.trim_roll << std::endl;
			}
			continue;
		}
		if ((evt.type & ~JS_EVENT_INIT) != JS_EVENT_AXIS)
			continue;
		auto i = joystick_axis_settings.find(Joystick_axis_setting(evt.number));
		if (i == joystick_axis_settings.end())
			continue;
		if (i->function == Axis_yaw) {
			// yaw, scale = 1.0
			float yaw = (float)evt.value / std::numeric_limits<int16_t>::max(); // convert to [-1, 1]
			yaw *= i->scale;
			if (yaw == data[d_yaw])
				continue;
			data[d_yaw] = yaw;
			set_yaw(cflieCopter, yaw, settings.max_yaw);
			continue;
		}
		else if (i->function == Axis_thrust) {
			float thrust = (float)evt.value / std::numeric_limits<int16_t>::max(); // convert to [-1, 1]
			thrust *= i->scale;
			if (thrust == data[d_thrust])
				continue;
			data[d_thrust] = thrust;
			set_thrust(cflieCopter, thrust, alt_hold, min_thrust_raw, max_thrust_raw, old_thrust);
			continue;
		}
		else if (i->function == Axis_roll) {
			float roll = (float)evt.value / std::numeric_limits<int16_t>::max(); // convert to [-1, 1]
			roll *= i->scale;
			if (roll == data[d_roll])
				continue;
			data[d_roll] = roll;
			float pitch = data[d_pitch];
			set_roll(cflieCopter, pitch, roll, settings.max_rp_angle, settings.trim_pitch, settings.trim_roll, settings.x_mode);
			continue;
		}
		else if (i->function == Axis_pitch) {
			float pitch = (float)evt.value / std::numeric_limits<int16_t>::max(); // convert to [-1, 1]
			pitch *= i->scale;
			if (pitch == data[d_pitch])
				continue;
			data[d_pitch] = pitch;
			float roll = data[d_roll];
			set_pitch(cflieCopter, pitch, roll, settings.max_rp_angle, settings.trim_pitch, settings.trim_roll, settings.x_mode);
			continue;
		}
	} // while

	// Stop logging
	if (has_pressure_sensor)
		cflieCopter.disableBarometerLogging();
	cflieCopter.disableBatteryLogging();

	settings.save(path);

	return 0;
} // main
