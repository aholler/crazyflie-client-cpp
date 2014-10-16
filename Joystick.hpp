//
// C++ Linux joystick API
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

#ifndef JOYSTICK_HPP
#define JOYSTICK_HPP

#include <fcntl.h> // open
#include <unistd.h> // close
#include <sys/ioctl.h>
#include <linux/joystick.h>

#include <cstring> // std::memset
#include <system_error>
#include <string>

class Joystick {
	public:
		Joystick()
			: axis_(0)
			, buttons_(0)
			, file_(0)
		{}
		~Joystick() {
			close();
		}
		void open(const std::string& devnode) {
			file_ = ::open(devnode.c_str(), O_RDONLY);
			if (file_ <= 0) {
				file_ = 0;
				throw std::system_error(std::error_code(errno, std::system_category()),
					"Can't open joystick device");
			}
			if (::ioctl(file_, JSIOCGAXES, &axis_) < 0)
				throw std::system_error(std::error_code(errno, std::system_category()),
					"joystick: ioctl error");
			if (::ioctl(file_, JSIOCGBUTTONS, &buttons_) < 0)
				throw std::system_error(std::error_code(errno, std::system_category()),
					"joystick: ioctl error");
			char buf[80];
			std::memset(buf, 0, sizeof(buf));
			if (::ioctl(file_, JSIOCGNAME(sizeof(buf)), buf) < 0)
				throw std::system_error(std::error_code(errno, std::system_category()),
					"joystick: ioctl error");
			buf[sizeof(buf)-1] = 0;
			name_ = buf;
			::fcntl(file_, F_SETFL, O_NONBLOCK); // use non-blocking mode
		}
		void open(void) {
			open("/dev/input/js0");
		}
		void close(void) {
			if (file_)
				::close(file_);
			file_ = 0;
		}
		bool try_get_event(js_event& evt) {
			int rc = ::read(file_, &evt, sizeof(evt));
			if (rc < 0) {
				if (errno == EWOULDBLOCK)
					return false;
				throw std::system_error(std::error_code(errno,
					std::system_category()), "joystick: read error");
			}
			return (rc == sizeof(evt));
		}
		std::string get_name(void) const {
			return name_;
		}
		unsigned get_axis(void) const {
			return axis_;
		}
		unsigned get_buttons(void) const {
			return buttons_;
		}

	private:
		std::string name_;
		unsigned axis_;
		unsigned buttons_;
		int file_;
};

#endif // JOYSTICK_HPP
