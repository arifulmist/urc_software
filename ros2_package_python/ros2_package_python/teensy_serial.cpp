#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <string>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdexcept>
#include <sstream>

namespace py = pybind11;

class TeensyComms {
public:
    TeensyComms() : fd_(-1) {}

    ~TeensyComms() {
        if (fd_ != -1) close(fd_);
    }

    void setup(const std::string &serial_device, int baud_rate, int timeout_ms) {
        fd_ = open(serial_device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0) throw std::runtime_error("Failed to open port");

        struct termios tty;
        if (tcgetattr(fd_, &tty) != 0) throw std::runtime_error("tcgetattr failed");

        speed_t speed;
        switch (baud_rate) {
            case 9600: speed = B9600; break;
            case 19200: speed = B19200; break;
            case 38400: speed = B38400; break;
            case 57600: speed = B57600; break;
            case 115200: speed = B115200; break;
            default: throw std::runtime_error("Unsupported baudrate");
        }

        cfsetospeed(&tty, speed);
        cfsetispeed(&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag &= ~IGNBRK;
        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = timeout_ms / 100; // VTIME is in 0.1s units

        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr(fd_, TCSANOW, &tty) != 0) throw std::runtime_error("tcsetattr failed");
    }

    void send_empty_msg() {
        send_msg("\r");
    }

    std::pair<int, int> read_encoder_values() {
        std::string response = send_msg("e\r");
        std::string delimiter = " ";
        size_t del_pos = response.find(delimiter);
        if (del_pos == std::string::npos) throw std::runtime_error("Invalid encoder response");
        std::string token_1 = response.substr(0, del_pos);
        std::string token_2 = response.substr(del_pos + delimiter.length());
        int val_1 = std::atoi(token_1.c_str());
        int val_2 = std::atoi(token_2.c_str());
        return {val_1, val_2};
    }

    void set_motor_values(int val_1, int val_2) {
        std::stringstream ss;
        ss << "m " << val_1 << " " << val_2 << "\r";
        send_msg(ss.str(), false);
    }

    void set_pid_values(float k_p, float k_d, float k_i, float k_o) {
        std::stringstream ss;
        ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
        send_msg(ss.str());
    }

    std::string send_msg(const std::string &msg_to_send, bool print_output = true) {
        if (fd_ == -1) throw std::runtime_error("Serial port not open");
        write(fd_, msg_to_send.c_str(), msg_to_send.size());
        char buf[256] = {0};
        int n = read(fd_, buf, sizeof(buf) - 1);
        std::string response = (n > 0) ? std::string(buf, n) : "";
        if (print_output) {
            // Optionally print or log here
        }
        return response;
    }

private:
    int fd_;
};

PYBIND11_MODULE(teensy_serial_backend, m) {
    py::class_<TeensyComms>(m, "TeensyComms")
        .def(py::init<>())
        .def("setup", &TeensyComms::setup, py::arg("serial_device"), py::arg("baud_rate"), py::arg("timeout_ms"))
        .def("send_empty_msg", &TeensyComms::send_empty_msg)
        .def("read_encoder_values", &TeensyComms::read_encoder_values)
        .def("set_motor_values", &TeensyComms::set_motor_values)
        .def("set_pid_values", &TeensyComms::set_pid_values)
        .def("send_msg", &TeensyComms::send_msg, py::arg("msg_to_send"), py::arg("print_output") = true);
}