// Using modified code from joshnewans articubot-one
#ifndef TB3_ARDUINO_ARDUINO_COMMS_HPP
#define TB3_ARDUINO_ARDUINO_COMMS_HPP

#include <sstream>
#include <libserial/SerialPort.h>
#include <iostream>

LibSerial::BaudRate convert_baud_rate(int baud_rate) {
  // Just handle some common baud rates
  switch (baud_rate) {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;

    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 115200" << std::endl;
      return LibSerial::BaudRate::BAUD_115200;
  }
}

class ArduinoComms {

public:
  ArduinoComms() = default;

  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms) {  
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
  }

  void disconnect() {
    serial_conn_.Close();
  }

  bool connected() {
    return serial_conn_.IsOpen();
  }


  std::string send_msg(const std::string &msg_to_send, bool print_output = false) {
    serial_conn_.FlushIOBuffers(); // Just in case
    serial_conn_.Write(msg_to_send);

    std::string response = "";
    try {
      // Responses end with \r\n\n so we will read up to (and including) the \n.
      serial_conn_.ReadLine(response, '\n', timeout_ms_);
    }
    catch(const std::exception& err) {
      std::cerr << "Communication failed. Retrying..." << std::endl ;
    }
    /* catch (const LibSerial::ReadTimeout&) {
      std::cerr << "The ReadByte() call has timed out." << std::endl;
    } */

    if (print_output) {
      std::cout << "Sent: " << msg_to_send << " Recv: " << response << std::endl;
    }

    return response;
  }


  void send_empty_msg() {
    std::string response = send_msg("\r\n");
  }

  void read_encoder_values(int &val_1, int &val_2) {
    std::string response = send_msg("e\r\n");

    std::string delimiter = " ";
    size_t del_pos = response.find(delimiter);
    std::string token_1 = response.substr(0, del_pos);
    std::string token_2 = response.substr(del_pos + delimiter.length());

    val_1 = std::atoi(token_1.c_str());
    val_2 = std::atoi(token_2.c_str());
  }

  void set_motor_values(int val_1, int val_2) {
    std::stringstream ss;
    ss << "c " << val_1 << " " << val_2 << "\r\n";
    send_msg(ss.str());
  }

  void set_pid_values(int k_p, int k_i, int k_d) {
    std::stringstream ss;
    ss << "p " << k_p << " " << k_i << " " << k_d << "\r\n";
    send_msg(ss.str());
  }

  void reset_encoder_counts() {
    send_msg("r\r\n");
  }

private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;
};

#endif // TB3_ARDUINO_ARDUINO_COMMS_HPP