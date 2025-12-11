#include "ProtoTMRSerial.h"
#include <mc_rtc/io_utils.h>
#include <mc_rtc/logging.h>
#include <fcntl.h>
#include <iostream>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
namespace io
{

#define BUFFER_SIZE 256

static unsigned char buffer[BUFFER_SIZE];
bool packet_started = 0;
char packet_bytes[BUFFER_SIZE];
int packet_bytes_index = 0;
// Packet received_packet;
// SensorData sensors_data_decoded;

int baud_to_int(speed_t baud)
{
  switch(baud)
  {
    case B0:
      return 0;
    case B50:
      return 50;
    case B75:
      return 75;
    case B110:
      return 110;
    case B134:
      return 134;
    case B150:
      return 150;
    case B200:
      return 200;
    case B300:
      return 300;
    case B600:
      return 600;
    case B1200:
      return 1200;
    case B1800:
      return 1800;
    case B2400:
      return 2400;
    case B4800:
      return 4800;
    case B9600:
      return 9600;
    case B19200:
      return 19200;
    case B38400:
      return 38400;
    case B57600:
      return 57600;
    case B115200:
      return 115200;
    case B230400:
      return 230400;
    case B460800:
      return 460800;
    case B500000:
      return 500000;
    case B576000:
      return 576000;
    case B921600:
      return 921600;
    case B1000000:
      return 1000000;
    case B1152000:
      return 1152000;
    case B1500000:
      return 1500000;
    case B2000000:
      return 2000000;
    case B2500000:
      return 2500000;
    case B3000000:
      return 3000000;
    case B3500000:
      return 3500000;
    case B4000000:
      return 4000000;
    default:
      return -1; // Unknown baud rate
  }
}
speed_t int_to_baud(int baud)
{
  switch(baud)
  {
    case 0:
      return B0;
    case 50:
      return B50;
    case 75:
      return B75;
    case 110:
      return B110;
    case 134:
      return B134;
    case 150:
      return B150;
    case 200:
      return B200;
    case 300:
      return B300;
    case 600:
      return B600;
    case 1200:
      return B1200;
    case 1800:
      return B1800;
    case 2400:
      return B2400;
    case 4800:
      return B4800;
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 500000:
      return B500000;
    case 576000:
      return B576000;
    case 921600:
      return B921600;
    case 1000000:
      return B1000000;
    case 1152000:
      return B1152000;
    case 1500000:
      return B1500000;
    case 2000000:
      return B2000000;
    case 2500000:
      return B2500000;
    case 3000000:
      return B3000000;
    case 3500000:
      return B3500000;
    case 4000000:
      return B4000000;
    default:
      return (speed_t)-1; // Unknown baud rate
  }
}
ProtoTMRSerial::ProtoTMRSerial()
  {
    this->SENSOR_COUNT = 23;
    avg_buffer.resize(SENSOR_COUNT, 0);
    last_received_data.resize(SENSOR_COUNT, 0);
  }

ProtoTMRSerial::~ProtoTMRSerial()
{
  close_serial_port();
}

void ProtoTMRSerial::open_serial_port(const std::string & portName, const int baudRate)
{
  serialPort = open(portName.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if(serialPort < 0)
  {
    throw std::runtime_error(fmt::format("[ProtoTMRSerial] Failed to open serial port {}", portName));
  }
  // Configure the serial port
  struct termios tty;
  if(tcgetattr(serialPort, &tty) != 0)
  {
    throw std::runtime_error(fmt::format("[ProtoTMRSerial] Error getting terminal attributes"));
    close(serialPort);
  }

  // Set baud rate to 9600
  cfsetispeed(&tty, int_to_baud(baudRate));
  cfsetospeed(&tty, int_to_baud(baudRate));

  // Configure 8N1 (8 data bits, no parity, 1 stop bit)
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit characters
  tty.c_iflag &= ~IGNBRK; // Disable break processing
  tty.c_lflag = 0; // No canonical mode, no echo
  tty.c_oflag = 0; // No remapping, no delays
  tty.c_cc[VMIN] = 1; // Read at least 1 character
  tty.c_cc[VTIME] = 1; // Timeout after 0.1 seconds

  // Apply the settings to the serial port
  if(tcsetattr(serialPort, TCSANOW, &tty) != 0)
  {
    throw std::runtime_error(fmt::format("[ProtoTMRSerial] Error setting terminal attributes!"));
    close(serialPort);
  }
  isConnected = true;
}

void ProtoTMRSerial::close_serial_port()
{
  close(serialPort);
  isConnected = false;
}

bool ProtoTMRSerial::connected()
{
  if(fcntl(serialPort, F_GETFL) < 0)
  {
    isConnected = false;
  }
  /*
  int status;

  if(ioctl(serialPort, TIOCMGET, &status) == -1)
  {
    // Error in getting port status
    mc_rtc::log::error("[ProtoTMRSerial] Error getting serial port status !");
    return false;
  }
  //mc_rtc::log::error("[ProtoTMRSerial] Serial port status : {}",
                     //(status & TIOCM_CAR) != 0 ? "Connected" : "Disconnected");
  // Check the Data Carrier Detect (DCD) signal
  return (status & TIOCM_CAR) != 0;*/

  // bool status = serialPort.is_open() && serialPort.good();
  // return true;
  return isConnected;
}

void ProtoTMRSerial::read_serial_port()
{
  mc_rtc::log::info("[ProtoTMRSerial] Serial reading started");

  std::string line_buffer;
  char read_buf[BUFFER_SIZE];

  while(true)
  {
    int BytesRead = read(serialPort, read_buf, sizeof(read_buf));
    if(BytesRead > 0)
    {
      for(int i = 0; i < BytesRead; ++i)
      {
        char c = read_buf[i];
        if(c == '\n')
        {
          // Optionally remove carriage return if present
          if(!line_buffer.empty() && line_buffer.back() == '\r')
          {
            line_buffer.pop_back();
          }
          // Copy line_buffer to buffer and call parse_buffer
          size_t line_len = std::min(line_buffer.size(), sizeof(buffer));
          memcpy(buffer, line_buffer.data(), line_len);
          parse_buffer(buffer, line_len);
          line_buffer.clear();
        }
        else
        {
          line_buffer += c;
        }
      }
      cycles_waited = 0;
      serial_status = true;
    }
    else
    {
      if(++cycles_waited > cycles_timeout) serial_status = false;
    }
  }
}

void ProtoTMRSerial::apply_filter(RawData & raw_data)
{
  // filtered_current = round(coef * raw_current + (1 - coef) * filtered_previous)
  size_t index = 0;
  for(auto & value : raw_data)
  {
    value = round(alphaFilter_ * value + (1 - alphaFilter_) * last_received_data[index]);
    index++;
  }
}

bool ProtoTMRSerial::validate_data(Data & raw_data)
{
  for(auto & value : raw_data)
  {
    if(value > 4100)
    {
      mc_rtc::log::warning("[ProtoTMRSerial] Invalid read (read {}, max {})", value, 4100);
      return false;
    }
  }
  return true;
}

void ProtoTMRSerial::parse_buffer(unsigned char * buff, size_t buff_len)
{
  if(buff_len == 0)
  {
    // std::cout << "Empty buffer received, start reading new sensors" << std::endl;
    return;
  }

  std::string line(reinterpret_cast<char*>(buff), buff_len);

  std::vector<uint16_t> numbers;
  std::stringstream ss(line);
  std::string item;

  while(std::getline(ss, item, ','))
  {
    if(item.empty()) continue;
    try
    {
      uint16_t value = static_cast<uint16_t>(std::stoi(item));
      numbers.push_back(value);
    }
    catch(const std::exception &)
    {
      // Ignore parse errors
    }
  }

  if(numbers.size() < 2)
  {
    // std::cout << "Not enough sensor data in buffer" << std::endl;
    return;
  }

  uint16_t sensor_index = numbers[0];
  if(sensor_index >= avg_buffer.size())
  {
    // std::cout << "Sensor index out of range: " << sensor_index << std::endl;
    return;
  }

  uint32_t sum = 0;
  for(size_t i = 1; i < numbers.size(); ++i)
  {
    sum += numbers[i];
  }
  uint16_t avg = static_cast<uint16_t>(sum / (numbers.size() - 1));
  avg_buffer[sensor_index] = avg;

  // std::cout << "Sensor index: " << sensor_index << ", Average: " << avg << std::endl;

  // Only lock and copy when the last sensor is processed
  if(sensor_index == SENSOR_COUNT - 1)
  {
    std::lock_guard<std::mutex> lock(last_received_data_mutex);
    last_received_data = avg_buffer;
    // std::cout << "All sensor averages updated." << std::endl;
  }
}

} // namespace io
