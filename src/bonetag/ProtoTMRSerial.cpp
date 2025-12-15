#include "ProtoTMRSerial.h"
#include <mc_rtc/io_utils.h>
#include <mc_rtc/logging.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
namespace io
{

#define BUFFER_SIZE 256
static unsigned char buffer[BUFFER_SIZE];

ProtoTMRSerial::ProtoTMRSerial(const std::string & portName, const int baudRate) : Serial(portName, baudRate, 23, 10)
{
  avg_buffer.resize(SENSOR_COUNT, 0);
}

ProtoTMRSerial::~ProtoTMRSerial()
{
  close_serial_port();
}

void ProtoTMRSerial::open_serial_port()
{
  serialPort = open(portName.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if(serialPort < 0)
  {
    throw std::runtime_error(fmt::format("[ProtoTMRSerial] Failed to open serial port \"{}\"", portName));
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
  // mc_rtc::log::info("read serial port called at t={}",
  // std::chrono::duration_cast<mc_rtc::duration_ms>(mc_rtc::clock::now().time_since_epoch()).count());
  char read_buf[512];

  int bytes_available = 0;
  if(ioctl(serialPort, FIONREAD, &bytes_available) == -1)
  {
    mc_rtc::log::error("ioctl FIONREAD failed");
    return;
  }
  if(bytes_available < 512) return;

  int BytesRead = read(serialPort, read_buf, sizeof(read_buf));
  if(BytesRead > 0)
  {
    line_buffer.append(read_buf, BytesRead);
    // mc_rtc::log::info("read {} bytes, line_buffer: {}", BytesRead, line_buffer);
    size_t pos = 0;
    while((pos = line_buffer.find('\n')) != std::string::npos)
    {
      std::string line = line_buffer.substr(0, pos);
      // mc_rtc::log::info("read line: {}", line);
      // Optionally remove carriage return if present
      if(!line.empty() && line.back() == '\r')
      {
        line.pop_back();
      }
      size_t line_len = std::min(line.size(), sizeof(buffer));
      memcpy(buffer, line.data(), line_len);
      parse_buffer(buffer, line_len);
      line_buffer.erase(0, pos + 1); // Remove parsed line (including '\n')
    }
    cycles_waited = 0;
    serial_status = true;
  }
  else
  {
    if(++cycles_waited > cycles_timeout) serial_status = false;
  }
}

void ProtoTMRSerial::apply_filter(RawData & raw_data)
{
  // filtered_current = round(coef * raw_current + (1 - coef) * filtered_previous)
  // size_t index = 0;
  // for(auto & value : raw_data)
  // {
  //   value = round(alphaFilter_ * value + (1 - alphaFilter_) * last_received_data[index]);
  //   index++;
  // }
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

  std::string line(reinterpret_cast<char *>(buff), buff_len);

  // mc_rtc::log::info("t={}, parse buffer got line: {}",
  // std::chrono::duration_cast<mc_rtc::duration_ms>(mc_rtc::clock::now().time_since_epoch()).count(), line);

  // N+1 values for a sensor : sensor id, value1, value2, ...
  auto numbers = std::vector<uint16_t>{};
  std::stringstream ss(line);
  std::string item;

  rawDataBuffer_.setCurrentTime();

  while(std::getline(ss, item, ','))
  {
    if(item.empty()) continue;
    try
    {
      numbers.emplace_back(static_cast<uint16_t>(std::stoi(item)));
    }
    catch(const std::exception &)
    {
      // Ignore parse errors
    }
  }

  if(numbers.size() < 11)
  {
    mc_rtc::log::warning("wrong sensor size, got {} expected 11", numbers.size());
    return;
  }
  else
  {
    // mc_rtc::log::info("got {}", mc_rtc::io::to_string(numbers));
  }

  auto sensorId = numbers[0];
  std::copy(numbers.begin() + 1, numbers.end(), rawDataBuffer_.data[sensorId].begin());

  uint32_t sum = 0;
  for(size_t i = 1; i < numbers.size(); ++i)
  {
    sum += numbers[i];
  }
  uint16_t avg = static_cast<uint16_t>(sum / (numbers.size() - 1));
  avg_buffer[sensorId] = avg;

  // std::cout << "Sensor index: " << sensor_index << ", Average: " << avg << std::endl;

  // Only lock and copy when the last sensor is processed
  if(sensorId == SENSOR_COUNT - 1)
  {
    {
      std::lock_guard<std::mutex> lock{raw_data_mutex};
      currentRawData_ = rawDataBuffer_;
      rawDataUpdated_ = true;
    }
    {
      std::lock_guard<std::mutex> lock(last_received_data_mutex);
      last_received_data = avg_buffer;
    }
    // std::cout << "All sensor averages updated at t=" << rawDataBuffer_.timestamp_ms.count() / 1000. << "s" <<
    // std::endl;
  }
}

} // namespace io
