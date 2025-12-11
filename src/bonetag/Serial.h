#pragma once

#include <mutex>
#include <vector>
#include <string>

namespace io
{
struct Serial
{
  size_t SENSOR_COUNT = 32;
  using Data = std::vector<uint16_t>;
  using RawData = std::vector<uint16_t>;

  Serial() {};
  virtual ~Serial() = 0;

  virtual void open_serial_port(const std::string & portName, const int baudRate) = 0;
  virtual void close_serial_port() = 0;
  virtual bool connected() = 0;
  virtual void read_serial_port() = 0;
  bool isConnected = false;

  std::mutex received_data_mutex;
  std::vector<Data> received_data;

  mutable std::mutex last_received_data_mutex;
  io::Serial::Data last_received_data; // used for filtering

  Data lastReceivedData() const
  {
    std::lock_guard<std::mutex> lock(last_received_data_mutex);
    return last_received_data;
  }

  int cycles_waited = 0;
  int cycles_timeout = 5000;
  bool serial_status = false;

  bool debug_bytes = true;
  bool debug_raw = true;
  bool debug_results = true;

  inline double alphaFilter() const noexcept
  {
    return alphaFilter_;
  }
  void alphaFilter(double a)
  {
    alphaFilter_ = a;
  }

protected:
  virtual void parse_buffer(unsigned char * buff, size_t buff_len) = 0;
  virtual void apply_filter(RawData & raw_data) = 0;
  virtual bool validate_data(Data & data) = 0;

protected:
  int serialPort;
  double alphaFilter_ = 0.3;
};
} // namespace io
