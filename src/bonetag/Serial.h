#pragma once

#include <mc_rtc/clock.h>
#include <mutex>
#include <string>
#include <termios.h>
#include <thread>
#include <vector>
#include <atomic>

namespace io
{

inline int baud_to_int(speed_t baud)
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
inline speed_t int_to_baud(int baud)
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

template<typename DataT>
struct TimedData
{
  using SensorDataT = typename DataT::value_type;
  TimedData() {}
  TimedData(std::chrono::time_point<mc_rtc::clock> initial_time,
            unsigned int sensor_count,
            unsigned int meaurementsPerSensor)
  : timestamp_ms(0.0), initial_time(initial_time), data{}
  {
    data.resize(sensor_count);
    for(auto & sensor_data : data)
    {
      sensor_data.resize(meaurementsPerSensor);
    }
  }

  void setCurrentTime()
  {
    auto now = mc_rtc::clock::now();
    timestamp_ms = std::chrono::duration_cast<mc_rtc::duration_ms>(now - initial_time);
  }

  TimedData(DataT && data)
  {
    setCurrentTime();
    this->data = std::move(data);
  }

  mc_rtc::duration_ms timestamp_ms{0.0};
  std::chrono::time_point<mc_rtc::clock> initial_time;
  DataT data;
};

struct Serial
{
  size_t SENSOR_COUNT = 32;
  std::chrono::time_point<mc_rtc::clock> start_time_;

  using Data = std::vector<uint16_t>;
  using RawData = std::vector<std::vector<uint16_t>>;

  using TimedFilteredData = TimedData<Data>;
  using TimedRawData = TimedData<RawData>;

  std::string portName = "";
  const int baudRate = 115200;

  Serial(const std::string & portName, const int baudRate, size_t sensor_count, size_t meaurementsPerSensor)
  : SENSOR_COUNT(sensor_count), start_time_(mc_rtc::clock::now()),
    rawDataBuffer_(start_time_, sensor_count, meaurementsPerSensor),
    currentRawData_(start_time_, sensor_count, meaurementsPerSensor), portName(portName), baudRate(baudRate)
  {
    last_received_data.resize(SENSOR_COUNT, 0);

    thread_ = std::thread(&Serial::runThread, this);
  };
  virtual ~Serial() = 0;

  void runThread()
  {
    open_serial_port();

    const auto period = std::chrono::milliseconds(1); // 1000Hz = 1ms period

    while(true)
    {
      auto loop_start = std::chrono::steady_clock::now();

      read_serial_port();

      auto loop_end = std::chrono::steady_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(loop_end - loop_start);

      if(elapsed < period)
      {
        std::this_thread::sleep_for(period - elapsed);
      }
    }

    close_serial_port();
  }

  virtual void open_serial_port() = 0;
  virtual void close_serial_port() = 0;
  virtual bool connected() = 0;
  virtual void read_serial_port() = 0;
  bool isConnected = false;

  // processed in a thread, do not use outside

  // all received raw data since we last read it with popRawData()
  mutable std::mutex raw_data_mutex;
  TimedRawData rawDataBuffer_;
  TimedRawData currentRawData_;
  std::atomic<bool> rawDataUpdated_{false};

  bool lastRawDataUpdated()
  {
    return rawDataUpdated_;
  }

  TimedRawData readLastRawData()
  {
    rawDataUpdated_ = false;
    std::lock_guard<std::mutex> lock(raw_data_mutex);
    return currentRawData_;
  }

  TimedRawData lastRawData() const
  {
    std::lock_guard<std::mutex> lock(raw_data_mutex);
    return currentRawData_;
  }

  std::vector<TimedRawData> rawData_;
  std::vector<TimedRawData> popRawData()
  {
    std::lock_guard<std::mutex> lock(raw_data_mutex);
    return rawData_;
  }

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

  std::thread thread_;

protected:
  int serialPort;
  double alphaFilter_ = 0.3;
};
} // namespace io
