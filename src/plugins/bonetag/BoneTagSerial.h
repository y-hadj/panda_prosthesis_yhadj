#ifndef BONETAGSERIAL_H
#define BONETAGSERIAL_H
#include "Packet.hpp"
#include <array>
#include <mutex>
#include <vector>
#define SENSOR_COUNT 8
namespace io
{
struct BoneTagSerial
{
  using Data = std::array<uint16_t, SENSOR_COUNT>;
  using RawData = std::array<uint16_t, SENSOR_COUNT>;

  BoneTagSerial();
  ~BoneTagSerial();

  void open_serial_port(const std::string & portName, const int baudRate);
  void close_serial_port();
  bool connected();
  void read_serial_port();
  bool isConnected = false;
  bool gotFullFrame_ = false;

  /**
   * \brief Request a new frame. You must wait until gotFullFrame() = true to have the new full sensor frame
   */
  void requestNewFrame()
  {
    gotFullFrame_ = false;
  }

  /**
   * \bried True after a full frame has been received:
   * - if you called requestNewFrame(): this is true when the next full frame has been received
   * - otherwise it always true after the first full sensor frame has been received
   **/
  inline bool gotFullFrame() const noexcept
  {
    return gotFullFrame_;
  }

  /**
   * \brief Get the last full sensor frame that has been received.
   * Call gotFullFrame() beforehand to check if the frame has changed since the last call to this function.
   */
  RawData getLastFrame() const
  {
    std::lock_guard<std::mutex> lock(received_data_mutex);
    return lastSensorFrame_;
  }

  mutable std::mutex received_data_mutex;
  std::vector<Data> received_data;
  Data lastSensorFrame_;

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
  void parse_buffer(unsigned char * buff, size_t buff_len);
  void apply_filter(std::array<uint16_t, SENSOR_COUNT> & raw_data);
  bool validate_data(std::array<uint16_t, SENSOR_COUNT> & data);

protected:
  int serialPort;
  double alphaFilter_ = 0.3;
};
} // namespace io

#endif /* BONETAGSERIAL_H */
