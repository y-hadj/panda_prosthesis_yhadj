#pragma once

#include <array>
#include <mutex>
#include <vector>
#include "Serial.h"

namespace io
{
struct ProtoTMRSerial : public Serial
{
  ProtoTMRSerial();
  ~ProtoTMRSerial();

  void open_serial_port(const std::string & portName, const int baudRate) override;
  void close_serial_port() override;
  bool connected() override;
  void read_serial_port() override;

  inline double alphaFilter() const noexcept
  {
    return alphaFilter_;
  }
  void alphaFilter(double a)
  {
    alphaFilter_ = a;
  }

protected:
  void parse_buffer(unsigned char * buff, size_t buff_len) override;
  void apply_filter(RawData & raw_data) override;
  bool validate_data(Data & data) override;

  Data avg_buffer;
};
} // namespace io
