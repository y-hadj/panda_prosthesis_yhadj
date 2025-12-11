#include "BoneTagSerialPlugin.h"
#include "ProtoTMRSerial.h"

#include <mc_control/GlobalPluginMacros.h>
#include <optional>

namespace mc_plugin
{

constexpr auto DEFAULT_RATE = 200; // [Hz]

BoneTagSerialPlugin::BoneTagSerialPlugin()
{
  serial_.reset(new io::ProtoTMRSerial{});
}

BoneTagSerialPlugin::~BoneTagSerialPlugin()
{
  mc_rtc::log::info("[BoneTagSerialPlugin] Stopping communication thread");
  running_ = false;
  thread_.join();
  mc_rtc::log::info("[BoneTagSerialPlugin] Communication thread stopped");
}

void BoneTagSerialPlugin::connect()
{
  serial_->close_serial_port();
  try
  {
    serial_->open_serial_port(serial_port_name, serial_port_baud_rate);
    mc_rtc::log::success("[BoneTagSerialPlugin] Communication with device {} opened", serial_port_name);
  }
  catch(std::runtime_error & e)
  {
    mc_rtc::log::warning(e.what());
  }
}

void BoneTagSerialPlugin::connectAndStartReading()
{
  connect();
  try
  {
    mc_rtc::log::success("[BoneTagSerialPlugin] Starting serial port reading ...");
    serial_->read_serial_port();
    mc_rtc::log::success("[BoneTagSerialPlugin] Serial port reading started !");
  }
  catch(std::runtime_error & e)
  {
    mc_rtc::log::error("[BoneTagSerialPlugin] Failed to read data");
    hasReceivedData_ = false;
  }
  connect_requested_ = false;
}
void BoneTagSerialPlugin::init(mc_control::MCGlobalController & gc, const mc_rtc::Configuration & config)
{
  auto & ctl = gc.controller();
  data_.assign(serial_->SENSOR_COUNT, 0);
  lastData_ = data_;

  config("verbose", verbose_);
  double rate = config("rate", DEFAULT_RATE);
  rate_ = (1. / rate) / ctl.timeStep;

  serial_port_name = config("serial_port_name", std::string{"/dev/ttyUSB0"});
  serial_port_baud_rate = config("serial_port_baud_rate", 9600);

  thread_ = std::thread(
      [this]()
      {
        mc_rtc::log::info("[BoneTagSerialPlugin] Starting serial communication thread with {}", serial_port_name);
        connectAndStartReading();

        unsigned iter_ = 0;
        while(running_)
        {
          if(iter_ == 0 || iter_ % rate_ == 0)
          {
            if(connect_requested_)
            {
              connectAndStartReading();
            }
            iter_ = 0;
          }
          ++iter_;
        }
        serial_->close_serial_port();
      });

  gc.controller().datastore().make<bool>("BoneTagSerialPlugin", true);
  gc.controller().datastore().make_call("BoneTagSerialPlugin::Connected", [this]() { return serial_->connected(); });
  gc.controller().datastore().make_call("BoneTagSerialPlugin::GetLastData",
                                        [this]() -> const io::Serial::Data &
                                        {
                                          std::lock_guard<std::mutex> lockDataMutex(dataMutex_);
                                          return lastData_;
                                        });
  gc.controller().datastore().make_call("BoneTagSerialPlugin::GetNewData",
                                        [this]() -> std::optional<io::Serial::Data>
                                        {
                                          // std::lock_guard<std::mutex> lockDataMutex(dataMutex_);
                                          if(lastDataIsNew_)
                                          {
                                            lastDataIsNew_ = false;
                                            return lastData_;
                                          }
                                          else
                                          {
                                            return std::nullopt;
                                          }
                                        });
  gc.controller().datastore().make_call("BoneTagSerialPlugin::Stop", [this]() -> void { running_ = false; });
  std::vector<std::string> label;
  {
    std::lock_guard<std::mutex> lockDataMutex(dataMutex_);

    label.resize(lastData_.size());
    for(int i = 0; i < lastData_.size(); ++i)
    {
      label[i] = std::to_string(i);
    }
  }

  gc.controller().gui()->addElement(
      {"BoneTagSerialPlugin"}, mc_rtc::gui::Label("Connected", [this]() { return serial_->connected(); }),
      mc_rtc::gui::Button("Connect", [this]() { connect_requested_ = true; }),
      mc_rtc::gui::ArrayLabel("Data", label,
                              [this]()
                              {
                                std::lock_guard<std::mutex> lockDataMutex(dataMutex_);
                                return lastData_;
                              }),
      mc_rtc::gui::NumberInput(
          "Alpha Filter", [this]() { return serial_->alphaFilter(); }, [this](double a) { serial_->alphaFilter(a); }),

      mc_rtc::gui::Button("Stop",
                          [this]()
                          {
                            running_ = false;
                            thread_.join();
                          }));
}

void BoneTagSerialPlugin::reset(mc_control::MCGlobalController & controller) {}

template<unsigned N, typename F>
auto make_plots_tuple(F && make_plot)
{
  // Helper to build a tuple of plots
  return [&]<std::size_t... Is>(std::index_sequence<Is...>)
  { return std::make_tuple(make_plot(Is)...); }(std::make_index_sequence<N>{});
}

void BoneTagSerialPlugin::before(mc_control::MCGlobalController & gc)
{
  std::lock_guard<std::mutex> lockReceivedDataMutex(serial_->received_data_mutex);

  // all_data.insert(all_data.end(),serial_->received_data.begin(),serial_->received_data.end());
  auto lastReceivedData = serial_->lastReceivedData();
  hasReceivedData_ = !lastReceivedData.empty();

  if(hasReceivedData_)
  {
    lastData_ = lastReceivedData;

    if(!plotDisplayed)
    {
      plotDisplayed = true;
      using Color = mc_rtc::gui::Color;
      using Style = mc_rtc::gui::plot::Style;
      const std::vector<std::pair<Color, Style>> sensorColors = {
          {Color(0.8, 0.0, 0.0), Style::Solid}, // Dark Red Solid
          {Color(0.8, 0.0, 0.0), Style::Dashed}, // Dark Red Dashed
          {Color(0.0, 0.0, 0.8), Style::Solid}, // Dark Blue Solid
          {Color(0.0, 0.0, 0.8), Style::Dashed}, // Dark Blue Dashed
          {Color(0.0, 0.6, 0.0), Style::Solid}, // Dark Green Solid
          {Color(0.0, 0.6, 0.0), Style::Dashed}, // Dark Green Dashed
          {Color(0.0, 0.0, 0.0), Style::Solid}, // Black Solid
          {Color(0.0, 0.0, 0.0), Style::Dashed}, // Black Dashed
          {Color(0.5, 0.0, 0.5), Style::Solid}, // Dark Magenta Solid
          {Color(0.5, 0.0, 0.5), Style::Dashed}, // Dark Magenta Dashed
          {Color(0.0, 0.5, 0.5), Style::Solid}, // Teal Solid
          {Color(0.0, 0.5, 0.5), Style::Dashed}, // Teal Dashed
          {Color(0.8, 0.4, 0.0), Style::Solid}, // Dark Orange Solid
          {Color(0.8, 0.4, 0.0), Style::Dashed}, // Dark Orange Dashed
          {Color(0.5, 0.2, 0.7), Style::Solid}, // Indigo Solid
          {Color(0.5, 0.2, 0.7), Style::Dashed}, // Indigo Dashed
          {Color(0.5, 0.3, 0.0), Style::Solid}, // Brown Solid
          {Color(0.5, 0.3, 0.0), Style::Dashed}, // Brown Dashed
          {Color(0.2, 0.2, 0.6), Style::Solid}, // Navy Solid
          {Color(0.2, 0.2, 0.6), Style::Dashed} // Navy Dashed
      };

      auto make_sensor_plot = [this, sensorColors](unsigned index)
      {
        const auto & colorStyle = sensorColors[index % sensorColors.size()];
        return mc_rtc::gui::plot::Y(
            fmt::format("Sensor {}", index + 1), [this, index]() { return lastData_[index]; }, colorStyle.first,
            colorStyle.second);
      };

      // Usage:
      // auto plots_tuple = make_plots_tuple<23>(make_sensor_plot);
      //
      // std::apply(
      //     [&](auto &&... plot_args)
      //     {
      //       gc.controller().gui()->addPlot("BoneTag Measurements", mc_rtc::gui::plot::X("N", [this]() { return t_;
      //       }),
      //                                      plot_args...);
      //     },
      //     plots_tuple);

      auto & gui = *gc.controller().gui();
      gui.addPlot("BoneTag Measurements", mc_rtc::gui::plot::X("t", [this]() { return t_; }));

      for(unsigned i = 0; i < serial_->SENSOR_COUNT; ++i)
      {
        gui.addPlotData("BoneTag Measurements", make_sensor_plot(i));
      }

      std::vector<double> data;
      data.resize(lastData_.size());
      gc.controller().logger().addLogEntry("BoneTag_Sensors", this,
                                           [this, data]() mutable -> std::vector<double>
                                           {
                                             for(int i = 0; i < lastData_.size(); ++i)
                                             {
                                               data[i] = lastData_[i];
                                             }
                                             return data;
                                           });
    }
  }
  hasReceivedData_ = false;
}

void BoneTagSerialPlugin::after(mc_control::MCGlobalController & controller)
{
  t_ += controller.timestep();
}

mc_control::GlobalPlugin::GlobalPluginConfiguration BoneTagSerialPlugin::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = true;
  out.should_always_run = false;
  return out;
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("BoneTagSerialPlugin", mc_plugin::BoneTagSerialPlugin)
