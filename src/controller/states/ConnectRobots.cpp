#include "ConnectRobots.h"

#include <mc_control/fsm/Controller.h>

#include <mc_rbdyn/RobotLoader.h>
#include <mc_rtc/Schema.h>

struct ConnectRobotsSchema
{
  MC_RTC_NEW_SCHEMA(ConnectRobotsSchema)
  MC_RTC_SCHEMA_MEMBER(ConnectRobotsSchema, std::string, robot, "Robot Name", mc_rtc::schema::Interactive, "")
  MC_RTC_SCHEMA_MEMBER(ConnectRobotsSchema, bool, connect, "Connect/Disconnect", mc_rtc::schema::Interactive, true)
  MC_RTC_SCHEMA_MEMBER(ConnectRobotsSchema,
                       std::string,
                       connectModule,
                       "Connect with module",
                       mc_rtc::schema::Required | mc_rtc::schema::Interactive,
                       std::string{})
  MC_RTC_SCHEMA_MEMBER(ConnectRobotsSchema,
                       std::string,
                       fromLink,
                       "fromLink",
                       mc_rtc::schema::Required | mc_rtc::schema::Interactive,
                       std::string{})
  MC_RTC_SCHEMA_MEMBER(ConnectRobotsSchema,
                       std::string,
                       toLink,
                       "toLink",
                       mc_rtc::schema::Required | mc_rtc::schema::Interactive,
                       std::string{})
  MC_RTC_SCHEMA_MEMBER(ConnectRobotsSchema,
                       sva::PTransformd,
                       X_connect,
                       "X_connect",
                       mc_rtc::schema::Required | mc_rtc::schema::Interactive,
                       sva::PTransformd::Identity())
};

void ConnectRobots::start(mc_control::fsm::Controller & ctl)
{
  mc_rtc::log::info("config is {}", config_.dump(true, true));
  auto schema = ConnectRobotsSchema{};
  schema.load(config_);
  if(schema.robot.empty())
  {
    schema.robot = ctl.robot().name();
  }

  auto rm = ctl.robot(schema.robot).module();
  auto connect_rm = mc_rbdyn::RobotLoader::get_robot_module(schema.connectModule);
  if(schema.connect)
  {
    mc_rtc::log::info("[{}] Connecting robot module to robot {}", name(), schema.robot);
    auto connect = rm.connect(*connect_rm, schema.fromLink, schema.toLink, "",
                              mc_rbdyn::RobotModule::ConnectionParameters{}.X_other_connection(schema.X_connect));
    connect.name = schema.robot + "_connect";
    mc_rtc::log::info("[{}] Robot module connected to robot {}", name(), schema.robot);

    // TODO: this should replace the robot instead of creating a new one
    //
    // mc_rtc::log::info("[{}] Removing robot {}", name(), schema.robot);
    // ctl.robots().removeRobot(schema.robot);
    // mc_rtc::log::info("[{}] Removed robot {}", name(), schema.robot);

    mc_rtc::log::info("[{}] Loading robot {}", name(), schema.robot);
    ctl.loadRobot(connect, connect.name);
    mc_rtc::log::info("[{}] Loaded robot {}", name(), schema.robot + "_connect");
    mc_rtc::log::info("robots size: {}", ctl.robots().size());
    mc_rtc::log::info("robot new name: {}", ctl.robot(schema.robot + "_connect").name());
  }
  else
  {
    rm.disconnect(*connect_rm, schema.fromLink, schema.toLink, "", {});
  }
}

bool ConnectRobots::run(mc_control::fsm::Controller & ctl)
{
  return true;
}

void ConnectRobots::teardown(mc_control::fsm::Controller & ctl) {}

EXPORT_SINGLE_STATE("ConnectRobots", ConnectRobots)
