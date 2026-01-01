#include "rclcpp/rclcpp.hpp"

std::atomic<bool> shutdown_flag(false);

void signal_handler(int signal) {
  std::cerr << "Interrupt signal (" << signal << ") received." << std::endl;
  shutdown_flag.store(true);
}

struct Parameters {
  std::string user_id;
  std::string robot_id;
  std::string ws_uri;
  std::string turn_url;
  std::string turn_username;
  std::string turn_credential;
  std::string rtc_sender_log_level;
  std::string webrtc_log_level;
  std::string remote_control_dc_label;
  std::string lidar_dc_label_prefix;
  std::string camera_media_track_label;
  std::string gps_coordinate_dc_label;
  std::string ros_2d_map_pose_dc_label;
};

std::shared_ptr<Parameters> GetParameters();

void SetMOSAICLog(const std::shared_ptr<Parameters>& parameters, rclcpp::Logger logger);
void SetWebRTCLog(const std::shared_ptr<Parameters>& parameters);

int main(int argc, char ** argv)
{
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  rclcpp::init(argc, argv);

  const auto parameters = GetParameters();

  SetWebRTCLog(parameters);

  return 0;
}
