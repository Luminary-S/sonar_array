#include "sonar_array/sonar_reader.h"

namespace sonar {

SonarReader::SonarReader(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
  if (!initSerial()) {
    ROS_WARN("Init Serial port %s failure!!", port_id_.c_str());
    return;
  }

  sonar_pub_ = nh.advertise<sensor_msgs::LaserEcho>(sonar_topic_name_, 10);

  uint8_t GET_DATA_CMD[] = {0x55, 0xaa, 0x01, 0x01, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  int cmd_len = sizeof(GET_DATA_CMD) / sizeof(GET_DATA_CMD[0]);
  getChecksum(GET_DATA_CMD, cmd_len);

  if (sonar_ser_.isOpen()) {
    ros::Rate r(pub_rate_);
    while (ros::ok()) {
      sonar_ser_.write(GET_DATA_CMD, cmd_len);
      //   ros::Duration(0.01).sleep();
      if (sonar_ser_.available() > 13) {
        uint8_t* data;
        sonar_ser_.read(data, 13);
        Int2Byte tfr;

        for (size_t i = 0; i < 4; i++) {
          tfr.byte[0] = data[5 + i * 2];
          tfr.byte[1] = data[5 + i * 2 - 1];
          ranges[i] = tfr.num;
        }

        sensor_msgs::LaserEcho echo;
        for (auto echo_data : ranges) {
          echo.echoes.push_back(echo_data);
        }
        sonar_pub_.publish(echo);
      }

      ros::spinOnce();
      r.sleep();
    }  // end of ros-ok while loop
  }
}

void SonarReader::initParam(ros::NodeHandle& nh_private) {
  nh_private.param("port_id", port_id_, std::string("/dev/ttyUSB)"));
  nh_private.param("baud_rate", baud_rate_, 9600);
  nh_private.param("pub_rate", pub_rate_, 10.0);
  nh_private.param("sonar_topic_name", sonar_topic_name_, std::string("sonar"));
}

bool SonarReader::initSerial() {
  try {
    sonar_ser_.setPort(port_id_);
    sonar_ser_.setBaudrate(baud_rate_);
    sonar_ser_.setParity(serial::parity_none);
    serial::Timeout time_out = serial::Timeout::simpleTimeout(1000);
    sonar_ser_.setTimeout(time_out);
    sonar_ser_.open();
  } catch (const serial::IOException& e) {
    ROS_ERROR("Unable to open port on %s -> [%s]", port_id_.c_str(), e.what());
    return false;
  }

  return true;
}

void SonarReader::getChecksum(uint8_t* cmd, const int& cmd_len) {
  int sum = 0;
  for (size_t i = 0; i < cmd_len; i++) {
    sum += cmd[i];
  }
  cmd[cmd_len - 1] = sum & 0xff;
}

}  // namespace sonar