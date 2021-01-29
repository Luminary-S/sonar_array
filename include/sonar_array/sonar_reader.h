#ifndef SONAR_READER_H
#define SONAR_READER_H

#include "ros/ros.h"
#include "sensor_msgs/LaserEcho.h"
#include "serial/serial.h"

//union, num and byte share same memory, byte high is byte[1],byte low is byte[0]; used for 16 to 10 transfer
union Int2Byte {
  int16_t num;
  uint8_t byte[4];
};
union char2Byte{
  unsigned short val;
  unsigned char ch[2];
};

namespace sonar {

class SonarReader {
 public:
  SonarReader(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
  virtual ~SonarReader() {}
  void initParam(ros::NodeHandle& nh_private);
  bool initSerial();
  void getChecksum(uint8_t* cmd, const int& cmd_len);
  bool checkCheckSum(uint8_t* cmd, const int& cmd_len);
  bool crcCheck(unsigned char* cmd, unsigned int cmd_len);
 private:
  std::string port_id_, sonar_topic_name_;
  double pub_rate_;
  int baud_rate_;
  int ranges[4];

  serial::Serial sonar_ser_;
  ros::Publisher sonar_pub_;

};  // class SonarReader

}  // namespace sonar

#endif
