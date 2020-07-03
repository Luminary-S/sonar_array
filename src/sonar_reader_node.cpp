#include "sonar_array/sonar_reader.h"

using sonar::SonarReader;

int main(int argc, char** argv) {
  ros::init(argc, argv, "sonar_reader_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  SonarReader sonar_reader(nh, nh_private);

  //   ros::spin();
  return 0;
}