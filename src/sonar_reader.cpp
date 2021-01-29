#include "sonar_array/sonar_reader.h"
// #include "sonar_array/crcCompute.h"
namespace sonar {

void InvertUint8(unsigned char *dBuf,unsigned char *srcBuf)
{
	int i;
	unsigned char tmp[4]={0};
 
	for(i=0;i< 8;i++)
	{
		if(srcBuf[0]& (1 << i))
		tmp[0]|=1<<(7-i);
	}
	dBuf[0] = tmp[0];
	
}
void InvertUint16(unsigned short *dBuf,unsigned short *srcBuf)
{
	int i;
	unsigned short tmp[4]={0};
 
	for(i=0;i< 16;i++)
	{
		if(srcBuf[0]& (1 << i))
		tmp[0]|=1<<(15 - i);
	}
	dBuf[0] = tmp[0];
}
void InvertUint32(unsigned int *dBuf,unsigned int *srcBuf)
{
	int i;
	unsigned int tmp[4]={0};
	
	for(i=0;i< 32;i++)
	{
		if(srcBuf[0]& (1 << i))
		tmp[0]|=1<<(31 - i);
	}
	dBuf[0] = tmp[0];
}


unsigned short CRC16_MODBUS(unsigned char *data, unsigned int datalen)
{
	unsigned short wCRCin = 0xFFFF;
	unsigned short wCPoly = 0x8005;
	unsigned char wChar = 0;
	
	while (datalen--) 	
	{
		wChar = *(data++);
		InvertUint8(&wChar,&wChar);
		wCRCin ^= (wChar << 8);
		for(int i = 0;i < 8;i++)
		{
			if(wCRCin & 0x8000)
				wCRCin = (wCRCin << 1) ^ wCPoly;
			else
				wCRCin = wCRCin << 1;
		}
	}
	InvertUint16(&wCRCin,&wCRCin);
	return ((wCRCin>>8)|(wCRCin<<8));
}

SonarReader::SonarReader(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {

  initParam(nh_private);
  if (!initSerial()) {
    ROS_WARN("Init Serial port %s failure!!", port_id_.c_str());
    return;
  }


  sonar_pub_ = nh.advertise<sensor_msgs::LaserEcho>(sonar_topic_name_, 10);

  uint8_t GET_DATA_CMD[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x10, 0x44, 0x06};
  int cmd_len = sizeof(GET_DATA_CMD) / sizeof(GET_DATA_CMD[0]);
  // getChecksum(GET_DATA_CMD, cmd_len);
  // std::cout << "in loop" << std::endl;
  if (sonar_ser_.isOpen()) {
    ros::Rate r(pub_rate_);
    while (ros::ok()) {
      sonar_ser_.write(GET_DATA_CMD, cmd_len);
    //  ros::Duration(0.1).sleep();

      int ct = 0;
      // std::cout << "data len : " << sonar_ser_.available() << std::endl;
      while (sonar_ser_.available() < 37) {
        // std::cout << "in loop" << std::endl;
        ros::Duration(0.001).sleep();
	ct++;
      }
      std::cout << ct*0.001 << std::endl;
      // std::cout << sonar_ser_.available() << std::endl;

      int data_len = 37;
      uint8_t* data = new uint8_t[data_len];
      sonar_ser_.read(data, data_len);
      // std::cout << data << std::endl;
      Int2Byte tfr;
      //int data_len = sizeof(&data);
      if ( ! crcCheck( data, 37) ){
        ROS_WARN("sonic array measurement error!");
         continue;
      }

      for (size_t i = 0; i < 4; i++) {
        tfr.byte[0] = data[4 + i * 4];
        tfr.byte[1] = data[4 + i * 4 - 1];
        tfr.byte[2] = data[4 + i * 4 - 2];
        tfr.byte[3] = data[4 + i * 4 - 3];
        ranges[i] = tfr.num;
      }
      delete[] data;
      
      // new fitting result revised here........................
      ranges[0] = 7.64357237e-02 * ranges[0] + 1.57451786e+02;
      ranges[1] = 7.81456318e-02 * ranges[1] + 1.59899913e+02 ;
      ranges[2] = 7.75188411e-02 * ranges[2] + 1.55329208e+02;
      ranges[3] = 7.82993902e-02 * ranges[3] + 1.70219036e+02 ;

      sensor_msgs::LaserEcho echo;
      for (auto echo_data : ranges) {
        // echo_data = 0.1938 * echo_data + 53.836;
        echo.echoes.push_back(echo_data);
        std::cout << echo_data << " ";
      }
      std::cout <<  std::endl;
      // std::cout << echo << std::endl;
      sonar_pub_.publish(echo);

      ros::spinOnce();
      r.sleep();
    }  // end of ros-ok while loop
  }
}
// void SonarReader::update_exp()

void SonarReader::initParam(ros::NodeHandle& nh_private) {
  nh_private.param("port_id", port_id_, std::string("/dev/ttyUSB0"));
  nh_private.param("baud_rate", baud_rate_, 9600);
  nh_private.param("pub_rate", pub_rate_, 10.0);
  nh_private.param("sonar_topic_name", sonar_topic_name_, std::string("sonar"));
}

bool SonarReader::initSerial() {
  try {
    sonar_ser_.setPort(port_id_);
    sonar_ser_.setBaudrate(baud_rate_);
    sonar_ser_.setParity(serial::parity_even); //  --------------
    // sonar_ser_
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

bool SonarReader::checkCheckSum(uint8_t* cmd, const int& cmd_len){
   int sum = 0;
   for (size_t i = 0; i < cmd_len; i++) {
     sum += cmd[i];
   }
   int temp = sum & 0xff;

   if (temp == cmd[cmd_len - 1]){
     return true;
   }
	// return true;
   
   return false;
}

bool SonarReader::crcCheck(unsigned char* cmd, unsigned int cmd_len){
  // unsigned char cmd[] = {0x01, 0x03, 0x00, 0x14, 0x00, 0x10, 0x04, 0x02};
  // cmd_len = 6;
  char2Byte crc;
  crc.ch[0] = cmd[cmd_len-1];
  crc.ch[1] = cmd[cmd_len-2];
  unsigned short temp_crc = CRC16_MODBUS(cmd, cmd_len-2);

  // unsigned short crc = (unsigned char)(cmd[cmd_len-2]) +  (unsigned char)(cmd[cmd_len-1]);
  // std::cout << "self calculate: " <<temp_crc << std::endl;
  // printf("%04x\n", temp_crc);
  // printf("%04x\n", crc);
  // std::cout << "sfrom cmd: " << crc.val << std::endl;
  if (temp_crc == crc.val)
   return true;
  return false;
  // std::cout << "sfrom cmd: " << cmd[cmd_len-1] << std::endl;
  // return true;
}


}  // namespace sonar
