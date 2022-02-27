#include "FileRead.h"
#include <string.h>
#include "ReadConfig.h"

#ifdef USE_Ros
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointField.h"
ros::Publisher lidar_pub,leftimg_pub,rightimg_pub,imu_pub,gpgga_pub,inspva_pub;
#endif

std::string PATH = "/home/yc/catkin_ws/src/lidar_camera_sample_data/";
std::string namespath = PATH+"data/names.txt";
std::string inspvapath = PATH+"data/gnssimu/inspva.txt";
std::string imupath = PATH+"data/gnssimu/imu.txt";
std::string rawimupath = PATH+"data/gnssimu/rawimu.txt";
std::string gpggapath = PATH+"data/gnssimu/gpgga.txt";
std::vector<double> d0,d1,d2,d3,d4,d5;
std::vector<string>s0,s1,s2,s3,s4,s5;
int e0=0,e1=0,e2=0,e3=0,e4=0,e5=0;
using namespace std;

#ifdef USE_Ros
int main(int argc, char **argv)
{
  ros::init(argc, argv, "read");
  ros::NodeHandle n;
  int Img=1,Lidar=1,Inspva=1,Imu=1,Gpgga=1;
   lidar_pub = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 1);
   leftimg_pub = n.advertise<sensor_msgs::Image>("/mynteye/left/image_raw", 1);
   rightimg_pub = n.advertise<sensor_msgs::Image>("/mynteye/right/image_raw", 1);
   imu_pub = n.advertise<sensor_msgs::Imu>("/imu",1);
   gpgga_pub = n.advertise<novatel_gps_msgs::Gpgga>("/gpgga",1);
   inspva_pub = n.advertise<novatel_gps_msgs::Inspva>("/inspva",1);
  
#else
  
int main()
{
int Img=1,Lidar=1,Inspva=1,Imu=1,RawImu=1,Gpgga=1;
#endif 

int i=0;
struct Data data;
std::vector<double> Allstamp = Initial(Img,Lidar,Inspva,Imu,RawImu,Gpgga);
  while(i <= Allstamp.size())
  {
   data = GetNextData(Allstamp[i]);
   i++;
   //cout<<data.inspva<<endl;
#ifdef USE_Ros    
   ros::spinOnce();
#endif
  }
 
}


