#ifndef _FILEREAD
#define _FILEREAD
#include "ReadConfig.h"

#ifdef USE_Ros
#include "ros/ros.h"
#include <ros/package.h>
#include "sensor_msgs/Imu.h"
#include "novatel_gps_msgs/Inspva.h"
#include <novatel_gps_driver/parsers/inspva.h>
#include <novatel_gps_driver/parsers/gpgga.h>
#endif

#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <stdio.h>
#include <opencv2/opencv.hpp>  
#include <opencv2/calib3d.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include "Types.h"
extern std::string PATH ;
extern std::string namespath;
extern std::string inspvapath;
extern std::string imupath;
extern std::string rawimupath;
extern std::string gpggapath;
extern std::vector<double> d0,d1,d2,d3,d4,d5;
extern std::vector<string> s0,s1,s2,s3,s4,s5;
extern int e0,e1,e2,e3,e4,e5;
//extern ros::Publisher lidar_pub,leftimg_pub,rightimg_pub,imu_pub,gpgga_pub,inspva_pub;
using namespace std;
//这个文件的作用是读取文件到程序中	

  class Inspva {
  public:

    double time = 0.0;
    int week = 0;
    double second_of_week=0.0;
    double latitude = 0.0;
    double longitude = 0.0;
    double height = 0.0;
    double north_velocity = 0.0;
    double east_velocity = 0.0;
    double up_velocity = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
    double azimuth = 0.0;
    string status;
    
    int extended_status = 0;
    static double origin_longitude;
    static double origin_latitude;
    static double origin_altitude;
    
  };
  
  
    class Imu {
     public:
       
    double time=0.0;
    double orientation_x=0.0;
    double orientation_y=0.0;
    double orientation_z=0.0;
    double orientation_w=0.0;
    double angular_velocity_x=0.0;
    double angular_velocity_y=0.0;
    double angular_velocity_z=0.0;
    double linear_acceleration_x=0.0;
    double linear_acceleration_y=0.0;
    double linear_acceleration_z=0.0;
    
  };
  
  
  
  class RawImu {
     public:
       
     double time = 0.0;
  /*   int week = 0;
     double seconds_into_week=0.0;
     int imu_state=0;   */
     double z_accel_output=0.0;
     double y_accel_output=0.0;
     double x_accel_output=0.0;
     double z_gyro_output=0.0;
     double y_gyro_output=0.0;
     double x_gyro_output=0.0;
     
  };
  
  
  
  class Gpgga{
    public:
   double time = 0.0;
   double utc_seconds=0.0;
   double lat=0.0;
   double lon=0.0;
   string lat_dir;
   string lon_dir;
   int gps_qual=0;
   int num_sats=0;
   float hdop=0.0;
   float alt=0.0;
   string altitude_units;
   float undulation=0.0;
   string undulation_units;
   int diff_age=0;
   string station_id;
  }; 
  	  
	vector<string> read_format(string content,string s)
	{
	  int l=0;
	  vector<string> result;
	  while(content.find(s)!=std::string::npos)
	  {
	    int locate = content.find(s);
	    string s_num= content.substr(0,locate);
	    l = content.length();
	    content = content.substr(locate+1,l-locate-1);
	    result.push_back(s_num);
	  }
	  result.push_back(content);
	  return result;
	}
	
        template <class Type>  
	Type stringToNum(const string& str)  
	{  
	    istringstream iss(str);  
	    Type num;  
	    iss >> num;  
	    return num;      
	} 

	//定位txt文件某一行
	ifstream & seek_to_line(ifstream & in,int w)
	{
	  int i;
	  char buffer[2000];
	  in.seekg(0,ios::beg);
	  for(i=0;i<w;i++)
	  {
	    in.getline(buffer,sizeof(buffer));
	  }
	  return in;
	}
	

	//用于读取如下类型文件：
	//name img size
	//752 480
	//存储结果为key = name img size. value ={752,480}
	std::map<string,vector<float>> read_confs(string path)
	{
	      map<string,vector<float>> res;
	      ifstream f;
	      f.open(path.c_str());
	      char buffer[2000];
	      string name;
	      vector<float> values;
	      while(!f.eof())
	      {
		f.getline(buffer,2000);
		string content = buffer;
		vector<string> messages = read_format(content," ");
		if(messages[0]=="name")
		{
		  if(values.size()!=0)
		  {
		    res.insert(pair<string,vector<float>>(name,values));
		    values.clear();
		  }
		  name = content;
		}
		else
		{
		  for(int i=0;i<messages.size();i++)
		  {
		    values.push_back(std::stof(messages[i]));
		  }
		}
	      }
	      res.insert(pair<string,vector<float>>(name,values));
	      return res;
	}
	
	
	
	pcl::PointCloud<pcl::PointXYZI> Read_KITTI_Bin(string path)//从kitti的 代码中拷贝下来的
	{
	   fstream input(path.c_str(), ios::in | ios::binary);
	   input.seekg(0, ios::beg);//调到文件最起始的位置
	   pcl::PointCloud<pcl::PointXYZI> points;
           int i;
	   for (i=0; input.good() && !input.eof(); i++)
	   {
		  pcl::PointXYZI point;
		  input.read((char *) &point.x, 3*sizeof(float));
		  input.read((char *) &point.intensity, sizeof(float));
		  points.push_back(point);
	    }
	    input.close();
	    return points;
	}
	
#ifdef USE_Ros
      //下面是读取图像
	sensor_msgs::Image read_Img(string path,double stamp )
	{
	  cout<<path.c_str()<<endl;
	  sensor_msgs::Image res;
	  cv::Mat cvimg = cv::imread(path,cv::IMREAD_UNCHANGED);
	  cv_bridge::CvImage cvi;
	  ros::Time img_stamp(stamp);
	  cvi.header.stamp = img_stamp;
	  cvi.header.frame_id = "camera";
	  cvi.encoding = "mono8";
	  cvi.image = cvimg.clone();
	  cvi.toImageMsg(res);
	  return res;
	}
	
	//读取激光
	sensor_msgs::PointCloud2 read_Lidar(string path,double stamp)
	{
	    sensor_msgs::PointCloud2 ros_cloud;
	    pcl::PointCloud<pcl::PointXYZI> res_pcl;
	    ifstream f;
	    f.open(path.c_str());
	    char buffer[2000];
// 	    for(int i=0;i<11;i++)//skip前面几个信息有关的行
// 	    {
// 	      f.getline(buffer,2000);
// 	    }
	    cout<<path.c_str()<<endl;
	    while(!f.eof())
	    {
		      f.getline(buffer,2000);
		      string content;
		      content = buffer;
		      //cout<<"1"<<endl;
		      if(content.length()!=0)
		      {
			  vector<string> messages = read_format(content," ");
			  pcl::PointXYZI temp;
			  temp.x = stringToNum<float>(messages[0]);
			  temp.y = stringToNum<float>(messages[1]);
			  temp.z = stringToNum<float>(messages[2]);
			  //temp.intensity = stringToNum<float>(messages[3]);
			  //cout<<temp.x<<endl;
			  if(!((temp.x==0)&&(temp.y==0)&&(temp.z==0)))
			  res_pcl.points.push_back(temp);
		      }
	    }
	    f.close();
	    
	    pcl::toROSMsg(res_pcl,ros_cloud);
	    ros_cloud.header.frame_id ="velodyne";
	    ros::Time lidar_stamp(stamp);
	    ros_cloud.header.stamp = lidar_stamp;
	    return ros_cloud;
	}
	
     	 sensor_msgs::Imu read_Imu(string path,double stamp,int s)
	{
	  sensor_msgs::Imu ros_imu;
	  sensor_msgs::Imu imu;
	   ifstream f;
	    f.open(path.c_str());
	    char buffer[2000];
	    seek_to_line(f,s);
		      f.getline(buffer,2000);
		      string content;
		      content = buffer;
		      if(content.length()!=0)
		      {
			  vector<string> messages = read_format(content," ");		  
	  imu.orientation.x = stringToNum<double>(messages[1]);
	  imu.orientation.y = stringToNum<double>(messages[2]);
	  imu.orientation.z = stringToNum<double>(messages[3]);
	  imu.orientation.w = stringToNum<double>(messages[4]);
	  imu.angular_velocity.x = stringToNum<double>(messages[5]);
	  imu.angular_velocity.y = stringToNum<double>(messages[6]);
	  imu.angular_velocity.z = stringToNum<double>(messages[7]);
	  imu.linear_acceleration.x = stringToNum<double>(messages[8]);
	  imu.linear_acceleration.y = stringToNum<double>(messages[9]);
	  imu.linear_acceleration.z = stringToNum<double>(messages[10]);
	    }
	   f.close();
	   ros_imu=imu;
	  ros_imu.header.frame_id ="camera_imu_frame";
	   ros::Time imu_stamp(stamp);
	   ros_imu.header.stamp = imu_stamp;
	  return ros_imu;
	}   
	
	novatel_gps_msgs::Inspva read_Inspva(string path,double stamp,int m)
	{  novatel_gps_msgs::Inspva ros_inspva;
	  novatel_gps_msgs::Inspva inspva;
	  ifstream f;
	    f.open(path.c_str());
	    char buffer[2000];
	    seek_to_line(f,m);
		      f.getline(buffer,2000);
		      string content;
		      content = buffer;
		      if(content.length()!=0)
		      {
			  vector<string> messages = read_format(content," ");
			  
	  inspva.week = stringToNum<int>(messages[1]);
	  inspva.seconds = stringToNum<double>(messages[2]);
          inspva.latitude = stringToNum<double>(messages[3]);
          inspva.longitude = stringToNum<double>(messages[4]);
	  inspva.height= stringToNum<double>(messages[5]);
	  inspva.north_velocity = stringToNum<double>(messages[6]);
	  inspva.east_velocity= stringToNum<double>(messages[7]);
	  inspva.up_velocity= stringToNum<double>(messages[8]);
	  inspva.roll= stringToNum<double>(messages[9]);
	  inspva.pitch= stringToNum<double>(messages[10]);
	  inspva.azimuth= stringToNum<double>(messages[11]);
	  inspva.status= messages[12];
		      }
	    
	     f.close();
	   ros_inspva = inspva;
	  ros_inspva.header.frame_id ="inspva";
	   ros::Time inspva_stamp(stamp);
	   ros_inspva.header.stamp = inspva_stamp;
	  return ros_inspva;
	  }   
			  
         novatel_gps_msgs::Gpgga read_Gpgga(string path,double stamp,int w)
	{ 
	 novatel_gps_msgs::Gpgga gpgga;
	 novatel_gps_msgs::Gpgga rosData_gpgga;
	 
	    ifstream f;
	    f.open(path.c_str());
	    char buffer[2000];
	    seek_to_line(f,w);
		      f.getline(buffer,2000);
		      string content;
		      content = buffer;
		      if(content.length()!=0)
		      {
			  vector<string> messages = read_format(content," ");
			 	  
	  gpgga.utc_seconds = stringToNum<double>(messages[1]);
	  gpgga.lat = stringToNum<int>(messages[2]);
	  gpgga.lon = stringToNum<double>(messages[3]);
          gpgga.lat_dir = messages[4];
          gpgga.lon_dir = messages[5];
	  gpgga.gps_qual= stringToNum<int>(messages[6]);
	  gpgga.num_sats = stringToNum<int>(messages[7]);
	  gpgga.hdop = stringToNum<float>(messages[8]);
	  gpgga.alt = stringToNum<float>(messages[9]);
	  gpgga.altitude_units = stringToNum<double>(messages[10]);
	  gpgga.undulation = stringToNum<float>(messages[11]);
	  gpgga.undulation_units = messages[12];
	  gpgga.diff_age = stringToNum<int>(messages[13]);
	  gpgga.station_id = messages[14];
	  cout<<to_string(gpgga.utc_seconds)<<" "<<to_string(gpgga.lat)<<" "<<to_string(gpgga.lon)<<" "<<gpgga.lat_dir<<" "<<gpgga.lon_dir<<" "<<to_string(gpgga.gps_qual)<<" "<<to_string(gpgga.num_sats)<<" "<<to_string(gpgga.hdop)<<" "<<to_string(gpgga.alt)<<" "<<gpgga.altitude_units<<" "<<to_string(gpgga.undulation)<<" "<<gpgga.undulation_units<<" "<<to_string(gpgga.diff_age)<<" "<<gpgga.station_id<<endl;
      
		      }
	    
	     f.close();
	     rosData_gpgga =gpgga;
	     rosData_gpgga.header.frame_id ="map";
	     rosData_gpgga.message_id ="GPGGA";
	     ros::Time gpgga_stamp(stamp);
	     rosData_gpgga.header.stamp = gpgga_stamp;	  
	  return rosData_gpgga;
	  }  
	  
	  struct Data
  { 
    sensor_msgs::Image leftimg,rightimg;
    sensor_msgs::PointCloud2 lidarpoints;
    sensor_msgs::Imu imu;
    novatel_gps_msgs::Inspva inspva;
    novatel_gps_msgs::Gpgga gpgga;
    double timestamp;
   };  
   
   
         vector<double> Initial(int& Img,int& Lidar,int& Inspva,int& Imu,int& Gpgga)
      {
	ifstream f0,f1,f2,f3,f4,f5;
	string c0,c1,c2,c3,c4,c5;
	vector<string> m0,m1,m2,m3,m4,m5;
        vector<double> allstamp={};	
	char b0[2000],b1[2000],b2[2000],b3[2000],b4[2000],b5[2000];
	
	if(Img==1)
	{
	  f0.open(namespath.c_str());       
	  while((!f0.eof()))
	  {     
		 f0.getline(b0,2000);
		 c0 = b0;
		  if(c0.length()!=0)
		  {
		    m0 = read_format(c0," ");
		    s0.push_back(m0[1]);
		    d0.push_back(stringToNum<double>(m0[1]));
		  }		  
	  }
         f0.close();
	 if(!d0.empty())
	 {allstamp.insert(allstamp.end(),d0.begin(),d0.end());}	
	}
	
	if(Lidar==1) 
	{
	  f1.open(namespath.c_str());       
	  while((!f1.eof()))
	  {
		 f1.getline(b1,2000);
		 c1 = b1;
		  if(c1.length()!=0)
		  {
		    m1 = read_format(c1," ");
		    s1.push_back(m1[0]);
		    d1.push_back(stringToNum<double>(m1[0]));
		  }		  
	  }
         f1.close();
	 if(!d1.empty())
	 {allstamp.insert(allstamp.end(),d1.begin(),d1.end());}	
	}
	
	if(Inspva==1) 
	{
	  f2.open(inspvapath.c_str());       
	  while((!f2.eof()))
	  {
		 f2.getline(b2,2000);
		 c2 = b2;
		  if(c2.length()!=0)
		  {
		    m2 = read_format(c2," ");
		    s2.push_back(m2[0]);
		    d2.push_back(stringToNum<double>(m2[0]));
		  }		  
	  }
         f2.close();
	 if(!d2.empty())
	 {allstamp.insert(allstamp.end(),d2.begin(),d2.end());}	
	}
	
	if(Imu==1) 
	{
	  f3.open(imupath.c_str());       
	  while((!f3.eof()))
	  {
		 f3.getline(b3,2000);
		 c3 = b3;
		  if(c3.length()!=0)
		  {
		    m3 = read_format(c3," ");
		    s3.push_back(m3[0]);
		    d3.push_back(stringToNum<double>(m3[0]));
		  }		  
	  }
         f3.close();
	 if(!d3.empty())
	 {allstamp.insert(allstamp.end(),d3.begin(),d3.end());}	
	}
	
	
	if(Gpgga==1) 
	{
	  f5.open(gpggapath.c_str());       
	  while((!f5.eof()))
	  {
		 f5.getline(b5,2000);
		 c5 = b5;
		  if(c5.length()!=0)
		  {
		    m5 = read_format(c5," ");
		    s5.push_back(m5[0]);
		    d5.push_back(stringToNum<double>(m5[0]));
		  }		  
	  }
         f4.close();
	 if(!d5.empty())
	 {allstamp.insert(allstamp.end(),d5.begin(),d5.end());}	
	}
	
	sort(allstamp.begin(),allstamp.end());
		
	return allstamp;
      }
      
   
     Data GetNextData(double a)
      {
	 Data data;
	 
	  if(!d0.empty()){
	  if(d0[e0]==a)
	   {
	    data.leftimg = read_Img(PATH +"data/leftImg/"+"left_"+s0[e0]+".png",d0[e0]);
	    data.rightimg = read_Img(PATH +"data/rightImg/"+"right_"+s0[e0]+".png",d0[e0]);
	    leftimg_pub.publish(data.leftimg);
	    rightimg_pub.publish(data.rightimg);
	    e0++;
	   }}
	   
	  if(!d1.empty()){  
	  if(d1[e1]==a)
	   {
	    data.lidarpoints = read_Lidar(PATH+"data/lidar/"+s1[e1]+".txt",d1[e1]);
	    lidar_pub.publish(data.lidarpoints);
	    e1++;
	   }}
	   
	  if(!d2.empty()){
	  if(d2[e2]==a)
	   {
	    data.inspva = read_Inspva(inspvapath,d2[e2],e2);
	    inspva_pub.publish(data.inspva);
	    e2++;
	   }}
	   
	  if(!d3.empty()){
	  if(d3[e3]==a)
	   {
	    data.imu = read_Imu(imupath,d3[e3],e3);
	    imu_pub.publish(data.imu);
	    e3++;
	   }}
	   
	   if(!d5.empty()){
	   if(d5[e5]==a)
	    {
	     data.gpgga=read_Gpgga(gpggapath,d5[e5],e5);
	     gpggaa_pub.publish(data.gpgga);
	     e5++;
	    }}
	     
	    data.timestamp = a;
	   return data;
       }
	  
#else  
	//下面是读取图像
	cv::Mat read_Img(string path,double stamp )
	{
	  cout<<path.c_str()<<endl;
	  cv::Mat cvimg = cv::imread(path,cv::IMREAD_UNCHANGED);
	  return cvimg;
	}
	
	//读取激光
	pcl::PointCloud<pcl::PointXYZI> read_Lidar(string path,double stamp)
	{
	    pcl::PointCloud<pcl::PointXYZI> res_pcl;
	    ifstream f;
	    f.open(path.c_str());
	    char buffer[2000];
	    cout<<path.c_str()<<endl;
	    while(!f.eof())
	    {
		      f.getline(buffer,2000);
		      string content;
		      content = buffer;
		      if(content.length()!=0)
		      {
			  vector<string> messages = read_format(content," ");
			  pcl::PointXYZI temp;
			  temp.x = stringToNum<float>(messages[0]);
			  temp.y = stringToNum<float>(messages[1]);
			  temp.z = stringToNum<float>(messages[2]);
			  if(!((temp.x==0)&&(temp.y==0)&&(temp.z==0)))
			  res_pcl.points.push_back(temp);
		      }
	    }
	    f.close();
	    return res_pcl;
	}
	
	
	   RawImu read_RawImu(string path,double stamp,int l)
	{
	     RawImu rawimu;
	   ifstream f;
	   f.open(path.c_str());
	    char buffer[2000];
	     cout<<path.c_str()<<endl;
	    seek_to_line(f,l);
		      f.getline(buffer,2000);
		      string content;
		      content = buffer;
		      if(content.length()!=0)
		      {
			  vector<string> messages = read_format(content," ");		 
          rawimu.time = stringToNum<double>(messages[0]);
          rawimu.z_accel_output= stringToNum<double>(messages[1]);
	  rawimu.y_accel_output=stringToNum<double>(messages[2]);
	  rawimu.x_accel_output= stringToNum<double>(messages[3]);
	  rawimu.z_gyro_output=stringToNum<double>(messages[4]);
	  rawimu.y_gyro_output=stringToNum<double>(messages[5]);
	  rawimu.x_gyro_output=stringToNum<double>(messages[6]);
		      }	  
           f.close();
	  return rawimu;	  
	}   
     	 Imu read_Imu(string path,double stamp,int s)
	{
	  Imu imu;
	   ifstream f;
	    f.open(path.c_str());
	    char buffer[2000];
	     cout<<path.c_str()<<endl;
	    seek_to_line(f,s);
		      f.getline(buffer,2000);
		      string content;
		      content = buffer;
		      if(content.length()!=0)
		      {
			  vector<string> messages = read_format(content," ");		  
	  imu.orientation_x = stringToNum<double>(messages[1]);
	  imu.orientation_y = stringToNum<double>(messages[2]);
	  imu.orientation_z = stringToNum<double>(messages[3]);
	  imu.orientation_w = stringToNum<double>(messages[4]);
	  imu.angular_velocity_x = stringToNum<double>(messages[5]);
	  imu.angular_velocity_y = stringToNum<double>(messages[6]);
	  imu.angular_velocity_z = stringToNum<double>(messages[7]);
	  imu.linear_acceleration_x = stringToNum<double>(messages[8]);
	  imu.linear_acceleration_y = stringToNum<double>(messages[9]);
	  imu.linear_acceleration_z = stringToNum<double>(messages[10]);
	    }
	   f.close();
	  return imu;
	}   
	
	Inspva read_Inspva(string path,double stamp,int m)
	{  
	  Inspva inspva;
	  ifstream f;
	    f.open(path.c_str());
	    char buffer[2000];
	     cout<<path.c_str()<<endl;
	    seek_to_line(f,m);
		      f.getline(buffer,2000);
		      string content;
		      content = buffer;
		      if(content.length()!=0)
		      {
			  vector<string> messages = read_format(content," ");
			  
	  inspva.week = stringToNum<int>(messages[1]);
	  inspva.second_of_week = stringToNum<double>(messages[2]);
          inspva.latitude = stringToNum<double>(messages[3]);
          inspva.longitude = stringToNum<double>(messages[4]);
	  inspva.height= stringToNum<double>(messages[5]);
	  inspva.north_velocity = stringToNum<double>(messages[6]);
	  inspva.east_velocity= stringToNum<double>(messages[7]);
	  inspva.up_velocity= stringToNum<double>(messages[8]);
	  inspva.roll= stringToNum<double>(messages[9]);
	  inspva.pitch= stringToNum<double>(messages[10]);
	  inspva.azimuth= stringToNum<double>(messages[11]);
	  inspva.status= messages[12];
		      }
	    
	     f.close();
	  return inspva;
	  }   
			  
         Gpgga read_Gpgga(string path,double stamp,int w)
	{ 
	 Gpgga gpgga;
	 
	    ifstream f;
	    f.open(path.c_str());
	     cout<<path.c_str()<<endl;
	    char buffer[2000];
	    seek_to_line(f,w);
		      f.getline(buffer,2000);
		      string content;
		      content = buffer;
		      if(content.length()!=0)
		      {
			  vector<string> messages = read_format(content," ");
			 	  
	  gpgga.utc_seconds = stringToNum<double>(messages[1]);
	  gpgga.lat = stringToNum<int>(messages[2]);
	  gpgga.lon = stringToNum<double>(messages[3]);
          gpgga.lat_dir = messages[4];
          gpgga.lon_dir = messages[5];
	  gpgga.gps_qual= stringToNum<int>(messages[6]);
	  gpgga.num_sats = stringToNum<int>(messages[7]);
	  gpgga.hdop = stringToNum<float>(messages[8]);
	  gpgga.alt = stringToNum<float>(messages[9]);
	  gpgga.altitude_units = stringToNum<double>(messages[10]);
	  gpgga.undulation = stringToNum<float>(messages[11]);
	  gpgga.undulation_units = messages[12];
	  gpgga.diff_age = stringToNum<int>(messages[13]);
	  gpgga.station_id = messages[14];
		      }
	    
	     f.close();
	  return gpgga;
	  }  
	  
	  
	   struct Data
  { 
    cv::Mat leftimg,rightimg;
    pcl::PointCloud<pcl::PointXYZI> lidarpoints;
    Inspva inspva;
    Imu imu;  
    Gpgga gpgga;
    RawImu rawimu;
    double timestamp;
   };
	  
#endif	  

 
      
     vector<double> Initial(int& Img,int& Lidar,int& Inspva,int& Imu,int& RawImu,int& Gpgga)
      {
	ifstream f0,f1,f2,f3,f4,f5;
	string c0,c1,c2,c3,c4,c5;
	vector<string> m0,m1,m2,m3,m4,m5;
        vector<double> allstamp={};	
	char b0[2000],b1[2000],b2[2000],b3[2000],b4[2000],b5[2000];
	
	if(Img==1)
	{
	  f0.open(namespath.c_str());       
	  while((!f0.eof()))
	  {     
		 f0.getline(b0,2000);
		 c0 = b0;
		  if(c0.length()!=0)
		  {
		    m0 = read_format(c0," ");
		    s0.push_back(m0[1]);
		    d0.push_back(stringToNum<double>(m0[1]));
		  }		  
	  }
         f0.close();
	 if(!d0.empty())
	 {allstamp.insert(allstamp.end(),d0.begin(),d0.end());}	
	}
	
	if(Lidar==1) 
	{
	  f1.open(namespath.c_str());       
	  while((!f1.eof()))
	  {
		 f1.getline(b1,2000);
		 c1 = b1;
		  if(c1.length()!=0)
		  {
		    m1 = read_format(c1," ");
		    s1.push_back(m1[0]);
		    d1.push_back(stringToNum<double>(m1[0]));
		  }		  
	  }
         f1.close();
	 if(!d1.empty())
	 {allstamp.insert(allstamp.end(),d1.begin(),d1.end());}	
	}
	
	if(Inspva==1) 
	{
	  f2.open(inspvapath.c_str());       
	  while((!f2.eof()))
	  {
		 f2.getline(b2,2000);
		 c2 = b2;
		  if(c2.length()!=0)
		  {
		    m2 = read_format(c2," ");
		    s2.push_back(m2[0]);
		    d2.push_back(stringToNum<double>(m2[0]));
		  }		  
	  }
         f2.close();
	 if(!d2.empty())
	 {allstamp.insert(allstamp.end(),d2.begin(),d2.end());}	
	}
	
	if(Imu==1) 
	{
	  f3.open(imupath.c_str());       
	  while((!f3.eof()))
	  {
		 f3.getline(b3,2000);
		 c3 = b3;
		  if(c3.length()!=0)
		  {
		    m3 = read_format(c3," ");
		    s3.push_back(m3[0]);
		    d3.push_back(stringToNum<double>(m3[0]));
		  }		  
	  }
         f3.close();
	 if(!d3.empty())
	 {allstamp.insert(allstamp.end(),d3.begin(),d3.end());}	
	}
	
	if(RawImu==1) 
	{
	  f4.open(rawimupath.c_str());       
	  while((!f4.eof()))
	  {
		 f4.getline(b4,2000);
		 c4 = b4;
		  if(c4.length()!=0)
		  {
		    m4 = read_format(c4," ");
		    s4.push_back(m4[0]);
		    d4.push_back(stringToNum<double>(m4[0]));
		  }		  
	  }
         f4.close();
	 if(!d4.empty())
	 {allstamp.insert(allstamp.end(),d4.begin(),d4.end());}	
	}
	
	if(Gpgga==1) 
	{
	  f5.open(gpggapath.c_str());       
	  while((!f5.eof()))
	  {
		 f5.getline(b5,2000);
		 c5 = b5;
		  if(c5.length()!=0)
		  {
		    m5 = read_format(c5," ");
		    s5.push_back(m5[0]);
		    d5.push_back(stringToNum<double>(m5[0]));
		  }		  
	  }
         f4.close();
	 if(!d5.empty())
	 {allstamp.insert(allstamp.end(),d5.begin(),d5.end());}	
	}
	
	sort(allstamp.begin(),allstamp.end());
		
	return allstamp;
      }
      
     Data GetNextData(double a)
      {
	 Data data;
	 
	  if(!d0.empty()){
	  if(d0[e0]==a)
	   {
	    data.leftimg = read_Img(PATH +"data/leftImg/"+"left_"+s0[e0]+".png",d0[e0]);
	    data.rightimg = read_Img(PATH +"data/rightImg/"+"right_"+s0[e0]+".png",d0[e0]);
	    e0++;
	   }}
	   
	  if(!d1.empty()){  
	  if(d1[e1]==a)
	   {
	    data.lidarpoints = read_Lidar(PATH+"data/lidar/"+s1[e1]+".txt",d1[e1]);
	    e1++;
	   }}
	   
	  if(!d2.empty()){
	  if(d2[e2]==a)
	   {
	    data.inspva = read_Inspva(inspvapath,d2[e2],e2);
	    e2++;
	   }}
	   
	  if(!d3.empty()){
	  if(d3[e3]==a)
	   {
	    data.imu = read_Imu(imupath,d3[e3],e3);
	    e3++;
	   }}
	   
	  if(!d4.empty()){
	  if(d4[e4]==a)
	   {
	    data.rawimu=read_RawImu(rawimupath,d4[e4],e4);
	    e4++;
	   }}
	   
	   if(!d5.empty()){
	   if(d5[e5]==a)
	    {
	     data.gpgga=read_Gpgga(gpggapath,d5[e5],e5);
	     e5++;
	    }}
	     
	    data.timestamp = a;
	   return data;
       }
	  


#endif

