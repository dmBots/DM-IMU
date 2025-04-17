#include "dm_imu/imu_driver.h"
#include "std_msgs/Float64MultiArray.h"

namespace dmbot_serial
{
DmImu::DmImu(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
{
  nh_private.param("port", imu_serial_port, std::string("/dev/ttyACM1")); 
  nh_private.param("baud", imu_seial_baud, 921600);
                 
  imu_msgs.header.frame_id = "imu_link";

  init_imu_serial();//初始化串口

  imu_pub_ = nh.advertise<sensor_msgs::Imu>("imu/data", 2);
  imu_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("pose", 100);
  euler2_pub_ = nh.advertise<std_msgs::Float64MultiArray>("dm/imu", 9);

  enter_setting_mode();
  ros::Duration(0.01).sleep();

  turn_on_accel();
  ros::Duration(0.01).sleep();

  turn_on_gyro();
  ros::Duration(0.01).sleep();

  turn_on_euler();
  ros::Duration(0.01).sleep();

  turn_off_quat();
  ros::Duration(0.01).sleep();

  set_output_1000HZ();
  ros::Duration(0.01).sleep();

  save_imu_para();
  ros::Duration(0.01).sleep();
  
  exit_setting_mode();
  ros::Duration(0.1).sleep();

  rec_thread = std::thread(&DmImu::get_imu_data_thread, this);

  ROS_INFO("imu init complete");
}

DmImu::~DmImu()
{ 
  ROS_INFO("enter ~DmImu()");
  
  stop_thread_ = true;
  
  if(rec_thread.joinable())
  {
    rec_thread.join(); 
  }

  if (serial_imu.isOpen())
  {
    serial_imu.close(); 
  } 
}

void DmImu::init_imu_serial()
{         
    try
    {
      serial_imu.setPort(imu_serial_port);
      serial_imu.setBaudrate(imu_seial_baud);
      serial_imu.setFlowcontrol(serial::flowcontrol_none);
      serial_imu.setParity(serial::parity_none); //default is parity_none
      serial_imu.setStopbits(serial::stopbits_one);
      serial_imu.setBytesize(serial::eightbits);
      serial::Timeout time_out = serial::Timeout::simpleTimeout(20);
      serial_imu.setTimeout(time_out);
      serial_imu.open();
      //usleep(1000000);//1s
    } 
    catch (serial::IOException &e)  // 抓取异常
    {
        ROS_ERROR_STREAM("In initialization,Unable to open imu serial port ");
        exit(0);
    }
    if (serial_imu.isOpen())
    {
        ROS_INFO_STREAM("In initialization,Imu Serial Port initialized");
    }
    else
    {
        ROS_ERROR_STREAM("In initialization,Unable to open imu serial port ");
        exit(0);
    }
   
}

void DmImu::enter_setting_mode()
{
  uint8_t txbuf[4]={0xAA,0x06,0x01,0x0D};
  for(int i=0;i<5;i++)
  {
    serial_imu.write(txbuf,sizeof(txbuf));
    ros::Duration(0.01).sleep();
  }
}

void DmImu::turn_on_accel()
{
  uint8_t txbuf[4]={0xAA,0x01,0x14,0x0D};
  for(int i=0;i<5;i++)
  {
    serial_imu.write(txbuf,sizeof(txbuf));
    ros::Duration(0.01).sleep();
  }
}

void DmImu::turn_on_gyro()
{
  uint8_t txbuf[4]={0xAA,0x01,0x15,0x0D};
  for(int i=0;i<5;i++)
  {
    serial_imu.write(txbuf,sizeof(txbuf));
    ros::Duration(0.01).sleep();
  }
}

void DmImu::turn_on_euler()
{
  uint8_t txbuf[4]={0xAA,0x01,0x16,0x0D};
  for(int i=0;i<5;i++)
  {
    serial_imu.write(txbuf,sizeof(txbuf));
    ros::Duration(0.01).sleep();
  }
}

void DmImu::turn_off_quat()
{
  uint8_t txbuf[4]={0xAA,0x01,0x07,0x0D};
  for(int i=0;i<5;i++)
  {
    serial_imu.write(txbuf,sizeof(txbuf));
    ros::Duration(0.01).sleep();
  }
}

void DmImu::set_output_1000HZ()
{
  uint8_t txbuf[5]={0xAA,0x02,0x01,0x00,0x0D};
  for(int i=0;i<5;i++)
  {
    serial_imu.write(txbuf,sizeof(txbuf));
    ros::Duration(0.01).sleep();
  }
}

void DmImu::save_imu_para()
{
  uint8_t txbuf[4]={0xAA,0x03,0x01,0x0D};
  for(int i=0;i<5;i++)
  {
    serial_imu.write(txbuf,sizeof(txbuf));
    ros::Duration(0.01).sleep();
  }
}

void DmImu::exit_setting_mode()
{
  uint8_t txbuf[4]={0xAA,0x06,0x00,0x0D};
  for(int i=0;i<5;i++)
  {
    serial_imu.write(txbuf,sizeof(txbuf));
    ros::Duration(0.01).sleep();
  }
}

void DmImu::restart_imu()
{
  uint8_t txbuf[4]={0xAA,0x00,0x00,0x0D};
  for(int i=0;i<5;i++)
  {
    serial_imu.write(txbuf,sizeof(txbuf));
    ros::Duration(0.01).sleep();
  }
}

void DmImu::get_imu_data_thread()
{ 
  int error_num=0;
  while (ros::ok()&&!stop_thread_)
  {    
    if (!serial_imu.isOpen())
    {
      ROS_WARN("In get_imu_data_thread,imu serial port unopen");
    }       

    ros::Time start_time = ros::Time::now();

    serial_imu.read((uint8_t*)(&receive_data.FrameHeader1),4);
    if(receive_data.FrameHeader1==0x55&&receive_data.flag1==0xAA&&receive_data.slave_id1==0x01&&receive_data.reg_acc==0x01)
    {
      serial_imu.read((uint8_t*)(&receive_data.accx_u32),57-4); 

      if(Get_CRC16((uint8_t*)(&receive_data.FrameHeader1), 16)==receive_data.crc1)
      {
        //std::cerr<<"calculate1: "<<Get_CRC16((uint8_t*)(&receive_data.FrameHeader1), 16)<<std::endl;
        //std::cerr<<"actuaal1: "<<receive_data.crc1<<std::endl;
        data.accx =*((float *)(&receive_data.accx_u32));
        data.accy =*((float *)(&receive_data.accy_u32));
        data.accz =*((float *)(&receive_data.accz_u32));
      }
      if(Get_CRC16((uint8_t*)(&receive_data.FrameHeader2), 16)==receive_data.crc2)
      {
        //std::cerr<<"calculate2: "<<Get_CRC16((uint8_t*)(&receive_data.FrameHeader2), 16)<<std::endl;
        //std::cerr<<"actuaal2: "<<receive_data.crc2<<std::endl;
        data.gyrox =*((float *)(&receive_data.gyrox_u32));
        data.gyroy =*((float *)(&receive_data.gyroy_u32));
        data.gyroz =*((float *)(&receive_data.gyroz_u32));
      }
      if(Get_CRC16((uint8_t*)(&receive_data.FrameHeader3), 16)==receive_data.crc3)
      { 
        //std::cerr<<"calculate3: "<<Get_CRC16((uint8_t*)(&receive_data.FrameHeader3), 16)<<std::endl;
        //std::cerr<<"actuaal3: "<<receive_data.crc3<<std::endl;
        data.roll =*((float *)(&receive_data.roll_u32));
        data.pitch =*((float *)(&receive_data.pitch_u32));
        data.yaw =*((float *)(&receive_data.yaw_u32));
        
      }
        // publish imu message
        imu_msgs.header.stamp = ros::Time::now();
  
        imu_msgs.orientation = tf::createQuaternionMsgFromRollPitchYaw(
          data.roll/ 180.0 * M_PI,data.pitch/ 180.0 * M_PI,data.yaw/ 180.0 * M_PI);

        imu_msgs.angular_velocity.x = data.gyrox;
        imu_msgs.angular_velocity.y = data.gyroy;
        imu_msgs.angular_velocity.z = data.gyroz;
    
        imu_msgs.linear_acceleration.x = data.accx;
        imu_msgs.linear_acceleration.y = data.accy;
        imu_msgs.linear_acceleration.z = data.accz;
    
        imu_pub_.publish(imu_msgs);

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "imu_link";
        pose.header.stamp = imu_msgs.header.stamp;
        pose.pose.position.x = 0.0;
        pose.pose.position.y = 0.0;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = imu_msgs.orientation.w;
        pose.pose.orientation.x = imu_msgs.orientation.x;
        pose.pose.orientation.y = imu_msgs.orientation.y;
        pose.pose.orientation.z = imu_msgs.orientation.z;
        
        imu_pose_pub_.publish(pose);

        std_msgs::Float64MultiArray euler2_msg;
        euler2_msg.data.resize(9);
        euler2_msg.data[0]=data.gyrox;
        euler2_msg.data[1]=data.gyroy;
        euler2_msg.data[2]=data.gyroz;
        euler2_msg.data[3]=data.accx;
        euler2_msg.data[4]=data.accy;
        euler2_msg.data[5]=data.accz;
        euler2_msg.data[6]=data.roll/ 180.0 * M_PI;
        euler2_msg.data[7]=data.pitch/ 180.0 * M_PI;
        euler2_msg.data[8]=data.yaw/ 180.0 * M_PI;
        //euler2_pub_.publish(euler2_msg);
    }
    else
    { 
      error_num++;
      if(error_num>1200)
      {
        std::cerr<<"fail to get the correct imu data,finding header 0x55"<<std::endl;
      }
    }
  }
}

}


