#include <vector>
#include <thread>
#include <iostream>
#include <ros/ros.h>
#include <controlcan.h>
#include <can_msgs/FrameArray.h>

using namespace std;
#define _NODE_NAME_ "usb_can_driver"
#define CAN_CHANNEL1 0
#define CAN_CHANNEL2 1
#define DEVICE_INDEX 0
#define CHANNEL_CNT  2

class UsbCanDriver
{
public:
	UsbCanDriver();
	~UsbCanDriver();
	void run();
	
private:
	bool rosInit();
	bool deviceInit();
	void closeDevice();
	bool configCanChannel(int channel);
	void frameArray_callback(const can_msgs::FrameArray::ConstPtr& msgs);
	void receiveThread();
	
private:
	vector<int>    mFilterMode; //0.1 all frame 
								  //2   only std frame
								  //3   only exd frame
	vector<int>    mAccCode;
	vector<int>    mMaskCode;
	vector<int>    mBaudrate;
	vector<string> mFrameId;     //不同frame_id 对应不同的通道
	
	ros::Subscriber mSubFrameArray;
	ros::Publisher  mPub;
};

UsbCanDriver::UsbCanDriver()
{
	mFilterMode.resize(CHANNEL_CNT);
	mAccCode.resize(CHANNEL_CNT);
	mMaskCode.resize(CHANNEL_CNT);
	mBaudrate.resize(CHANNEL_CNT);
	mFrameId.resize(CHANNEL_CNT);
}

UsbCanDriver::~UsbCanDriver()
{
	closeDevice();
}

void UsbCanDriver::closeDevice()
{
	if(1 == VCI_CloseDevice(VCI_USBCAN2,DEVICE_INDEX))
		printf("[%s] close device ok.\r\n", _NODE_NAME_);
	else
		printf("[%s] close device failed.\r\n", _NODE_NAME_);
}

bool UsbCanDriver::rosInit()
{
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	bool ok = false;
	
	ok = ros::param::get("~filter_mode",mFilterMode);
	if(!ok)
		for(int &mode:mFilterMode) mode = 0;
		
	ok = ros::param::get("~acc_code",mAccCode);
	if(!ok)
		for(int &acc:mAccCode) acc = 0x00000000;
	else
		for(int &acc:mAccCode) acc <<= 21;
		
	ok = ros::param::get("~mask_code",mMaskCode);
	if(!ok)
		for(int &mask:mMaskCode) mask = 0xFFFFFFFF;
	else
		for(int &mask:mMaskCode) mask <<= 21;
		
	ok = ros::param::get("~baudrate",mBaudrate);
	if(!ok)
	{
		for(int &baudrate:mBaudrate) 
		{
			baudrate = 500;
			ROS_INFO("[%s] baudrate set as default:%d",_NODE_NAME_,baudrate);
		}
	}
	
	ok = ros::param::get("~frame_id",mFrameId);
	if(!ok)
	{
		ROS_ERROR("[%s] no frame_id param!",_NODE_NAME_);
		return false;
	}
	
	std::string to_can_topic   = nh_private.param<std::string>("to_can_topic","/to_usbcan");
	std::string from_can_topic = nh_private.param<std::string>("from_can_topic","/from_usbcan");
	
	mSubFrameArray = nh.subscribe(to_can_topic,100,&UsbCanDriver::frameArray_callback,this);
	mPub = nh.advertise<can_msgs::FrameArray>(from_can_topic, 100);
	return true;
}

void UsbCanDriver::frameArray_callback(const can_msgs::FrameArray::ConstPtr& msgs)
{
	for(int channel=0; channel<CHANNEL_CNT; ++channel)
	{
		if(mFrameId[channel] != msgs->header.frame_id)
			continue;

		//ROS_INFO("callback %d", msgs->frames.size());
		for(int i=0; i< msgs->frames.size(); ++i)
		{
			if(msgs->frames[i].len ==0) 
				continue;
				
			VCI_CAN_OBJ canObj;
			canObj.ID = msgs->frames[i].id;
			canObj.SendType = 0; //0 auto resend if error
								 //1 never resend
			canObj.RemoteFlag = msgs->frames[i].is_rtr;
			canObj.ExternFlag = msgs->frames[i].is_extended;
			canObj.DataLen =    msgs->frames[i].len;
			for(int j=0; j<canObj.DataLen; ++j)
				canObj.Data[j] =  msgs->frames[i].data[j];
			
			VCI_Transmit(VCI_USBCAN2, DEVICE_INDEX, channel, &canObj, 1);
		}
	}
	
}

// 配置设备通道
bool UsbCanDriver::configCanChannel(int channel)
{
	VCI_INIT_CONFIG config;
	config.AccCode = mAccCode[channel];
	config.AccMask = mMaskCode[channel];
	
	cout << "[" << _NODE_NAME_ << "] " << "ch" << channel << "\t"
		 << hex << config.AccCode << "\t" << config.AccMask << endl;
	
	config.Filter =  mFilterMode[channel];
	if(mBaudrate[channel] == 125)
	{
		config.Timing0=0x03;
		config.Timing1=0x1C;
	}
	else if(mBaudrate[channel] == 250)
	{
		config.Timing0=0x01;
		config.Timing1=0x1C;
	}
	else if(mBaudrate[channel] == 500)
	{
		config.Timing0=0x00;
		config.Timing1=0x1C;
	}
	else if(mBaudrate[channel] == 1000)
	{
		config.Timing0=0x00;
		config.Timing1=0x14;
	}
	else
	{
		ROS_ERROR("[%s] channel: %d : baudrate error, \
			please query manual and update source code!",_NODE_NAME_,channel);
		return false;
	}
	
	config.Mode=0;// 0: normal,  1:only receive,  2:cycle
	
	if(VCI_InitCAN(VCI_USBCAN2,DEVICE_INDEX,channel,&config)!=1)
	{
		ROS_ERROR("[%s] init channel %d error!", _NODE_NAME_,channel);
		VCI_CloseDevice(VCI_USBCAN2,DEVICE_INDEX);
		return false;
	}

	if(VCI_StartCAN(VCI_USBCAN2,DEVICE_INDEX,channel)!=1)
	{
		ROS_ERROR("[%s] start channel %d error!", _NODE_NAME_,channel);
		VCI_CloseDevice(VCI_USBCAN2,DEVICE_INDEX);
		return false;
	}
	return true;
}

bool UsbCanDriver::deviceInit()
{
	VCI_BOARD_INFO deviceInfos[5];
	int deviceCnt = VCI_FindUsbDevice2(deviceInfos);
	if(deviceCnt <1 )
	{
		ROS_ERROR("[%s] : No available can device.",_NODE_NAME_);
		return false;
	}
	
	if(VCI_OpenDevice(VCI_USBCAN2,DEVICE_INDEX,0)==1)
		ROS_INFO("[%s] : open usbcan deivce success!",_NODE_NAME_);
	else
	{
		ROS_ERROR("[%s] : open usbcan deivce failed!",_NODE_NAME_);
		return false;
	}
	
	for(int i=0; i<CHANNEL_CNT; ++i)
	{
		if(!configCanChannel(i))
			return false;
	}
	return true;
}

void UsbCanDriver::receiveThread()
{
	const int MaxLen = 500;
	VCI_CAN_OBJ msgs[MaxLen];
	can_msgs::FrameArray frames;
	int reclen;
	while(ros::ok())
	{
		for(int channel=0; channel<CHANNEL_CNT; ++channel)
		{
			reclen = VCI_Receive(VCI_USBCAN2,DEVICE_INDEX,channel,msgs,MaxLen,100);
			if(reclen <= 0) continue;
			//ROS_INFO("[%s] receive msgs len: %d",_NODE_NAME_,reclen);
			
			frames.frames.resize(reclen);
			frames.header.frame_id = mFrameId[channel];
			
			for(int i=0; i<reclen; ++i)
			{
				frames.frames[i].id          = msgs[i].ID;
				frames.frames[i].is_rtr      = msgs[i].RemoteFlag;
				frames.frames[i].is_extended = msgs[i].ExternFlag;
				frames.frames[i].len         = msgs[i].DataLen;
				for(int j=0; j<frames.frames[i].len; ++j)
					frames.frames[i].data[j] = msgs[i].Data[j];
			}
			mPub.publish(frames);
			//ROS_INFO("[%s] : published ",_NODE_NAME_);
		}
		
		usleep(10000);
		
	}
}


void UsbCanDriver::run()
{
	if(!rosInit())    return; //else ROS_INFO("rosInit ok");
	if(!deviceInit()) return; //else ROS_INFO("deviceInit ok");
	
	std::thread t(&UsbCanDriver::receiveThread,this);
	
	ros::spin();
	closeDevice();
}

int main(int argc,char** argv)
{
	ros::init(argc, argv, _NODE_NAME_);
	
	UsbCanDriver app;
	app.run();
	
	ros::Duration(0.5).sleep();
	
	return 0;
}

 
