#pragma once
#include <winsock2.h>
#include <atomic>
#include <thread>
#include <mutex>
#pragma comment(lib, "ws2_32.lib")
#include "Lib/nlohmann/json.hpp"

#ifndef __CM_YD_Glove_H__
#define __CM_YD_Glove_H__


#define YDGlove_SDK __declspec( dllexport )

#endif



using json = nlohmann::json;
using namespace std;


class GloveSDK;
const array<string, 5> FingerNames = { "Thumb", "Index", "Middle", "Ring", "Pinky" };
const array<string, 4> BonePartNames = { "Metacarpal", "Proximal", "Intermediate", "Distal" };
const array<string, 2> ThumbBonePartNames = { "Metacarpal", "Proximal" };

static mutex data_mutex;
static condition_variable data_cond;
static atomic<bool> data_available(false);
static mutex control_console_mux;

void OnlyPrintFrame();
//数据结构
struct HandDriverData
{
	int DeviceID = 0;
	int FrameIndex = 0;
	string DeviceName;
	int CalibrationStatus;
	int Battery;
	array<array<float, 4>, 16> JointDatas; //15个指节 每个指节有4旋转项数据 ，新增一个 imu 数据 位于第16个位置
	int Count = 0;
};

struct PairGloveData //此结构体存放了1双手套的数据 分为L和R
{
	HandDriverData HandData_L;
	HandDriverData HandData_R;
	bool bPrintData;
};
void print_control_thread(GloveSDK* glove_sdk);

using GloveCallBack = function<void(GloveSDK*)>; //回调函数类型

class YDGlove_SDK GloveSDK
{
public:
	//初始化 socket 和线程
	bool Initialize(const char* udp_ip, unsigned short udp_port);

	//关闭线程，清理socket
	void Shutdown();

	//是否要打印数据到控制台
	void EnablePrintData(int DeviceID, bool bEnable);

	//2组“1双手套”数据，因为现在软件里一台电脑只能发两幅，后面迭代版本会增多
	//GLoveData[0]就是ID为0的设备
	array<PairGloveData, 8> GloveData;//需要几双手套就把它改成几即可

	//注册回调函数
	void RegisterGloveCallBack(GloveCallBack callback);

	//显示手套数据（静态函数）
	static void show_data(HandDriverData& GloveData);

	//打印手的数据（包含15个指节+IMU+设备名字+id+标定状态）
	void PrintHandData(HandDriverData handData);

	//四元数转欧拉角
	void toEulerAngle(const double x, const double y, const double z, const double w, double& roll, double& pitch, double& yaw);

	//弧度到角度
	double RadToAngle(double rad);

private:
	static const int BUFFER_SIZE = 80000; //缓存大小
	SOCKET m_udp_socket;
	atomic<bool> m_running;
	thread m_receive_thread;
	void ReceiveThread();
	void ParseJson(char* json_buffer, int length);

	void TrySaveData(HandDriverData& GloveData, nlohmann::json received_data);

	void IniData();	

	GloveCallBack dataUpdateCallback;
};



