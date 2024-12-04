#pragma once
#include <winsock2.h>
#include <atomic>
#include <thread>
#include <mutex>
#include <vector>
#include <memory>
#include "nlohmann/json.hpp"


#pragma comment(lib, "ws2_32.lib")

#ifndef __CM_YD_Glove_H__
#define __CM_YD_Glove_H__

#define YDGlove_SDK __declspec( dllexport )

#endif

using json = nlohmann::json;
using namespace std;

class GloveSDK;

static mutex data_mutex;
static condition_variable data_cond;
static atomic<bool> data_available(false);
static mutex control_console_mux;



/*
* 骨骼数据
* Bone data 
*/
struct Bone
{
	string name;
	int parent = 0;
	vector<float> location = {};
	vector<float> rotation = {};
	vector<float> scale = {};	
};

struct Parameter 
{
	string name;
	float value;
};

/*
* 保存双手指关节旋转数据
* Saving data for both hands finger joint
*/
struct handsfingerJoint
{
	vector<Parameter> fingerJoint_L;
	vector<Parameter> fingerJoint_R;
};

/*
* 此结构体存放手套的数据
* The glove data is stored within this structure.
*/
struct HandData
{
	Bone bones;
	handsfingerJoint fingerJoints;
};

/*
* 此结构体存放获取到的数据
* Get all data is stored within this structure.
*/
struct GloveData
{
	string deviceName;
	HandData handDatas;
};


void print_control_thread(GloveSDK* glove_sdk);

/*
* 回调函数类型
* Callback function type
*/
using GloveCallBack = function<void(GloveSDK*)>;

/*
* 只打印框架
*/
void OnlyPrintFrame();

class YDGlove_SDK GloveSDK
{
public:
	/*
	* 初始化 socket 和线程
	* Init Soicket and thread
	*/
	bool Initialize(const char* udp_ip, unsigned short udp_port);

	/*
	* 关闭线程，清理socket
	* close thread, clear socket
	*/
	void Shutdown();

	/*
	* 手套数据，数量取决于软件发送几副手套的数据
	* The amount of glove data depends on the number of glove pairs sent by the software.
	*/
	vector<GloveData>gloveDataList;

	/*
	* 注册回调函数
	* Registering callback function
	*/
	void RegisterGloveCallBack(GloveCallBack callback);

	//Print all data
	void PrintAllGloveData(GloveData inputGloveData);
	

private:
	static const int BUFFER_SIZE = 80000; //缓存大小 //Cache size
	SOCKET m_udp_socket;
	atomic<bool> m_running;
	unique_ptr<thread> m_receive_thread;
	GloveCallBack dataUpdateCallback;

	void ReceiveThread();

	void ParseJson(char* json_buffer, int length);

	void IniData();

};



