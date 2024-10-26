// main.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include "UnicastLib/CM_YD_Unicast.h"


#pragma comment(lib,"UnicastLib/CM_YD_Unicast.lib")


/*
* 定义了一个函数用于注册回调
* The cache size defines a function for registering a callback
*/
void OnNewGloveData(GloveSDK* glovePtr)
{
	/*
	* 打印手数据
	* Print gloveData
	*/


	//Print all data 
	glovePtr->PrintAllGloveData(glovePtr->gloveDataList[0]);

	//// 获取设备的角色名
	//std::string name = glovePtr->gloveDataList[0].deviceName;
	//std::cout << "Device Name: " << name << std::endl;

	//// 获取左手标定状态
	//float l_calibrationStatus = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_L[0].value;
	//std::cout << "Left Hand Calibration Status: " << l_calibrationStatus << std::endl;

	//// 获取左手食指第二指节俯仰角
	//float l_indexIntermediatePitch = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_L[6].value;
	//std::cout << "Left Index Intermediate Pitch: " << l_indexIntermediatePitch << std::endl;

	cout << endl << endl;

}

int main()
{
	/*
	 * 创建一个 GloveSDK 的实例
	 * Create an instance of GloveSDK
	 */
	GloveSDK glove_sdk;

	/*
	* 注册回调
	* Registering callback function
	*/
	glove_sdk.RegisterGloveCallBack(OnNewGloveData);

	/*
	* 创建Socket并检测是否成功，成功则接受数据并解析
	* Create a Socket and check if it is successful. If successful, receive and parse the data
	*/
	if (!glove_sdk.Initialize("127.0.0.1", 7777))
	{
		cerr << "Failed to initialize GloveSDK." << endl;
		return -1;
	}
	/*
	* 开启打印线程，打印glove_sdk的数据
	* Start the print thread and print the glove_sdk data
	*/
	//thread print_th(print_control_thread, &glove_sdk);

	/*
	* 调用后只打印帧数
	* After calling, only the number of frames is printed
	*/
	while (true)
	{
		int a;
		cin >> a;

		switch (a)
		{

		case 1:
			OnlyPrintFrame();
			break;

		case 0:
			glove_sdk.Shutdown();
			printf("Shutdown");
			break;
		}
	}
	/*
	* 等待输入 保存程序
	* Waiting for input, Save program
	*/
	//getchar();

	/*
	* 关闭清理socket
	* close and clean up the socket
	*/
	glove_sdk.Shutdown();
	printf("Shutdown");

	return 0;
}

