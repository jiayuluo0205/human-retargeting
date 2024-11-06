// CM_YD_SDK_Lib_2022.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include "Lib/CM_YD_SDK.h"
#pragma comment(lib,"Lib/CM_YD_SDK.lib")

double roll, pitch, yaw;

/*
* 定义了一个函数用于注册回调
* The cache size defines a function for registering a callback
*/
void OnNewGloveData(GloveSDK* glovePtr)
{
	/*
	* 这个函数将在接受线程中被调用
	* This function will be called in the receiving thread
	*
	* 打印手数据
	* Print gloveData
	*/
	auto a = glovePtr->GloveData[0].HandData_L.JointDatas[6];//取第6个数据  ////Get the 6th data

	cout << "LeftRingProximal： " << a[1] << endl;//打印四元数的第二个数 //Print the second number of quaternions

	glovePtr->toEulerAngle(a[0], a[1], a[2], a[3], roll, pitch, yaw);//四元数转欧拉角,注意 IMU 的数据是 WXYZ 顺序  //Convert quaternions to Euler angles, note that IMU data is in WXYZ order

	cout << "roll:" << glovePtr->RadToAngle(roll) << " pitch:" << glovePtr->RadToAngle(pitch) << " yaw:" << glovePtr->RadToAngle(yaw) << endl;//打印角度  //Print angle

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