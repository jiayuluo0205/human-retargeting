// CM_YD_SDK.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#include "Lib/CM_YD_SDK.h"
#include <iostream>
#include<chrono>
#include <vector>
#pragma comment(lib,"Lib/CM_YD_SDK.lib")

double roll, pitch, yaw;


void OnNewGloveData(GloveSDK* glovePtr) //定义了一个函数用于注册回调
{
	//这个函数将在接受线程中被调用

	//打印手数据

	auto a = glovePtr->GloveData[0].HandData_L.JointDatas[6];//取第6个数据

	cout << "LeftRingProximal： " << a[1] << endl;//打印四元数的第二个数

	glovePtr->toEulerAngle(a[0], a[1], a[2], a[3], roll, pitch, yaw);//四元数转欧拉角,注意 IMU 的数据是 WXYZ 顺序

	cout << "roll:" << glovePtr->RadToAngle(roll) << " pitch:" << glovePtr->RadToAngle(pitch) << " yaw:" << glovePtr->RadToAngle(yaw) << endl;//打印角度

}

int main()
{
	//创建一个 GloveSDK 的实例
	GloveSDK glove_sdk;

	//注册回调
	glove_sdk.RegisterGloveCallBack(OnNewGloveData);

	//创建Socket并检测是否成功，成功则接受数据并解析
	if (!glove_sdk.Initialize("127.0.0.1", 7777))
	{
		cerr << "Failed to initialize GloveSDK." << endl;
		return -1;
	}

	//开启打印线程，打印glove_sdk的数据
	thread print_th(print_control_thread, &glove_sdk);

	//调用后只打印帧数


	while (true)
	{
		int a;
		a = 1;

		switch (a)
		{

		case 1:
			OnlyPrintFrame();
			//print_control_thread(&glove_sdk);
			break;

		case 0://结束，关闭清理socket
			glove_sdk.Shutdown();
			printf("Shutdown");
			break;
		}
	}

	//等待输入 保存程序
	//getchar();

	//结束，关闭清理socket
	glove_sdk.Shutdown();
	printf("Shutdown");



	return 0;
}

//数据示例
/*
{
	"DevicelD":1,
	"Framelndex" : 0,
	"DeviceName" : "UDXST0001L",
	"CalibrationStatus" : -2,
	"Battery" : 5,
	"Bones" : [
		[1, 1, 1, 1],//{"LeftindexProximal":[1,1,1,1]}
		[1, 1, 1, 1],///{"LeftindexIntermediate":[1,1,1,1,1]},
		[1, 1, 1, 1],//{"LeftindexDistal":[1,1,1,1,1]}
		[1, 1, 1, 1],//{"LeftMiddleProximal":[1,1,1,1]}
		[1, 1, 1, 1],//{"LeftMiddlelntermediate":[1,1,1,1]}
		[1, 1, 1, 1],//{"LeftMiddleDistal":[1,1,1,1]}
		[1, 1, 1, 1],//"LeftRingProximal":[1,1,1,1]},
		[1, 1, 1, 1],//{"LeftRingIntermediate":[1,1,1,1,1]}
		[1, 1, 1, 1],//{"LeftRingDistal":[1,1,1,1,1]}
		[1, 1, 1, 1],//{"LeftLittleProximal":[1,1,1, 1]}
		[1, 1, 1, 1],//{"LeftLittlelntermediate":[1,1,1,1,1,1]}
		[1, 1, 1, 1],//{"LeftLittleDistal":[1,1,1,1]
		[1, 1, 1, 1],//{"LeftThumbProximal":[1,1,1,1]}
		[1, 1, 1, 1],//{"LeftThumbintermediate":[1,1,1,1]}
		[1, 1, 1, 1],//{"LeftThumbDistal":[1,1,1,1]}
		[1, 1, 1, 1],//{"IMU数据
	]
}*/