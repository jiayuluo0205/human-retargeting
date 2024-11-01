// CM_YD_SDK.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#include "Lib/CM_YD_SDK.h"
#include <iostream>
#include<chrono>
#include <vector>
#include <sstream>  // 用于格式化四元数数据
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")  // 链接Windows套接字库
#pragma comment(lib,"Lib/CM_YD_SDK.lib")

double roll, pitch, yaw;

//void OnNewGloveData(GloveSDK* glovePtr) //定义了一个函数用于注册回调
//{
//	//这个函数将在接受线程中被调用
//
//	//打印手数据
//
//	auto a = glovePtr->GloveData[0].HandData_L.JointDatas[6];//取第6个数据
//
//	cout << "LeftRingProximal： " << a[1] << endl;//打印四元数的第二个数
//
//	glovePtr->toEulerAngle(a[0], a[1], a[2], a[3], roll, pitch, yaw);//四元数转欧拉角,注意 IMU 的数据是 WXYZ 顺序
//
//	cout << "roll:" << glovePtr->RadToAngle(roll) << " pitch:" << glovePtr->RadToAngle(pitch) << " yaw:" << glovePtr->RadToAngle(yaw) << endl;//打印角度
//
//}

//void OnNewGloveData(GloveSDK* glovePtr)
//{
//    // 打印左手数据
//    cout << "Left Hand Data:" << endl;
//
//    // 遍历每个关节的数据
//    for (int joint = 0; joint < 15; ++joint) // 左手有 15 个关节
//    {
//        // 获取每个关节的四元数数据
//        auto& quaternion = glovePtr->GloveData[0].HandData_L.JointDatas[joint];
//
//        // 根据 joint 的值决定关节的名称
//        string jointName;
//        switch (joint) {
//        case 0: jointName = "LeftIndexProximal"; break;
//        case 1: jointName = "LeftIndexIntermediate"; break;
//        case 2: jointName = "LeftIndexDistal"; break;
//        case 3: jointName = "LeftMiddleProximal"; break;
//        case 4: jointName = "LeftMiddleIntermediate"; break;
//        case 5: jointName = "LeftMiddleDistal"; break;
//        case 6: jointName = "LeftRingProximal"; break;
//        case 7: jointName = "LeftRingIntermediate"; break;
//        case 8: jointName = "LeftRingDistal"; break;
//        case 9: jointName = "LeftLittleProximal"; break;
//        case 10: jointName = "LeftLittleIntermediate"; break;
//        case 11: jointName = "LeftLittleDistal"; break;
//        case 12: jointName = "LeftThumbProximal"; break;
//        case 13: jointName = "LeftThumbIntermediate"; break;
//        case 14: jointName = "LeftThumbDistal"; break;
//        default: jointName = "Unknown Joint"; break;
//        }
//
//        // 打印关节名称和四元数数据
//        cout << jointName << ": Quaternion ["
//            << quaternion[0] << ", "  // W
//            << quaternion[1] << ", "  // X
//            << quaternion[2] << ", "  // Y
//            << quaternion[3] << "]"   // Z
//            << endl;
//
//        //// 将四元数转换为欧拉角
//        //double roll, pitch, yaw;
//        //glovePtr->toEulerAngle(quaternion[0], quaternion[1], quaternion[2], quaternion[3], roll, pitch, yaw);
//
//        //// 打印欧拉角
//        //cout << "Euler Angles for " << jointName << ": Roll: "
//        //    << glovePtr->RadToAngle(roll) << " Pitch: "
//        //    << glovePtr->RadToAngle(pitch) << " Yaw: "
//        //    << glovePtr->RadToAngle(yaw) << endl;
//    }
//
//    cout << endl; // 分隔不同手的数据
//
//    // 打印右手数据，类似左手
//    cout << "Right Hand Data:" << endl;
//
//    for (int joint = 0; joint < 15; ++joint) // 右手也有 15 个关节
//    {
//        // 获取右手的关节四元数数据
//        auto& quaternion = glovePtr->GloveData[0].HandData_R.JointDatas[joint];
//
//        // 根据 joint 的值决定关节的名称
//        string jointName;
//        switch (joint) {
//        case 0: jointName = "RightIndexProximal"; break;
//        case 1: jointName = "RightIndexIntermediate"; break;
//        case 2: jointName = "RightIndexDistal"; break;
//        case 3: jointName = "RightMiddleProximal"; break;
//        case 4: jointName = "RightMiddleIntermediate"; break;
//        case 5: jointName = "RightMiddleDistal"; break;
//        case 6: jointName = "RightRingProximal"; break;
//        case 7: jointName = "RightRingIntermediate"; break;
//        case 8: jointName = "RightRingDistal"; break;
//        case 9: jointName = "RightLittleProximal"; break;
//        case 10: jointName = "RightLittleIntermediate"; break;
//        case 11: jointName = "RightLittleDistal"; break;
//        case 12: jointName = "RightThumbProximal"; break;
//        case 13: jointName = "RightThumbIntermediate"; break;
//        case 14: jointName = "RightThumbDistal"; break;
//        default: jointName = "Unknown Joint"; break;
//        }
//
//        // 打印关节名称和四元数数据
//        cout << jointName << ": Quaternion ["
//            << quaternion[0] << ", "  // W
//            << quaternion[1] << ", "  // X
//            << quaternion[2] << ", "  // Y
//            << quaternion[3] << "]"   // Z
//            << endl;
//
//        //// 将四元数转换为欧拉角
//        //double roll, pitch, yaw;
//        //glovePtr->toEulerAngle(quaternion[0], quaternion[1], quaternion[2], quaternion[3], roll, pitch, yaw);
//
//        //// 打印欧拉角
//        //cout << "Euler Angles for " << jointName << ": Roll: "
//        //    << glovePtr->RadToAngle(roll) << " Pitch: "
//        //    << glovePtr->RadToAngle(pitch) << " Yaw: "
//        //    << glovePtr->RadToAngle(yaw) << endl;
//    }
//
//    cout << endl;
//}

void OnNewGloveData(GloveSDK* glovePtr)
{
    // 初始化Winsock
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);

    // 创建套接字
    SOCKET sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock == INVALID_SOCKET) {
        std::cerr << "Socket creation failed!" << std::endl;
        WSACleanup();
        return;
    }

    // 设置服务器地址
    sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(5555);  // 设置端口号
    if (inet_pton(AF_INET, "127.0.0.1", &serverAddr.sin_addr) <= 0) {
        std::cerr << "Invalid address/ Address not supported" << std::endl;
    }

    // 连接服务器
    if (connect(sock, (sockaddr*)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR) {
        std::cerr << "Connection failed!" << std::endl;
        closesocket(sock);
        WSACleanup();
        return;
    }

    // 遍历每个关节的数据并发送
    for (int joint = 0; joint < 15; ++joint)
    {
        // 获取每个关节的四元数数据
        auto& quaternion = glovePtr->GloveData[0].HandData_L.JointDatas[joint];

        // 根据 joint 的值决定关节的名称
        std::string jointName;
        switch (joint) {
        case 0: jointName = "LeftIndexProximal"; break;
        case 1: jointName = "LeftIndexIntermediate"; break;
        case 2: jointName = "LeftIndexDistal"; break;
        case 3: jointName = "LeftMiddleProximal"; break;
        case 4: jointName = "LeftMiddleIntermediate"; break;
        case 5: jointName = "LeftMiddleDistal"; break;
        case 6: jointName = "LeftRingProximal"; break;
        case 7: jointName = "LeftRingIntermediate"; break;
        case 8: jointName = "LeftRingDistal"; break;
        case 9: jointName = "LeftLittleProximal"; break;
        case 10: jointName = "LeftLittleIntermediate"; break;
        case 11: jointName = "LeftLittleDistal"; break;
        case 12: jointName = "LeftThumbProximal"; break;
        case 13: jointName = "LeftThumbIntermediate"; break;
        case 14: jointName = "LeftThumbDistal"; break;
        }

        // 拼接四元数数据成字符串
        std::ostringstream dataStream;
        dataStream << jointName << ": [" << quaternion[0] << ", " << quaternion[1] << ", " << quaternion[2] << ", " << quaternion[3] << "]";
        std::string data = dataStream.str();

        // 打印要发送的数据
        std::cout << "Sending data: " << data << std::endl;

        // 发送数据
        send(sock, data.c_str(), data.size(), 0);
    }

    // 关闭套接字
    closesocket(sock);
    WSACleanup();
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