// main.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#include <winsock2.h>
#include <ws2tcpip.h>
#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <sstream>
#include <thread>
#include <cstdio> 
#include "UnicastLib/CM_YD_Unicast.h"


#pragma comment(lib,"UnicastLib/CM_YD_Unicast.lib")
SOCKET sock;

bool InitializeSocket()
{
	WSADATA wsaData;
	if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
		std::cerr << "WSAStartup failed!" << std::endl;
		return false;
	}

	sock = socket(AF_INET, SOCK_STREAM, 0);
	if (sock == INVALID_SOCKET) {
		std::cerr << "Socket creation failed!" << std::endl;
		WSACleanup();
		return false;
	}

	sockaddr_in serverAddr;
	serverAddr.sin_family = AF_INET;
<<<<<<< Updated upstream
	serverAddr.sin_port = htons(5556);
=======
	serverAddr.sin_port = htons(5555);
>>>>>>> Stashed changes
	if (inet_pton(AF_INET, "172.25.105.244", &serverAddr.sin_addr) <= 0) {
		std::cerr << "Invalid address/ Address not supported" << std::endl;
		closesocket(sock);
		WSACleanup();
		return false;
	}

	if (connect(sock, (sockaddr*)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR) {
		std::cerr << "Connection failed!" << std::endl;
		closesocket(sock);
		WSACleanup();
		return false;
	}

	std::cout << "Socket initialized and connected successfully!" << std::endl;
	return true;
}

void CleanupSocket()
{
	closesocket(sock);
	WSACleanup();
	std::cout << "Socket closed and cleaned up." << std::endl;
}

void PrintGloveData(GloveSDK* glovePtr) {
	// 获取和打印左手标定状态
	float l_calibrationStatus = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_L[0].value;
	std::cout << "Left Hand Calibration Status: " << l_calibrationStatus << "\n\n";

	// 打印左手关节数据
	std::cout << "Left Hand Joint Data:\n";
	for (int i = 1; i <= 28; ++i) {
		float value = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_L[i].value;
		std::cout << "  L" << std::setw(2) << i - 1 << ": " << std::fixed << std::setprecision(2) << value << "\n";
	}

	// 获取和打印右手标定状态
	float r_calibrationStatus = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_R[0].value;
	std::cout << "\nRight Hand Calibration Status: " << r_calibrationStatus << "\n\n";

	// 打印右手关节数据
	std::cout << "Right Hand Joint Data:\n";
	for (int i = 1; i <= 28; ++i) {
		float value = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_R[i].value;
		std::cout << "  R" << std::setw(2) << i - 1 << ": " << std::fixed << std::setprecision(2) << value << "\n";
	}

	std::cout << std::endl;
}

// 打包并通过套接字发送数据
void SendGloveData(GloveSDK* glovePtr) {
	std::ostringstream oss;

	// 获取左手标定状态
	oss << "Left Hand Calibration Status: "
		<< glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_L[0].value << "\n";

	// 打包左手关节数据
	for (int i = 1; i <= 28; ++i) {
		oss << "  L" << (i - 1) << ": "
			<< std::fixed << std::setprecision(2)
			<< glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_L[i].value << "\n";
	}

	// 获取右手标定状态
	oss << "\nRight Hand Calibration Status: "
		<< glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_R[0].value << "\n";

	// 打包右手关节数据
	for (int i = 1; i <= 28; ++i) {
		oss << "  R" << (i - 1) << ": "
			<< std::fixed << std::setprecision(2)
			<< glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_R[i].value << "\n";
	}

	// 将数据字符串发送到套接字
	std::string dataString = oss.str();
	int bytesSent = send(sock, dataString.c_str(), dataString.size(), 0);

	if (bytesSent == SOCKET_ERROR) {
		std::cerr << "Send failed!" << std::endl;
	}
	else {
<<<<<<< Updated upstream
		//std::cout << "Data sent successfully! Bytes sent: " << bytesSent << std::endl;
=======
		std::cout << "Data sent successfully! Bytes sent: " << bytesSent << std::endl;
>>>>>>> Stashed changes
	}
}

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
	//PrintGloveData(glovePtr);

	//Print all data 
	//glovePtr->PrintAllGloveData(glovePtr->gloveDataList[0]);

	// 获取设备的角色名
	//std::string name = glovePtr->gloveDataList[0].deviceName;
	//std::cout << "Device Name: " << name << std::endl;

	// 获取左手标定状态
	float l_calibrationStatus = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_L[0].value;
	//std::cout << "Left Hand Calibration Status: " << l_calibrationStatus << std::endl;

	// 获取左手关节数据（索引从 1 开始）
	float l0 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_L[1].value;  // 左手拇指第三关节俯仰角
	float l1 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_L[2].value;  // 左手拇指第二关节俯仰角
	float l2 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_L[3].value;  // 左手拇指第一关节俯仰角
	float l3 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_L[4].value;  // 左手拇指第一关节偏航角
	float l4 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_L[5].value;  // 左手食指第三关节俯仰角
	float l5 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_L[6].value;  // 左手食指第二关节俯仰角
	float l6 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_L[7].value;  // 左手食指第一关节俯仰角
	float l7 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_L[8].value;  // 左手食指第一关节偏航角
	float l8 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_L[9].value;  // 左手中指第三关节俯仰角
	float l9 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_L[10].value; // 左手中指第二关节俯仰角
	float l10 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_L[11].value; // 左手中指第一关节俯仰角
	float l11 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_L[12].value; // 左手中指第一关节偏航角
	float l12 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_L[13].value; // 左手无名指第三关节俯仰角
	float l13 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_L[14].value; // 左手无名指第二关节俯仰角
	float l14 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_L[15].value; // 左手无名指第一关节俯仰角
	float l15 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_L[16].value; // 左手无名指第一关节偏航角
	float l16 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_L[17].value; // 左手小指第三关节俯仰角
	float l17 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_L[18].value; // 左手小指第二关节俯仰角
	float l18 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_L[19].value; // 左手小指第一关节俯仰角
	float l19 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_L[20].value; // 左手小指第一关节偏航角
	float l20 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_L[21].value; // 左手拇指第一关节旋转角
	float l21 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_L[22].value; // 左手食指第一关节旋转角
	float l22 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_L[23].value; // 左手小指第一关节旋转角
	float l23 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_L[24].value; // 左手手势识别预留位
	float l24 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_L[25].value; // 左手 IMU 四元数 W
	float l25 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_L[26].value; // 左手 IMU 四元数 X
	float l26 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_L[27].value; // 左手 IMU 四元数 Y
	float l27 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_L[28].value; // 左手 IMU 四元数 Z

	// 获取右手标定状态
	float r_calibrationStatus = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_R[0].value;
	//std::cout << "Right Hand Calibration Status: " << r_calibrationStatus << std::endl;

	// 获取右手关节数据（索引从 1 开始）
	float r0 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_R[1].value;  // 右手拇指第三关节俯仰角
	float r1 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_R[2].value;  // 右手拇指第二关节俯仰角
	float r2 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_R[3].value;  // 右手拇指第一关节俯仰角
	float r3 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_R[4].value;  // 右手拇指第一关节偏航角
	float r4 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_R[5].value;  // 右手食指第三关节俯仰角
	float r5 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_R[6].value;  // 右手食指第二关节俯仰角
	float r6 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_R[7].value;  // 右手食指第一关节俯仰角
	float r7 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_R[8].value;  // 右手食指第一关节偏航角
	float r8 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_R[9].value;  // 右手中指第三关节俯仰角
	float r9 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_R[10].value; // 右手中指第二关节俯仰角
	float r10 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_R[11].value; // 右手中指第一关节俯仰角
	float r11 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_R[12].value; // 右手中指第一关节偏航角
	float r12 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_R[13].value; // 右手无名指第三关节俯仰角
	float r13 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_R[14].value; // 右手无名指第二关节俯仰角
	float r14 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_R[15].value; // 右手无名指第一关节俯仰角
	float r15 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_R[16].value; // 右手无名指第一关节偏航角
	float r16 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_R[17].value; // 右手小指第三关节俯仰角
	float r17 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_R[18].value; // 右手小指第二关节俯仰角
	float r18 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_R[19].value; // 右手小指第一关节俯仰角
	float r19 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_R[20].value; // 右手小指第一关节偏航角
	float r20 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_R[21].value; // 右手拇指第一关节旋转角
	float r21 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_R[22].value; // 右手食指第一关节旋转角
	float r22 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_R[23].value; // 右手小指第一关节旋转角
	float r23 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_R[24].value; // 右手手势识别预留位
	float r24 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_R[25].value; // 右手 IMU 四元数 W
	float r25 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_R[26].value; // 右手 IMU 四元数 X
	float r26 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_R[27].value; // 右手 IMU 四元数 Y
	float r27 = glovePtr->gloveDataList[0].handDatas.fingerJoints.fingerJoint_R[28].value; // 右手 IMU 四元数 Z

	// 发送手套数据
	SendGloveData(glovePtr);

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

	if (!InitializeSocket()) {
		std::cerr << "Failed to initialize socket." << std::endl;
		return -1;
	}

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
	while (true) {
		int a = 1;

		switch (a) 
		{

		case 1:
			OnlyPrintFrame();
			break;

		case 0:
			glove_sdk.Shutdown();
			CleanupSocket();
			std::cout << "Shutdown" << std::endl;
			return 0;
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

