# C# SDK

# c#运行模式
- 仅支持x64模式，不支持默认Any CPU模式

# example
- vs2022 打开当前项目即可
- 注意程序工作目录指向rebocap_ws_sdk.dll所在目录

# 基本使用示例
```csharp
var sdk = new RebocapWsSdk.RebocapWsSdk();
// 设置姿态回调
sdk.SetPoseMsgCallback(PoseMsgCallback);
// 设置异常断开回调
sdk.SetExceptionCloseCallback(ExceptionCloseCallback);
// 连接rebocap
var openRes = sdk.Open(7690);
// 检查连接返回值
if (openRes == 0)
{
    System.Console.WriteLine("连接成功");
}
else
{
    System.Console.WriteLine("连接失败" + openRes);
    switch (openRes)
    {
        case 1:
            System.Console.WriteLine("连接状态错误");
            break;
        case 2:
            System.Console.WriteLine("连接失败");
            break;
        case 3:
            System.Console.WriteLine("认证失败");
            break;
        default:
            System.Console.WriteLine("未知错误");
            break;
    }
    return;
}
Thread.Sleep(10000);
var poseMsg = new PoseMsg();
// 获取最近一帧数据
var getLastMsgRet = sdk.GetLastMsg(ref poseMsg);
if (getLastMsgRet != 0)
{
    System.Console.WriteLine("GetLastMsg 失败" + getLastMsgRet);
}
else
{
    System.Console.WriteLine("GetLastMsg 成功");
    System.Console.WriteLine(poseMsg);
}
// 关闭连接
sdk.Close();
```
# 数据结构
- Trans:
    - List 3个浮点数据
    - 目前是屁股[pelvis]的位置
- Pose24:
  24个关节的四元数，参考example中的用法，其中 Pelvis 是根节点，节点位置图，可以参考[SMPL关节点位](https://blog.csdn.net/weixin_43822395/article/details/124378186)
```csharp
# 关节名称
public static string[] RebocapJointNames =
{
    "Pelvis",
    "L_Hip",
    "R_Hip",
    "Spine1",
    "L_Knee",
    "R_Knee",
    "Spine2",
    "L_Ankle",
    "R_Ankle",
    "Spine3",
    "L_Foot",
    "R_Foot",
    "Neck",
    "L_Collar",
    "R_Collar",
    "Head",
    "L_Shoulder",
    "R_Shoulder",
    "L_Elbow",
    "R_Elbow",
    "L_Wrist",
    "R_Wrist",
    "L_Hand",
    "R_Hand",
};
```
- static_index
    - int 类型数据，-1表示没有检测到接触点，0,1,2 是左脚前掌的左中右， 3,4,5 是左脚脚后跟左中右， 6,7,8 是右脚前脚掌左中右  9,10,11 是右脚后脚本左中右
    - 对地接触点信息，目前只做了单点接触判定
