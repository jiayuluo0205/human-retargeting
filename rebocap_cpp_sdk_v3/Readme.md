# CPP SDK

# example
直接运行 `.\rebocap_ws_sdk_cpp_example.exe`
注意，调试过程中， .dll 文件务必位于 exe 同级目录或者exe所在目录的bin目录下，或者将dll放到到环境变量path中，否则无法找到dll
另外，使用上也可以直接参考 UE 插件代码

# 基本使用示例
- 请参考 cmake 以及代码
# 数据结构
- trans:
  - List 3个浮点数据
  - 目前是屁股[pelvis]的位置
- pose24:
24个关节的四元数，参考example中的用法，其中 Pelvis 是根节点，节点位置图，可以参考[SMPL关节点位](https://blog.csdn.net/weixin_43822395/article/details/124378186)
```python
# 关节名称
REBOCAP_JOINT_NAMES = [
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
    "R_Hand"
]
```
- static_index
  - int 类型数据，-1表示没有检测到接触点，0,1,2 是左脚前掌的左中右， 3,4,5 是左脚脚后跟左中右， 6,7,8 是右脚前脚掌左中右  9,10,11 是右脚后脚本左中右
  - 对地接触点信息，目前只做了单点接触判定
