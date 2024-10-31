using System.Collections.Generic;
using System.Diagnostics;
using System.Threading;
using System.IO;
using Newtonsoft.Json;
using RebocapWsSdk;

namespace RebocapWsSdkExample
{
    internal abstract class Program
    {
        private static uint _cnt = 0;
        private static void PoseMsgCallback(PoseMsg msg, RebocapWsSdk.RebocapWsSdk self)
        {
            if (_cnt % 60 == 0)
            {
                System.Console.WriteLine(msg);
            }
            _cnt++;
        }

        private static void ExceptionCloseCallback(RebocapWsSdk.RebocapWsSdk self)
        {
            System.Console.WriteLine("ExceptionCloseCallback");
        }
        
        public class DemoDataJsonParser
        {
            public Dictionary<int, List<List<float>>> left_foot_vertex { get; set; }
            public Dictionary<int, List<List<float>>> right_foot_vertex { get; set; }
            public List<List<float>> skeleton { get; set; }
        }

        static void ParseJson(string filePath, out List<UVector3> all_left_vertex, out List<UVector3> all_left_normals,
            out List<UVector3> all_right_vertex, out List<UVector3> all_right_normals, out List<UVector3> skeletons)
        {
            all_left_vertex = new List<UVector3>();
            all_left_normals = new List<UVector3>();
            all_right_vertex = new List<UVector3>();
            all_right_normals = new List<UVector3>();
            skeletons = new List<UVector3>();

            string json = File.ReadAllText(filePath);
            DemoDataJsonParser data = JsonConvert.DeserializeObject<DemoDataJsonParser>(json);

            foreach (var item in data.left_foot_vertex.Values)
            {
                all_left_vertex.Add(new UVector3(item[0][0], item[0][1], item[0][2]));
                all_left_normals.Add(new UVector3(item[1][0], item[1][1], item[1][2]));
            }

            foreach (var item in data.right_foot_vertex.Values)
            {
                all_right_vertex.Add(new UVector3(item[0][0], item[0][1], item[0][2]));
                all_right_normals.Add(new UVector3(item[1][0], item[1][1], item[1][2]));
            }
            foreach (var item in data.skeleton)
            {
                skeletons.Add(new UVector3(item[0], item[1], item[2]));
            }
        }

        static void TestFootVertex(RebocapWsSdk.RebocapWsSdk sdk)
        {
            List<UVector3> left_vertex;
            List<UVector3> left_normals;
            List<UVector3> right_vertex;
            List<UVector3> right_normals;
            List<UVector3> skeletons;
            string dataPath = "avatar_parsed_data.json";
            ParseJson(dataPath, out left_vertex, out left_normals,
                out right_vertex, out right_normals, out skeletons);
            // load data axis is:  x: right  y: up  z: forward   target axis is:  x: left  y: up  z: backward
            
            List<RebocapWsSdk.UVector3> output = new List<UVector3>();
            var res = sdk.CalculateFootVertexAndRegisterAvatar(skeletons, left_vertex, right_vertex, 
                left_normals, right_normals, output, "-xy-z");
            System.Console.WriteLine(output[0].x);
            res = 0;
        }
        
        public static void Main(string[] args)
        {
            var sdk = new RebocapWsSdk.RebocapWsSdk();
            // 设置姿态回调
            sdk.SetPoseMsgCallback(PoseMsgCallback);
            // 设置异常断开回调
            sdk.SetExceptionCloseCallback(ExceptionCloseCallback);
            // 连接rebocap
            var openRes = sdk.Open(7690);
            
            TestFootVertex(sdk);
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
            Thread.Sleep(60000);
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
        }
    }
}