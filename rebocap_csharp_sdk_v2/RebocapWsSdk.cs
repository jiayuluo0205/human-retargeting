using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Text.RegularExpressions;

namespace RebocapWsSdk
{
#if UNITY_5_3_OR_NEWER
    using UVector3 = UnityEngine.Vector3;
#else
    public class UVector3
    {
        public float x = 0.0f;
        public float y = 0.0f;
        public float z = 0.0f;

        public UVector3(float x, float y, float z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }
        public UVector3(double x, double y, double z)
        {
            this.x = Convert.ToSingle(x);
            this.y = Convert.ToSingle(y);
            this.z = Convert.ToSingle(z);
        }
    }
#endif
    public delegate void PoseMsgCallback(PoseMsg msg, RebocapWsSdk self);

    public delegate void ExceptionCloseCallback(RebocapWsSdk self);

    public class PoseMsg
    {
        public float[] Trans = new float[3];
        public readonly float[,] Pose24 = new float[24, 4];
        public int StaticIndex;
        public DateTime Tp;

        public override  string ToString()
        {
            var builder = new StringBuilder();
            builder.Append("tp=" + Tp + " ");
            builder.Append("Trans=[" + Trans[0] + "," + Trans[1] + "," + Trans[2] + "] ");
            builder.Append("Pose24=[");
            builder.Append(",[" + Pose24[0, 0] + "," + Pose24[0, 1] + "," + Pose24[0, 2] + "," + Pose24[0, 3] + "]");
            for (var i = 1; i < 24; i++)
            {
                builder.Append(",[" + Pose24[i, 0] + "," + Pose24[i, 1] + "," + Pose24[i, 2] + "," + Pose24[i, 3] + "]");
            }
            builder.Append("] ");
            builder.Append("StaticIndex=" + StaticIndex);
            return builder.ToString();
        }
    }

    public enum CoordinateSpaceType{
        DefaultCoordinate = 0,
        UnityCoordinate = 1,
        BlenderCoordinate = 2,
        MayaCoordinate = 3,
        MaxCoordinate = 4,
        UECoordinate = 5
    }
    public enum RebocapBones
    {
        Hip = 0,
        LeftUpperLeg,
        RightUpperLeg,
        Spine,
        LeftLowerLeg,
        RightLowerLeg,
        Chest,
        LeftFoot,
        RightFoot,
        UpperChest,
        LeftToe,
        RightToe,
        Neck,
        LeftShoulder,
        RightShoulder,
        Head,
        LeftUpperArm,
        RightUpperArm,
        LeftLowerArm,
        RightLowerArm,
        LeftHand,
        RightHand,
        LeftMiddleProximal,
        RightMiddleProximal
    }
    
    public class RebocapWsSdk
    {
        public RebocapWsSdk(CoordinateSpaceType coordinateSpaceType=CoordinateSpaceType.UnityCoordinate)
        {
            _poseMsgCallbackDelegateInstance = PoseMsgCallbackRaw;
            _exceptionCloseCallbackDelegateInstance = ExceptionCloseCallbackRaw;
            _handle = rebocap_ws_sdk_new((int)coordinateSpaceType);
            rebocap_ws_sdk_set_pose_msg_callback(_handle, IntPtr.Zero, _poseMsgCallbackDelegateInstance);
            rebocap_ws_sdk_set_exception_close_callback(_handle, IntPtr.Zero, _exceptionCloseCallbackDelegateInstance);
        }

        ~RebocapWsSdk()
        {
            rebocap_ws_sdk_release(_handle);
        }

        public int Open(uint port, string name = "reborn_app", long uid = 0)
        {
            return rebocap_ws_sdk_open(_handle, port, name, (ulong) name.Length, uid);
        }

        public void Close()
        {
            rebocap_ws_sdk_close(_handle);
        }

        public void SetPoseMsgCallback(PoseMsgCallback callback)
        {
            _poseMsgCallback = callback;
        }

        public void SetExceptionCloseCallback(ExceptionCloseCallback callback)
        {
            _exceptionCloseCallback = callback;
        }

        public int GetLastMsg(ref PoseMsg poseMsg)
        {
            var cRaw = new PoseMsgCRaw();
            var ret = rebocap_ws_sdk_get_last_msg(_handle, ref cRaw);
            if (ret != 0)
            {
                return ret;
            }
            poseMsg.Trans = cRaw.trans;
            for (int i = 0, j = 0; i < 24; i++, j += 4)
            {
                poseMsg.Pose24[i, 0] = cRaw.quat[j];
                poseMsg.Pose24[i, 1] = cRaw.quat[j + 1];
                poseMsg.Pose24[i, 2] = cRaw.quat[j + 2];
                poseMsg.Pose24[i, 3] = cRaw.quat[j + 3];
            }

            poseMsg.StaticIndex = cRaw.static_index;
            poseMsg.Tp = new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc).AddMilliseconds(cRaw.tp);
            return 0;
        }

        /// <summary>
        /// Register avatar skeleton to rebocap host and auto calculate foot vertices for avatar movement, if can also
        /// specific manually defined foot vertices for better experience
        /// </summary>
        /// <param name="skeleton">rebocap bone skeleton data, Length must be 24, order reference RebocapBones</param>
        /// <param name="leftFoot">vertices bind on left foot</param>
        /// <param name="rightFoot">vertices bind on right foot</param>
        /// <param name="leftFootNorm">normals which correspond to left foot vertices</param>
        /// <param name="rightFootNorm">normals which correspond to right foot vertices</param>
        /// <param name="outputVertex">if this is empty, this function will auto calculate foot vertices
        ///                            if this Length is 12, we would use the vertices passed in, which is absolute position on a avatar</param>
        /// <param name="axes">target axis is[ x:left y:up z:backward], unity axis is:[ x:right y:up z:backward]
        /// for unity: this should be "-xyz"!
        /// note:coordinate is based on your self, ex: left is you left hand direction when you in T-pose</param>
        /// <returns></returns>
        public int CalculateFootVertexAndRegisterAvatar(List<UVector3> skeleton,
            List<UVector3> leftFoot, List<UVector3> rightFoot,
            List<UVector3> leftFootNorm, List<UVector3> rightFootNorm,
            List<UVector3> outputVertex, string axes)
        {
            axes = axes.ToLower();
            if (!(new Regex(@"^(-?[xyz])((-?[xyz]){2})?$")).IsMatch(axes))
            {
                Console.WriteLine($"axes:{axes} is not correct!");
                return -1;
            }
            
            Debug.Assert(skeleton.Count == 24, "skeleton.Count == 24");
            UVf3[] leftFootV = leftFoot.Select(v => new UVf3{x=v.x, y=v.y, z=v.z}).ToArray();
            UVf3[] rightFootV = rightFoot.Select(v => new UVf3{x=v.x, y=v.y, z=v.z}).ToArray();
            UVf3[] leftFootNormV = leftFootNorm.Select(v => new UVf3{x=v.x, y=v.y, z=v.z}).ToArray();
            UVf3[] rightFootNormV = rightFootNorm.Select(v => new UVf3{x=v.x, y=v.y, z=v.z}).ToArray();
            UVf3[] skeletonV = skeleton.Select(v => new UVf3{x=v.x, y=v.y, z=v.z}).ToArray();
            UVf3[] output = outputVertex.Count == 12 ? outputVertex.Select(v => new UVf3{x=v.x, y=v.y, z=v.z}).ToArray() : new UVf3[12];
            int res = rebocap_ws_sdk_calculate_foot_vertex(_handle, 
                leftFootV, leftFootV.Length,
                rightFootV, rightFootV.Length,
                leftFootNormV, leftFootNormV.Length,
                rightFootNormV, rightFootNormV.Length,
                skeletonV, axes,
                output, outputVertex.Count == 12 ? 0 : 1
                );

            if (outputVertex.Count != 12)
            {
                outputVertex.Clear();
                outputVertex.Capacity = 12;
                for (int i = 0; i < output.Length; i++) outputVertex.Add(new UVector3(output[i].x, output[i].y, output[i].z));
            }

            return res;
        }

        private void PoseMsgCallbackRaw(ref PoseMsgCRaw poseMsgRaw, IntPtr extra)
        {
            var poseMsg = new PoseMsg
            {
                Trans = poseMsgRaw.trans
            };
            for (int i = 0, j = 0; i < 24; i++, j += 4)
            {
                poseMsg.Pose24[i, 0] = poseMsgRaw.quat[j];
                poseMsg.Pose24[i, 1] = poseMsgRaw.quat[j + 1];
                poseMsg.Pose24[i, 2] = poseMsgRaw.quat[j + 2];
                poseMsg.Pose24[i, 3] = poseMsgRaw.quat[j + 3];
            }

            poseMsg.StaticIndex = poseMsgRaw.static_index;
            poseMsg.Tp = new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc).AddMilliseconds(poseMsgRaw.tp);
            _poseMsgCallback?.Invoke(poseMsg, this);
        }

        private void ExceptionCloseCallbackRaw(IntPtr extra)
        {
            _exceptionCloseCallback?.Invoke(this);
        }

        private readonly IntPtr _handle;
        private PoseMsgCallback _poseMsgCallback;
        private ExceptionCloseCallback _exceptionCloseCallback;
        private PoseMsgCallbackDelegate _poseMsgCallbackDelegateInstance;
        private ExceptionCloseCallbackDelegate _exceptionCloseCallbackDelegateInstance;

        [StructLayout(LayoutKind.Sequential)]
        private struct PoseMsgCRaw
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
            public float[] trans;

            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 96)]
            public float[] quat;

            public sbyte static_index;
            public ulong tp;
        }

        [StructLayout(LayoutKind.Sequential)]
        private struct UVf3
        {
            public float x;
            public float y;
            public float z;
        }
        
        private delegate void PoseMsgCallbackDelegate(ref PoseMsgCRaw poseMsg, IntPtr extra);

        private delegate void ExceptionCloseCallbackDelegate(IntPtr extra);
        
        [DllImport("rebocap_ws_sdk.dll", CallingConvention = CallingConvention.StdCall,
            CharSet = CharSet.Unicode, EntryPoint = "rebocap_ws_sdk_new")]
        private static extern IntPtr rebocap_ws_sdk_new(int coordinateType);

        [DllImport("rebocap_ws_sdk.dll", CallingConvention = CallingConvention.StdCall,
            CharSet = CharSet.Unicode, EntryPoint = "rebocap_ws_sdk_release")]
        private static extern void rebocap_ws_sdk_release(IntPtr handle);
        
        [DllImport("rebocap_ws_sdk.dll", CallingConvention = CallingConvention.StdCall,
            CharSet = CharSet.Ansi, EntryPoint = "rebocap_ws_sdk_open")]
        private static extern int rebocap_ws_sdk_open(IntPtr handle, UInt32 port, String name, UInt64 nameLen, Int64 uid);
        
        [DllImport("rebocap_ws_sdk.dll", CallingConvention = CallingConvention.StdCall,
            CharSet = CharSet.Ansi, EntryPoint = "rebocap_ws_sdk_close")]

        private static extern int rebocap_ws_sdk_close(IntPtr handle);
        
        [DllImport("rebocap_ws_sdk.dll", CallingConvention = CallingConvention.StdCall,
            CharSet = CharSet.Ansi, EntryPoint = "rebocap_ws_sdk_set_pose_msg_callback")]
        
        private static extern void rebocap_ws_sdk_set_pose_msg_callback(IntPtr handle, IntPtr extra, PoseMsgCallbackDelegate func);
        
        [DllImport("rebocap_ws_sdk.dll", CallingConvention = CallingConvention.StdCall,
            CharSet = CharSet.Ansi, EntryPoint = "rebocap_ws_sdk_set_exception_close_callback")]
        
        private static extern void rebocap_ws_sdk_set_exception_close_callback(IntPtr handle, IntPtr extra, ExceptionCloseCallbackDelegate func);
        
        [DllImport("rebocap_ws_sdk.dll", CallingConvention = CallingConvention.StdCall,
            CharSet = CharSet.Ansi, EntryPoint = "rebocap_ws_sdk_get_last_msg")]
        
        private static extern int rebocap_ws_sdk_get_last_msg(IntPtr handle, ref PoseMsgCRaw extra);
        
        [DllImport("rebocap_ws_sdk.dll", CallingConvention = CallingConvention.StdCall,
            CharSet = CharSet.Ansi, EntryPoint = "rebocap_ws_sdk_calculate_foot_vertex")]
        private static extern int rebocap_ws_sdk_calculate_foot_vertex(IntPtr handle,
            UVf3[] leftFootV, int sleftFootV,
            UVf3[] rightFootV, int srightFootV,
            UVf3[] leftFootNormV, int sleftFootNormV,
            UVf3[] rightFootNormV, int srightFootNormV,
            UVf3[] skeleton, string coordinate_change,
            [Out] UVf3[] result, int calculate_vertex
            );
    }
}