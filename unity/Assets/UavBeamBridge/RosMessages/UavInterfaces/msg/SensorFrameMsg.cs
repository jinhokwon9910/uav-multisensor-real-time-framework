// Generated from ros2_ws/src/uav_interfaces/msg/SensorFrame.msg.
using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.UavInterfaces
{
    [Serializable]
    public class SensorFrameMsg : Message
    {
        public const string k_RosMessageName = "uav_interfaces/SensorFrame";
        public override string RosMessageName => k_RosMessageName;

        public Std.HeaderMsg header;
        public uint sequence;
        public Sensor.CompressedImageMsg camera_image;
        public Geometry.QuaternionMsg uav_orientation_noisy;
        public Geometry.QuaternionMsg camera_orientation_local;
        public float vertical_fov_deg;

        public SensorFrameMsg()
        {
            header = new Std.HeaderMsg();
            sequence = 0;
            camera_image = new Sensor.CompressedImageMsg();
            uav_orientation_noisy = new Geometry.QuaternionMsg();
            camera_orientation_local = new Geometry.QuaternionMsg();
            vertical_fov_deg = 0.0f;
        }

        public SensorFrameMsg(
            Std.HeaderMsg header,
            uint sequence,
            Sensor.CompressedImageMsg cameraImage,
            Geometry.QuaternionMsg uavOrientationNoisy,
            Geometry.QuaternionMsg cameraOrientationLocal,
            float verticalFovDeg)
        {
            this.header = header;
            this.sequence = sequence;
            camera_image = cameraImage;
            uav_orientation_noisy = uavOrientationNoisy;
            camera_orientation_local = cameraOrientationLocal;
            vertical_fov_deg = verticalFovDeg;
        }

        public static SensorFrameMsg Deserialize(MessageDeserializer deserializer)
        {
            return new SensorFrameMsg(deserializer);
        }

        SensorFrameMsg(MessageDeserializer deserializer)
        {
            header = Std.HeaderMsg.Deserialize(deserializer);
            deserializer.Read(out sequence);
            camera_image = Sensor.CompressedImageMsg.Deserialize(deserializer);
            uav_orientation_noisy = Geometry.QuaternionMsg.Deserialize(deserializer);
            camera_orientation_local = Geometry.QuaternionMsg.Deserialize(deserializer);
            deserializer.Read(out vertical_fov_deg);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(header);
            serializer.Write(sequence);
            serializer.Write(camera_image);
            serializer.Write(uav_orientation_noisy);
            serializer.Write(camera_orientation_local);
            serializer.Write(vertical_fov_deg);
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}
