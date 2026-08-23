// Generated from ros2_ws/src/uav_interfaces/msg/BeamDirectionEstimate.msg.
using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.UavInterfaces
{
    [Serializable]
    public class BeamDirectionEstimateMsg : Message
    {
        public const string k_RosMessageName = "uav_interfaces/BeamDirectionEstimate";
        public override string RosMessageName => k_RosMessageName;

        public const byte STATUS_OK = 0;
        public const byte STATUS_NO_DETECTION = 1;
        public const byte STATUS_INVALID_INPUT = 2;
        public const byte STATUS_INFERENCE_ERROR = 3;

        public Std.HeaderMsg header;
        public uint source_sequence;
        public Geometry.Vector3Msg direction_world;
        public float azimuth_deg;
        public float elevation_deg;
        public float confidence;
        public float processing_ms;
        public byte status;

        public BeamDirectionEstimateMsg()
        {
            header = new Std.HeaderMsg();
            source_sequence = 0;
            direction_world = new Geometry.Vector3Msg();
            azimuth_deg = 0.0f;
            elevation_deg = 0.0f;
            confidence = 0.0f;
            processing_ms = 0.0f;
            status = STATUS_INVALID_INPUT;
        }

        public BeamDirectionEstimateMsg(
            Std.HeaderMsg header,
            uint sourceSequence,
            Geometry.Vector3Msg directionWorld,
            float azimuthDeg,
            float elevationDeg,
            float confidence,
            float processingMs,
            byte status)
        {
            this.header = header;
            source_sequence = sourceSequence;
            direction_world = directionWorld;
            azimuth_deg = azimuthDeg;
            elevation_deg = elevationDeg;
            this.confidence = confidence;
            processing_ms = processingMs;
            this.status = status;
        }

        public static BeamDirectionEstimateMsg Deserialize(
            MessageDeserializer deserializer)
        {
            return new BeamDirectionEstimateMsg(deserializer);
        }

        BeamDirectionEstimateMsg(MessageDeserializer deserializer)
        {
            header = Std.HeaderMsg.Deserialize(deserializer);
            deserializer.Read(out source_sequence);
            direction_world = Geometry.Vector3Msg.Deserialize(deserializer);
            deserializer.Read(out azimuth_deg);
            deserializer.Read(out elevation_deg);
            deserializer.Read(out confidence);
            deserializer.Read(out processing_ms);
            deserializer.Read(out status);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(header);
            serializer.Write(source_sequence);
            serializer.Write(direction_world);
            serializer.Write(azimuth_deg);
            serializer.Write(elevation_deg);
            serializer.Write(confidence);
            serializer.Write(processing_ms);
            serializer.Write(status);
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
