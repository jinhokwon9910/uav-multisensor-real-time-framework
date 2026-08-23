using System;
using System.Collections;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.UavInterfaces;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

namespace UavBeamBridge
{
    public sealed class SensorFrameCoordinator : MonoBehaviour
    {
        [Header("Scene references")]
        [SerializeField] Camera sensorCamera;
        [SerializeField] Transform uavTransform;

        [Header("ROS topics")]
        [SerializeField] string sensorTopic = "/unity/sensor_frame";
        [SerializeField] string estimateTopic = "/beam/direction_estimate";

        [Header("Capture")]
        [SerializeField, Min(1)] int imageWidth = 1920;
        [SerializeField, Min(1)] int imageHeight = 1080;
        [SerializeField, Min(0.0f)] float captureIntervalSeconds = 0.0f;
        [SerializeField, Min(0.1f)] float responseTimeoutSeconds = 10.0f;
        [SerializeField] bool pauseSimulationWhileWaiting = true;

        [Header("Noisy UAV orientation (x=pitch, y=yaw, z=roll)")]
        [SerializeField] Vector3 noiseStandardDeviationDegrees =
            new Vector3(0.05f, 0.125f, 0.04f);

        [Header("Visualization")]
        [SerializeField] bool drawEstimate = true;
        [SerializeField, Min(0.1f)] float rayLength = 300.0f;

        [Header("Runtime state (read only)")]
        [SerializeField] bool awaitingResponse;
        [SerializeField] uint lastPublishedSequence;
        [SerializeField] uint lastAcceptedSequence;
        [SerializeField] float lastRoundTripMs;
        [SerializeField] float lastPythonProcessingMs;
        [SerializeField] uint successfulResponseCount;
        [SerializeField] string lastResult = "not started";

        static readonly DateTime UnixEpoch =
            new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);

        ROSConnection ros;
        Coroutine publishLoop;
        uint nextSequence;
        double sentAtRealtime;
        float timeScaleBeforePause = 1.0f;
        bool ownsTimeScalePause;

        public bool AwaitingResponse => awaitingResponse;
        public uint LastPublishedSequence => lastPublishedSequence;
        public uint LastAcceptedSequence => lastAcceptedSequence;
        public uint SuccessfulResponseCount => successfulResponseCount;
        public float LastRoundTripMs => lastRoundTripMs;
        public float LastPythonProcessingMs => lastPythonProcessingMs;
        public string LastResult => lastResult;

        /// <summary>
        /// Supplies the two scene references before Start runs. This keeps the
        /// coordinator reusable without depending on a project-specific scene.
        /// </summary>
        public bool TryConfigure(Camera camera, Transform uav)
        {
            if (camera == null || uav == null)
            {
                return false;
            }

            sensorCamera = camera;
            uavTransform = uav;
            return true;
        }

        void Start()
        {
            if (!ValidateConfiguration())
            {
                enabled = false;
                return;
            }

            ros = ROSConnection.GetOrCreateInstance();
            ros.RegisterPublisher<SensorFrameMsg>(sensorTopic, queue_size: 1);
            ros.Subscribe<BeamDirectionEstimateMsg>(estimateTopic, OnEstimate);
            publishLoop = StartCoroutine(PublishLoop());
        }

        void OnDisable()
        {
            if (publishLoop != null)
            {
                StopCoroutine(publishLoop);
                publishLoop = null;
            }

            awaitingResponse = false;
            RestoreSimulationTime();
        }

        bool ValidateConfiguration()
        {
            if (sensorCamera == null || uavTransform == null)
            {
                Debug.LogError(
                    "SensorFrameCoordinator requires Sensor Camera and UAV Transform.",
                    this);
                return false;
            }

            if (string.IsNullOrWhiteSpace(sensorTopic)
                || string.IsNullOrWhiteSpace(estimateTopic))
            {
                Debug.LogError("ROS topic names cannot be empty.", this);
                return false;
            }

            return true;
        }

        IEnumerator PublishLoop()
        {
            while (enabled)
            {
                // Camera.Render() below performs an explicit sensor render, so
                // waiting one update is sufficient and also works when the
                // Editor is showing Scene view instead of Game view.
                yield return null;
                PublishSnapshot();

                while (awaitingResponse)
                {
                    double elapsed = Time.realtimeSinceStartupAsDouble - sentAtRealtime;
                    if (elapsed >= responseTimeoutSeconds)
                    {
                        lastResult = $"timeout: seq={lastPublishedSequence}";
                        Debug.LogWarning(
                            $"Beam estimate timeout for sequence {lastPublishedSequence}.",
                            this);
                        awaitingResponse = false;
                        RestoreSimulationTime();
                        break;
                    }

                    yield return null;
                }

                if (captureIntervalSeconds > 0.0f)
                {
                    yield return new WaitForSeconds(captureIntervalSeconds);
                }
            }
        }

        void PublishSnapshot()
        {
            uint sequence = nextSequence++;
            byte[] png = CapturePng(sensorCamera, imageWidth, imageHeight);
            Quaternion noisyUavOrientation = SampleNoisyUavOrientation();
            TimeMsg stamp = UtcNowToRosTime();

            HeaderMsg frameHeader = new HeaderMsg(stamp, "unity_world");
            HeaderMsg imageHeader = new HeaderMsg(stamp, "unity_camera_optical");
            CompressedImageMsg image = new CompressedImageMsg(
                imageHeader,
                "rgb8; png compressed rgb8",
                png);
            SensorFrameMsg frame = new SensorFrameMsg(
                frameHeader,
                sequence,
                image,
                ToQuaternionMsg(noisyUavOrientation),
                ToQuaternionMsg(sensorCamera.transform.localRotation),
                sensorCamera.fieldOfView);

            lastPublishedSequence = sequence;
            awaitingResponse = true;
            sentAtRealtime = Time.realtimeSinceStartupAsDouble;
            PauseSimulationTime();

            try
            {
                ros.Publish(sensorTopic, frame);
                lastResult = $"published: seq={sequence}, png={png.Length} bytes";
            }
            catch (Exception exception)
            {
                awaitingResponse = false;
                RestoreSimulationTime();
                lastResult = $"publish failed: {exception.Message}";
                Debug.LogException(exception, this);
            }
        }

        void OnEstimate(BeamDirectionEstimateMsg estimate)
        {
            if (!awaitingResponse)
            {
                Debug.LogWarning(
                    $"Ignoring unsolicited beam estimate {estimate.source_sequence}.",
                    this);
                return;
            }

            if (estimate.source_sequence != lastPublishedSequence)
            {
                Debug.LogWarning(
                    $"Ignoring stale beam estimate {estimate.source_sequence}; "
                    + $"waiting for {lastPublishedSequence}.",
                    this);
                return;
            }

            lastRoundTripMs = (float)(
                (Time.realtimeSinceStartupAsDouble - sentAtRealtime) * 1000.0);
            lastPythonProcessingMs = estimate.processing_ms;
            lastAcceptedSequence = estimate.source_sequence;
            awaitingResponse = false;
            RestoreSimulationTime();

            if (estimate.status != BeamDirectionEstimateMsg.STATUS_OK)
            {
                lastResult =
                    $"rejected: seq={estimate.source_sequence}, status={estimate.status}";
                Debug.LogWarning(lastResult, this);
                return;
            }

            if (!TryReadUnitDirection(estimate.direction_world, out Vector3 direction))
            {
                lastResult = $"invalid direction: seq={estimate.source_sequence}";
                Debug.LogWarning(lastResult, this);
                return;
            }

            lastResult =
                $"ok: seq={estimate.source_sequence}, "
                + $"az={estimate.azimuth_deg:F2}, el={estimate.elevation_deg:F2}, "
                + $"rtt={lastRoundTripMs:F1} ms";
            successfulResponseCount = unchecked(successfulResponseCount + 1);
            Debug.Log(lastResult, this);

            if (drawEstimate)
            {
                Debug.DrawRay(
                    sensorCamera.transform.position,
                    direction * rayLength,
                    Color.cyan,
                    Mathf.Max(captureIntervalSeconds, 0.1f));
            }
        }

        Quaternion SampleNoisyUavOrientation()
        {
            Vector3 euler = uavTransform.eulerAngles;
            euler.x += GaussianNoise(noiseStandardDeviationDegrees.x);
            euler.y += GaussianNoise(noiseStandardDeviationDegrees.y);
            euler.z += GaussianNoise(noiseStandardDeviationDegrees.z);
            return Quaternion.Euler(euler);
        }

        static float GaussianNoise(float standardDeviation)
        {
            if (standardDeviation <= 0.0f)
            {
                return 0.0f;
            }

            float u1 = 1.0f - UnityEngine.Random.value;
            float u2 = 1.0f - UnityEngine.Random.value;
            float normal = Mathf.Sqrt(-2.0f * Mathf.Log(u1))
                * Mathf.Sin(2.0f * Mathf.PI * u2);
            return standardDeviation * normal;
        }

        static byte[] CapturePng(Camera camera, int width, int height)
        {
            RenderTexture previousActive = RenderTexture.active;
            RenderTexture previousTarget = camera.targetTexture;
            RenderTexture renderTexture = RenderTexture.GetTemporary(
                width,
                height,
                24,
                RenderTextureFormat.ARGB32);
            Texture2D texture = new Texture2D(width, height, TextureFormat.RGB24, false);

            try
            {
                camera.targetTexture = renderTexture;
                RenderTexture.active = renderTexture;
                camera.Render();
                texture.ReadPixels(new Rect(0, 0, width, height), 0, 0);
                texture.Apply(false);
                return texture.EncodeToPNG();
            }
            finally
            {
                camera.targetTexture = previousTarget;
                RenderTexture.active = previousActive;
                RenderTexture.ReleaseTemporary(renderTexture);
                Destroy(texture);
            }
        }

        static QuaternionMsg ToQuaternionMsg(Quaternion value)
        {
            return new QuaternionMsg(value.x, value.y, value.z, value.w);
        }

        static TimeMsg UtcNowToRosTime()
        {
            long ticksSinceEpoch = (DateTime.UtcNow - UnixEpoch).Ticks;
            int seconds = checked((int)(ticksSinceEpoch / TimeSpan.TicksPerSecond));
            uint nanoseconds = (uint)(
                (ticksSinceEpoch % TimeSpan.TicksPerSecond) * 100L);
            return new TimeMsg(seconds, nanoseconds);
        }

        static bool TryReadUnitDirection(Vector3Msg message, out Vector3 direction)
        {
            direction = new Vector3(
                (float)message.x,
                (float)message.y,
                (float)message.z);
            if (!IsFinite(direction.x)
                || !IsFinite(direction.y)
                || !IsFinite(direction.z)
                || direction.sqrMagnitude < 1.0e-8f)
            {
                return false;
            }

            direction.Normalize();
            return true;
        }

        static bool IsFinite(float value)
        {
            return !float.IsNaN(value) && !float.IsInfinity(value);
        }

        void PauseSimulationTime()
        {
            if (!pauseSimulationWhileWaiting || ownsTimeScalePause)
            {
                return;
            }

            timeScaleBeforePause = Time.timeScale;
            Time.timeScale = 0.0f;
            ownsTimeScalePause = true;
        }

        void RestoreSimulationTime()
        {
            if (!ownsTimeScalePause)
            {
                return;
            }

            Time.timeScale = timeScaleBeforePause;
            ownsTimeScalePause = false;
        }
    }
}
