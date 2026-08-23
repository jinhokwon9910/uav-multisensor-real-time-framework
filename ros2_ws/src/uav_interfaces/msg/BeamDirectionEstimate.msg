uint8 STATUS_OK=0
uint8 STATUS_NO_DETECTION=1
uint8 STATUS_INVALID_INPUT=2
uint8 STATUS_INFERENCE_ERROR=3

std_msgs/Header header
uint32 source_sequence
geometry_msgs/Vector3 direction_world
float32 azimuth_deg
float32 elevation_deg
float32 confidence
float32 processing_ms
uint8 status
