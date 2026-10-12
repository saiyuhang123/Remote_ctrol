# Native camera capture: x, y, width, height within the video area, excluding added captions.
std_msgs/Header header
uint32 channel
uint32 face_id
uint32 target_id
uint32 score                   # 0..100; 255 means unknown
uint32 camera_time_raw         # SDK packed camera timestamp; header uses host receipt time
bool end_mark
bool rect_valid
uint32 background_width
uint32 background_height
uint32 coordinate_width
uint32 coordinate_height
float32[4] rect_normalized
int32[4] rect_pixels
string face_path
string background_path
string annotated_path
string metadata_path
