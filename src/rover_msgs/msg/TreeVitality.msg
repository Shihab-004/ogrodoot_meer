# TreeVitality.msg
# ROS 2 message definition for Tree Vitality Detection results
# NAGC 2026 Autonomous Navigation Mission

std_msgs/Header header

# Classification verdict: "ALIVE", "DEAD", or "UNKNOWN"
string vitality

# Confidence score between 0.0 and 1.0
float32 confidence

# Detection status
bool tree_detected

# Proportion of canopy/foliage area within tree bounding box (0.0 to 1.0)
float32 foliage_ratio

# Vegetation index metric (e.g. VARI or ExG normalized between -1.0 and 1.0)
float32 greenness_index

# Bounding box in pixels: [x_min, y_min, width, height]
int32[4] bounding_box

# Detailed diagnostic explanation of visual characteristics
string details
