std_msgs/Header header
float64 extruder_distance    # metres, signed: +ve = grout line is to the robot's left
float64 angle_difference     # radians, signed heading error versus the grout line tangent
bool    valid                # false when the camera has no grout line beneath it
