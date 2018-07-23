uint8 WORKSPACE_VOLUME = 0
uint8 CENTERED_ROI = 1

uint8 mode                              # workspace definition mode, either WORKSPACE_VOLUME or CENTERED_ROI
float64 x_min                           # minimum x bound of rectangular workspace (for WORKSPACE_VOLUME only)
float64 x_max                           # maximum x bound of rectangular workspace (for WORKSPACE_VOLUME only)
float64 y_min                           # minimum y bound of rectangular workspace (for WORKSPACE_VOLUME only)
float64 y_max                           # maximum y bound of rectangular workspace (for WORKSPACE_VOLUME only)
float64 z_min                           # minimum z bound of rectangular workspace (for WORKSPACE_VOLUME only)
float64 z_max                           # maximum z bound of rectangular workspace (for WORKSPACE_VOLUME only)
geometry_msgs/Point roiCenter           # workspace center point (for CENTERED_ROI only)
geometry_msgs/Vector3 roiDimensions     # workspace bounds (for CENTERED_ROI only)