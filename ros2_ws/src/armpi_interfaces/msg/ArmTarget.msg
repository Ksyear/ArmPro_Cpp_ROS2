std_msgs/Header header
bool active        # false = 현재 위치 유지(새 목표 아님)
float64 x          # m, armpi_arm_base (REP-103: x 앞, y 왼쪽, z 위)
float64 y
float64 z
float64 pitch      # rad, 0 = 수평 앞, -pi/2 = 아래
float64 gripper    # 0 = 열림, 1 = 닫힘
