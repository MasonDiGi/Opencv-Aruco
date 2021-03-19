import math

def init():
    import pyrealsense2 as rs

    # Set up config to get data from camera
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.pose)
    profile = cfg.resolve(pipe)
    dev = profile.get_device()
    tm2 = dev.as_tm2()

    # Initialize stream
    pipe.start(cfg)
    return pipe


def getPose(pipe):
    import matrixfunctions as mf
    frames = pipe.wait_for_frames()

    # Fetch pose frame
    pose = frames.get_pose_frame()
    if pose:
        # Print some of the pose data to the terminal
        data = pose.get_pose_data()
        xPos = data.translation.x
        yPos = data.translation.y
        zPos = data.translation.z
        x = data.rotation.x
        y = data.rotation.y
        z = data.rotation.z
        w = data.rotation.w

        # Quaternion to Euler Angle conversions (wikipedia)
        # Used for  yaw, but swapped axes so it is around the y axis
        angle = math.atan2(2 * ((w * y) + (z * x)), 1 - 2 * ((x * x) + (y * y)))  # angle in radians (pi to -pi)
        odomToRealsense = mf.poseToMatrix(z, y,angle)
        realsenseToRobot = mf.poseToMatrix(21.35124, 13.871702, math.pi/2)

        return True, odomToRealsense*realsenseToRobot
    else:
        return False, 0, 0, 0, 0

