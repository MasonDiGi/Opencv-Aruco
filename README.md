# Opencv ArUco Detector
## Programs
- `aruco_pose.py` - Run to open the camera feed and detect markers.
    - Use the `-t` or `--type` parameter when running to select the type of marker (more info below)
- `aruco_pose_phone.py` - The same file as above, but without key presses, so it can run on the [pyto](https://www.pyto.app "Pyto  app") app on iPhones.
- `opencv_generate_aruco.py` - This file can be used to generate pngs of aruco tags with specified type and ids
    - `-o`, `--output`: The output path of the png file to be created
    - `-i`, `--id`: The id of the tag to be generated
    - `-t`, `--type`: The type of tag to be generated
- `convertTags.py` - Script to convert a directory of .jpg tags into a single printable pdf

## Folders
-  `tags/` - The tags I have generated so far. They are all named with their type and id for easy identification.
- `cals/` and `cals2/` - Images to be used for camera calibration. They are for my iPhone and webcam respecively.

## About ArUco Markers
There are many different types, or dictionaries of ArUco markers. Depending on what is selected, the markers will have different sizes and different amounts of possible ids. A full list of all the types can be seen in the `consts.py` file.
