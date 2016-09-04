# Contributors

* Lukas Rambold
* Pascal Fritzen 

# Repo Structure

The repo consists of two folders, namely **development** and **production**.

The development consists of various "hello world"-like examples on the open cv basics (e.g. reading the computer's camera), doing basic image processing (e.g. detecting objects) and steering the drone.

The production folder contains the main python file to fly the drone autonomously and steer it through the parkour as well as a file with basic constants and global parameters. It also contains a folder for taking snapshots of the drone's camera for later analysis.

# Approach

## Manual Movement

Similar to one of the demos, the drone can be controlled manually at all time by pressing keys (taking off, landing, hovering, left, right, up, down, turn left, turn right). Other keyboard shortcuts include resetting the drone, enable/disable autonomous flight, calibrating the camera (see below) & taking snapshots of the drone's camera.

## Autonomous Flight

Once enabled, the drone flies autonomously (i.e. follows a predefined object) we used the following software packages:

* ARDrone
* OpenCV

### Calibration

Before flying autonomously the drone should be calibrated in order to account for various light conditions. Calibration is performed by first pointing the drone's front camera towards the object (more specifically the color) to follow and then pressing the keyboard shortcut for calibrating. During calibration the drone takes the color values of the 10x10 center pixels, converts them into HSV, applies a gaussian blur and returns a bound of +/- 5 value for this color.

### Image Processing

![Screen Shot 2016-08-12 at 16.18.26.png](https://api.nuclino.com/api/files/54f2d230-3de3-414d-b652-acbb92497c51)
From its front camera, the drone detects an object to follow. This is achieved as follows:

1. The drone processes the image of its front camera by first converting it into the HSV color space and afterwards applying a median blur.
2. In case of previous calibration, the image is masked with a certain color (a pre-defined shade of red if not calibrated). Afterwards another, bigger median blur is applied. At this point, the camera image (as on the right side of the picture) is turned into a processed image (see on the left side of the picture) showing only the pre-calibrated color in white.
3. The idea is then to identify the bounds of the object to follow. For that, openCV's ability to find all contours within an image is used and then select the biggest contour as it is assumed that this would be the object to follow. Once found, the objects bounding box and center are returned.

### Steering

Now, that the object to track was identified, the drone adjusts height, orientation and distance by altering its horizontal and vertical position and forward/backward speed.

Height and orientation: this is done by calculating the horizontal and vertical offset of the object's center from the drone's camera center. As displayed in the right picture above, the blue circle depicts the center of the object which should ideally be at the entire camera's center.

Distance: in order to estimate the distance, it is calculated how much of the screen is filled with the object. Ideally, the object should be of the size of the green rectangle (see rith picture above again). The size of the rectangle can, as many other paramters, be played around with to achieve even more accurate results.

## Miscellaneous

On startup, a live camera feed is displayed, overlaid with the drone's height, orientation, battery status, x/y/z speed etc..

To access the drone's navigation data we wrote a little wrapper called ARDroneWrapper.

Screenshots can be captured by pressing the "c" key for later analysis.

## 

# 