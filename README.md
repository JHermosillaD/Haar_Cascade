# Haar cascade face detector
ROS package for face detection. The package subscribes to an image and uses the OpenCV haar cascade for detection. The image containing the bounding box and its coordinates is published in real time through different topics.
## Set-up
### Requirements
- [x] ROS melodic or higher
- [x] OpenCV
### Installation
Clone the repository to the workspace source path.
```
user@hostname:~/workspace/src$ git clone https://github.com/JHermosillaD/haar_cascade.git
```
Compile the package.
```
user@hostname:~/workspace$ catkin_make
```
## Usage
Edit file `launch/face.launch` by the appropriate value of parameter `/camera_topic` according to the name of the used image topic :bangbang:

Run the launcher.
```
user@hostname:~/workspace$ roslaunch haar_cascade face.launch
```
## Visualization
The image containing the bounding box can be displayed in Rviz, the coordinates through topic `/haar_cascade/bounding_box`.

<img width="605" height="360" src="/haar.png"> 
