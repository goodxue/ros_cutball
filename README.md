# ros_cutball

Using ROS and ZED to implement the UAV to judge whether a balloon is falling and where to track and cut it.
In my edition 1.0.0, we need firstly find 4 positions for 4 quadrants, and then the program determines which balloon is falling in which quadrant. Then the drone would move to that point and return.

## Purpose
For a compitation named *Air Robot Cut Fruit* in [*RoboCup China Open*](http://www.rcj.org.cn/index.php/race?catid=3).

## Dependence
* ROS
* zed-ros-wrapper
