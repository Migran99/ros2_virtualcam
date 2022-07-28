# ros2_virtualcam
ROS 2 package to create a virtual camera from a image topic.

## How to use:
It is necessary to install `pyvirtualcam`:

https://github.com/letmaik/pyvirtualcam

Then you may need to add your user to the video group:
```
sudo usermod -a -G video <username>
```

Log out and log in. Make sure you have set up your device on number 3 o change it in the code:

```
sudo modprobe v4l2loopback video_nr=3
```

Check with:
```
vv4l2-ctl --list-devices
```


You may need to install `vv4l2-ctl`:
```
sudo apt install vv4l2-ctl
```

Now you should be able to open the video stream as any other camera, for example with opencv `VideoCapture`.