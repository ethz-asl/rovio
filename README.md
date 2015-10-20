# README #

This repository contains the ROVIO (Robust Visual Inertial Odometry) framework. The code is open-source, but please remember that it is strongly coupled to on-going research and thus some parts are not fully mature yet. Furthermore, the code will also be subject to changes in the future which could include greater re-factoring of some parts.

### Install without opengl scene ###
Dependencies:
* ros
* kindr (https://github.com/ethz-asl/kindr)
* lightweight_filtering (https://bitbucket.org/bloesch/lightweight_filtering)

```
#!command

catkin build rovio --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Install with opengl scene ###
Additional dependencies: opengl, glut, glew (sudo apt-get install freeglut3-dev, sudo apt-get install libglew-dev)
```
#!command

catkin build rovio --cmake-args -DCMAKE_BUILD_TYPE=Release -DMAKE_SCENE=ON
```


### Further notes ###
* Camera matrix and distortion parameters should be provided by a yaml file or loaded through rosparam
* The cfg/rovio.info provides most parameters for rovio. The camera extrinsics qCM (quaternion from IMU to camera frame, JPL-convention) and MrMC (Translation between IMU and Camera expressed in the IMU frame) should also be set there. They are being estimated during runtime so only a rough guess should be sufficient.
