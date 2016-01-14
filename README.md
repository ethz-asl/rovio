# README
This repository contains the ROVIO (Robust Visual Inertial Odometry) framework. The code is open-source. Please remember that it is strongly coupled to on-going research and thus some parts are not fully mature yet. Furthermore, the code will also be subject to changes in the future which could include greater re-factoring of some parts.

Video: [https://youtu.be/ZMAISVy-6ao](https://youtu.be/ZMAISVy-6ao)

Paper:  [http://dx.doi.org/10.3929/ethz-a-010566547](http://dx.doi.org/10.3929/ethz-a-010566547)

Please also have a look at the wiki: [https://github.com/ethz-asl/rovio/wiki](https://github.com/ethz-asl/rovio/wiki)

## Install without opengl scene
Dependencies:
- ROS pkgs:
  - clone into the `catkin` workspace: `git clone https://github.com/ros-perception/vision_opencv.git`
  - `apt-get ros-<distro>-opencv3`
- [kindr](https://github.com/ethz-asl/kindr)
- lightweight_filtering (as submodule, use `git submodule update --init --recursive`)

```bash
#!command
catkin build rovio --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Install with opengl scene
Additional dependencies: `opengl`, `glut`, `glew` (`sudo apt-get install freeglut3-dev`, `sudo apt-get install libglew-dev`)

```bash
#!command
catkin build rovio --cmake-args -DCMAKE_BUILD_TYPE=Release -DMAKE_SCENE=ON
```

## Euroc Datasets
The `rovio_node.launch` file loads parameters such that ROVIO runs properly on the [Euroc datasets](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets).

## Further notes
- Camera matrix and distortion parameters should be provided by a `.yaml` file or loaded through `rosparam`
- The `cfg/rovio.info` provides most parameters for ROVIO. The camera extrinsics `qCM` (quaternion from IMU to camera frame, JPL-convention) and `MrMC` (Translation between IMU and Camera expressed in the IMU frame) should also be set there. They are being estimated during runtime so only a rough guess should be sufficient.
- Especially for application with little motion fixing the IMU-camera extrinsics can be beneficial. This can be done by setting the parameter `doVECalibration` to `false`. Please be careful that the overall robustness and accuracy can be very sensitive to bad extrinsic calibrations.
