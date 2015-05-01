# README #

### Install without opengl scene ###
Dependencies: ros, kindr, lightweight_filtering
```
#!command

catkin build rovio --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Install with opengl scene ###
Dependencies: ros, kindr, lightweight_filtering, opengl, glut, glew
```
#!command

catkin build rovio --cmake-args -DCMAKE_BUILD_TYPE=Release -DMAKE_SCENE=ON
```