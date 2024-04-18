ros2-examples
-------------

Examples of ROS2 with different packages

library-consumer
++++++++++++++++

Simple example with a package that provides a C++ library (`package_dep`) and a package that consumes the library from a node (`my_package`).

```bash
$ cd library-consumer
$ colcon build
Starting >>> package_dep
Finished <<< package_dep [15.9s]
Starting >>> my_package
Finished <<< my_package [8.30s]

Summary: 2 packages finished [25.7s]

$ source install/setup.sh
$ ros2 run my_package my_node
Hello World Debug!
```