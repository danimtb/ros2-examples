# ros2-examples

Examples of ROS2 to help with the development of the ROS2 integration for Conan C/C++ Package Manager.


## colcon_simple_example

Simple example with 4 different packages that depend on each other (app -> meet -> greet -> hello). The app package produces an executbale that uses the utilities from the other libraries. This is an example to showcase how Ament and Colcon work with transitive depdendencies.

This example does **NOT** use Conan.

```bash
$ source /opt/ros/humble/setup.bash
$ cd colcon_simple_example
$ colcon build
Starting >>> hello
Finished <<< hello [15.9s]
Starting >>> greet
Finished <<< greet [14.7s]
Starting >>> meet
Finished <<< meet [13.6s]
Starting >>> app
Finished <<< app [8.30s]

Summary: 4 packages finished [52.5s]

$ source install/setup.sh
$ ros2 run app main
Nice to meet you!
My greet is:
Hello World Debug!
```


## conan_install_ament_example

A ROS2 package (`consumer`) that contains a library (`hello`) with Conan dependencies (`box2d` & `boost`) that is later consumed from `app`.

The conan dependencies are installed by conan using a conanfile.txt

#### Configure

```bash
$ conan config install https://github.com/conan-io/conan-extensions.git --source-folder extensions/generators --target-folder extensions/generators
```

#### Build and run the example

```bash
$ source /opt/ros/humble/setup.bash
$ cd conan_install_ament_example
$ conan install consumer/conanfile.txt --update --build missing --output-folder install
$ colcon build --cmake-args '-DCMAKE_BUILD_TYPE=Release'
...

$ source install/setup.bash
$ ros2 run app main
Hello World Release! 8
{"pi":3.141E0,"happy":true,"name":"Boost","nothing":null,"answer":{"everything":42},"list":[1,0,2],"object":{"currency":"USD","value":4.299E1}}
```



## rosdep_install_ament_example

A ROS2 package (`consumer`) that contains a library (`hello`) with Conan dependencies (`box2d` & `boost`) that is later consumed from `app`.

The conan dependencies are installed by rosdep using the package.xml

#### Configure

```bash
$ git clone -b feature/conan git@github.com:danimtb/rosdep.git
$ cd rosdep
$ workon rosdep
$ pip install -e .
$ rosdep init
$ rosdep update
# edit /etc/ros/rosdep/sources.list.d/20-default.list to add the conan index:
# yaml https://raw.githubusercontent.com/danimtb/rosdistro/feature/conan/rosdep/conan.yaml
$ rosdep update
```

#### Build and run the example

```bash
$ source /opt/ros/humble/setup.bash
$ cd conan_consumer
$ rosdep install --from-paths consumer

$ colcon build --cmake-args '-DCMAKE_BUILD_TYPE=Release'
...

$ source install/setup.bash
$ ros2 run app main
Hello World Release! 8
{"pi":3.141E0,"happy":true,"name":"Boost","nothing":null,"answer":{"everything":42},"list":[1,0,2],"object":{"currency":"USD","value":4.299E1}}
```
