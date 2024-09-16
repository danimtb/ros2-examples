# ros2-examples

Examples of ROS2 with different packages.


## library-consumer

Simple example with 4 different packages that depend on each other (app -> meet -> greet -> hello). The app package produces an executbale that uses the utilities from the other libraries. This is an example to showcase how Ament and Colcon work with transitive depdendencies.

```bash
$ source /opt/ros/humble/setup.bash
$ cd library-consumer
$ rosdep install --from-paths pacakge_dep
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


## conan_library-consumer

A ROS2 package that contains a library (`hello`) with a Conan dependency (`Poco`) that is later consumed from `my_package`.

The Conan dependency of Poco has to be added to the consumer as well.

```bash
$ source /opt/ros/humble/setup.bash
$ cd conan_library-consumer/
$ conan install . --output-folder conan
...
======== Installing packages ========
bzip2/1.0.8: Already installed! (1 of 11)
expat/2.5.0: Already installed! (2 of 11)
libpq/15.4: Already installed! (3 of 11)
lz4/1.9.4: Already installed! (4 of 11)
sqlite3/3.45.0: Already installed! (5 of 11)
zlib/1.3.1: Already installed! (6 of 11)
zstd/1.5.5: Already installed! (7 of 11)
openssl/3.2.1: Already installed! (8 of 11)
pcre2/10.42: Already installed! (9 of 11)
libmysqlclient/8.1.0: Already installed! (10 of 11)
poco/1.13.3: Already installed! (11 of 11)
...

$ colcon build --cmake-args '-DCMAKE_BUILD_TYPE=Release' '-DCMAKE_TOOLCHAIN_FILE=../conan/conan_toolchain.cmake'
Starting >>> package_dep
Finished <<< package_dep [8.69s]
Starting >>> my_package

Finished <<< my_package [10.1s]

$ source install/setup.bash
$ ros2 run my_package my_node
3e25960a79dbc69b674cd4ec67a72c62 Release
```


## rosdep_example

Use rosdep integration with Conan to build a ros node.

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
$ cd rosdep_example/my_node
$ rosdep install --from-paths . [--rosdistro humble] [--os ubuntu:lucid]
$ colcon build --cmake-args '-DCMAKE_BUILD_TYPE=Release'


$ source install/setup.bash
$ ros2 run my_package my_node
```


## conan_consumer

A ROS2 package (`consumer`) that contains a library (`hello`) with a Conan dependencies (`box2d` & `boost`) that is later consumed from `app`.

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
