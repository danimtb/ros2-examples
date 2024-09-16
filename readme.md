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


## conan_node

Example with a package that provides a node (`my_node`) that sends messages to a topic but has a dependency to the Poco library provided by Conan.

```bash
$ source /opt/ros/humble/setup.bash
$ cd conan_node/conan-poco-package
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

$ colcon build --cmake-args '-DCMAKE_BUILD_TYPE=Release' '-DCMAKE_TOOLCHAIN_FILE=conan/conan_toolchain.cmake'
Starting >>> conan-poco-package
Finished <<< conan-poco-package [14.8s]

Summary: 1 package finished [16.1s]

$ source install/setup.bash
$ ros2 run conan-poco-package my_node
[INFO] [1713440783.508279100] [minimal_publisher]: Publishing: '6cd3556deb0da54bca060b4c39479839 0'
[INFO] [1713440784.005371600] [minimal_publisher]: Publishing: '6cd3556deb0da54bca060b4c39479839 1'
[INFO] [1713440784.502670600] [minimal_publisher]: Publishing: '6cd3556deb0da54bca060b4c39479839 2'
[INFO] [1713440785.006211400] [minimal_publisher]: Publishing: '6cd3556deb0da54bca060b4c39479839 3'
[INFO] [1713440785.500347300] [minimal_publisher]: Publishing: '6cd3556deb0da54bca060b4c39479839 4'
...
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

## conan_library-conan_consumer

A ROS2 package that contains a library (`hello`) with a Conan dependency (`Poco`) that is later consumed from `my_package` via Conan as well.


```bash
$ source /opt/ros/humble/setup.bash
$ cd conan_library-conan_consumer/package_dep
$ conan create .
======== Exporting recipe to the cache ========
package_dep/1.0.0: Exporting package recipe: /home/danimtb/ros2-examples/conan_library-conan_consumer/package_dep/conanfile.py
package_dep/1.0.0: Calling export_sources()
package_dep/1.0.0: Copied 1 '.py' file: conanfile.py
package_dep/1.0.0: Copied 1 '.txt' file: CMakeLists.txt
package_dep/1.0.0: Copied 1 '.xml' file: package.xml
package_dep/1.0.0: Copied 1 '.h' file: hello.h
package_dep/1.0.0: Copied 1 '.cpp' file: hello.cpp
...
ackage_dep/1.0.0: Calling package()
package_dep/1.0.0: package(): Packaged 1 '.h' file: hello.h
package_dep/1.0.0: package(): Packaged 1 '.a' file: libpackage_dep.a
package_dep/1.0.0: Created package revision 79c4366985d8b5a3edf081e3b7959d80

$ cd ../my_package
$ conan install . --output-folder conan
...
======== Installing packages ========
bzip2/1.0.8: Already installed! (1 of 12)
expat/2.5.0: Already installed! (2 of 12)
libpq/15.4: Already installed! (3 of 12)
lz4/1.9.4: Already installed! (4 of 12)
sqlite3/3.45.0: Already installed! (5 of 12)
zlib/1.3.1: Already installed! (6 of 12)
zstd/1.5.5: Already installed! (7 of 12)
openssl/3.2.1: Already installed! (8 of 12)
pcre2/10.42: Already installed! (9 of 12)
libmysqlclient/8.1.0: Already installed! (10 of 12)
poco/1.13.3: Already installed! (11 of 12)
package_dep/1.0.0: Already installed! (12 of 12)
...

$ colcon build --cmake-args '-DCMAKE_BUILD_TYPE=Release' '-DCMAKE_TOOLCHAIN_FILE=conan/conan_toolchain.cmake'
Starting >>> my_package
Finished <<< my_package [4.42s]

Summary: 1 package finished [5.54s]

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
