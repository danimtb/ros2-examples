# ros2-examples

Examples of ROS2 with different packages.

## library-consumer

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


## conan_node

Example with a package that provides a node (`my_node`) that sends messages to a topic but has a dependency to the Poco library provided by Conan.

```bash
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

$ colcon build --cmake-args "-DCMAKE_BUILD_TYPE=Release"
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

$ colcon build --cmake-args "-DCMAKE_BUILD_TYPE=Release"
Starting >>> package_dep
Finished <<< package_dep [8.69s]
Starting >>> my_package

Finished <<< my_package [10.1s]

$ source install/setup.bash
$ ros2 run my_package my_node
3e25960a79dbc69b674cd4ec67a72c62 Release
```