# conan_install_example

A ROS2 package (`consumer`) that contains a library (`hello`) with Conan dependencies (`box2d` & `boost`) that is later consumed from `app`.

The conan dependencies are installed by conan using a conanfile.txt

#### Configure

```bash
$ conan config install https://github.com/conan-io/conan-extensions.git --source-folder extensions/generators --target-folder extensions/generators
```

#### Build and run the example

```bash
$ source /opt/ros/humble/setup.bash
$ cd conan_install_example
$ conan install consumer/conanfile.txt --update --build missing --output-folder install
$ colcon build --cmake-args '-DCMAKE_BUILD_TYPE=Release'
...

$ source install/setup.bash
$ ros2 run app main
Hello World Release! 8
{"pi":3.141E0,"happy":true,"name":"Boost","nothing":null,"answer":{"everything":42},"list":[1,0,2],"object":{"currency":"USD","value":4.299E1}}
```