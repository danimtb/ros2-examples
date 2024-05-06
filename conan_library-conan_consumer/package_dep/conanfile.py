from conan import ConanFile
from conan.tools.files import copy

import os

class ExampleApplicationConan(ConanFile):
    name = "package_dep"
    version = "1.0.0"
    settings = "os", "arch", "compiler", "build_type"
    generators = ["CMakeToolchain", "CMakeDeps"]

    def __set_version(self):
        # Read version from pacakge.xml file
        pass

    def export_sources(self):
        copy(self, "include/*", self.recipe_folder, self.export_sources_folder)
        copy(self, "src/*", self.recipe_folder, self.export_sources_folder)
        copy(self, "CMakeLists.txt", self.recipe_folder, self.export_sources_folder)
        copy(self, "package.xml", self.recipe_folder, self.export_sources_folder)

    def requirements(self):
        self.requires("poco/1.13.3")

    def build(self):
        # source /opt/ros/humble/setup.sh
        self.run(f"colcon build --cmake-args '-DCMAKE_BUILD_TYPE={self.settings.build_type}' '-DCMAKE_TOOLCHAIN_FILE={self.build_folder}/conan_toolchain.cmake'")

    def package(self):
        copy(self, "*.h", os.path.join(self.source_folder, "include"), os.path.join(self.package_folder, "include"))
        copy(self, "*.a", os.path.join(self.build_folder, "build", "package_dep"), os.path.join(self.package_folder, "lib"), keep_path=False)

    def package_info(self):
        self.cpp_info.libs = ["package_dep"]
