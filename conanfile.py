from conan import ConanFile
from conan.tools.files import copy
from conan.tools.cmake import CMake


class Project(ConanFile):
    name = "coco-ieee802154"
    description = "IEEE 802.15.4 radio module for CoCo"
    license = "MIT"
    settings = "os", "compiler", "build_type", "arch"
    options = {
        "platform": [None, "ANY"]}
    default_options = {
        "platform": None}
    generators = "CMakeDeps", "CMakeToolchain"
    exports_sources = "conanfile.py", "CMakeLists.txt", "coco/*", "test/*", "tools/*"


    # check if we are cross compiling
    def cross(self):
        if hasattr(self, "settings_build"):
            return self.settings.os != self.settings_build.os
        return False

    def requirements(self):
        self.requires("coco-loop/pow10", options={"platform": self.options.platform})
        self.requires("coco-buffer/pow10", options={"platform": self.options.platform})
        # todo: maybe put RadioDevice into separate project
        #if self.options.platform == "native" or self.options.platform == "emu":
        self.requires("coco-usb/pow10", options={"platform": self.options.platform}) # radio via USB

    def build_requirements(self):
        self.tool_requires("coco-toolchain/pow10", options={"platform": self.options.platform})
        self.test_requires("coco-devboards/pow10", options={"platform": self.options.platform})

    keep_imports = True
    def imports(self):
        # copy dependent libraries into the build folder
        copy(self, "*", src="@bindirs", dst="bin")
        copy(self, "*", src="@libdirs", dst="lib")

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

        # run unit tests if CONAN_RUN_TESTS environment variable is set to 1
        #if os.getenv("CONAN_RUN_TESTS") == "1" and not self.cross():
        #    cmake.test()

    def package(self):
        # install from build directory into package directory
        cmake = CMake(self)
        cmake.install()

    def package_info(self):
        self.cpp_info.libs = [self.name]
