Support for all NXP boards is available in this directory.

# How to build

build script source environment file Env.cshrc

user need to run only build command.

1. export PACKAGES_PATH=PATH_TO_edk2-platforms

2. Build desired board
   ./build.sh <SoC-name> <board-type> <build-candidate> <clean> (optional)

   Soc-name        : LS1043 / LS1046 / LS2088
   board-type      : RDB / QDS
   build-candidate : DEBUG / RELEASE

Currently, support for LS1043 RDB is provided.
