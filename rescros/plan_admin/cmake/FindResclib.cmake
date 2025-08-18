# FindResclib.cmake
# Locate resclib library and headers

#  - RESCLIB_FOUND: Set to TRUE if the library was found
#  - RESCLIB_INCLUDE_DIRS: Include directories for resclib
#  - RESCLIB_LIBRARIES: Libraries to link against

# if you didn't run make install in resclib, just built it, you can use this to find the library
# you can add list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake") to your CMakeLists.txt if u want

set(RESCLIB_ROOT "/path to resclib" CACHE PATH "Root directory of resclib")
message(STATUS "Looking for resclib in: ${RESCLIB_ROOT}")

find_path(RESCLIB_INCLUDE_DIRS
    NAMES Resclib/Resclib.h
    HINTS ${RESCLIB_ROOT}/include
    PATHS ${PROJECT_SOURCE_DIR}/../resclib/include
)

find_library(RESCLIB_LIBRARIES
    NAMES resclib
    HINTS ${RESCLIB_ROOT}/lib
    PATHS ${PROJECT_SOURCE_DIR}/../resclib/lib
)

# If both are found, set RESCLIB_FOUND to TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Resclib
    REQUIRED_VARS RESCLIB_LIBRARIES RESCLIB_INCLUDE_DIRS
)

# Mark variables as advanced (they won't show up in the CMake GUI by default)
mark_as_advanced(RESCLIB_INCLUDE_DIRS RESCLIB_LIBRARIES)
