set(Torch_DIR "~/3rdParty/libtorch/share/cmake/Torch")

find_package(Torch REQUIRED)

#message(STATUS "Torch library status:\n"
#        "    version: ${Torch_VERSION}\n"
#        "    libraries: ${TORCH_LIBRARIES}\n"
#        "    include path: ${TORCH_INCLUDE_DIRS}")