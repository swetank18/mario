# Building mario on Ubuntu 24.04 without nix

`flake.nix` is still the reference dependency set, and the versions below are
taken from it. This is what to do on a machine with no `nix` -- and, as it
happens, no root either, so everything lands in a user-owned prefix and nothing
is installed system-wide.

Done once, this produces a working `mario`, all four test suites, and
`slam_map_check`. It took about 40 minutes of wall clock, most of it Arrow and
librealsense compiling.

## What Ubuntu already provides

`apt` on 24.04 covers OpenCV 4.6, Eigen 3.4, PCL 1.14, yaml-cpp 0.8,
spdlog 1.12, libzmq 4.3.5, GLEW, sqlite3, libusb, nlohmann/json and Boost 1.83.
Everything else is built from source.

## The prefix

```sh
export PREFIX=$HOME/mario-deps/prefix
export PATH=$PREFIX/bin:$PATH
export CMAKE_PREFIX_PATH=$PREFIX:$PREFIX/share/ompl
export LD_LIBRARY_PATH=$PREFIX/lib:$LD_LIBRARY_PATH
export PKG_CONFIG_PATH=$PREFIX/lib/pkgconfig
```

CMake itself has to be newer than Ubuntu's: `CMakeLists.txt` asks for 3.31.6
and 24.04 ships 3.28. The official Kitware tarball is enough, no install
needed.

## Built from source

| What | Version / rev | Why not apt |
|---|---|---|
| CMake | 3.31.6 | project requires it, Ubuntu has 3.28 |
| g2o | tag `20230806_git` | not packaged; stella_vslam needs `g2o::solver_csparse` |
| CXSparse | from the `libsuitesparse-dev` deb | g2o's CSparse solver; see below |
| stella_vslam | `7b78cc95` (the flake's pin), submodules | not packaged |
| grid_map_core | `Yasharth011/grid_map_UNROS` @ `1cf4564` | the fork the code is written against |
| rerun_cpp_sdk | 0.28.1 release zip | not packaged; builds its own Arrow |
| OMPL | `eabb4d0f` = 1.7.0 | Ubuntu has 1.5.2, `CMakeLists.txt` asks for 1.7.0 |
| librealsense | v2.56.3 | not in Ubuntu main |
| yasmin | `Yasharth011/yasmin_UNROS` @ `95bcb30` | the ROS-free fork |
| onnxruntime | 1.20.1 prebuilt tarball | not packaged |
| Boost | 1.87.0 | `CMakeLists.txt` asks for 1.87, Ubuntu has 1.83 |
| cppzmq | 4.11.0 | Ubuntu has 4.10, `CMakeLists.txt` asks for 4.11 |
| taskflow, asio, cobs-c | 3.7.0 / 1.30.2 / `6cc55cd` | header-only or tiny |

Notable flags:

- **g2o**: `-DG2O_USE_CHOLMOD=OFF -DG2O_USE_CSPARSE=ON -DG2O_USE_OPENGL=OFF
  -DBUILD_WITH_MARCH_NATIVE=OFF`, plus `-DCSPARSE_INCLUDE_DIR` and
  `-DCSPARSE_LIBRARY` pointing at CXSparse.
- **stella_vslam**: `-DUSE_AVX=OFF -DBUILD_SHARED_LIBS=ON -DUSE_OPENMP=ON`.
  It builds its bundled FBoW, json and tinycolormap; only spdlog is taken from
  the system.
- **rerun**: `-DRERUN_DOWNLOAD_AND_BUILD_ARROW=ON -DRERUN_ARROW_LINK_SHARED=OFF`.
  This is the long one.
- **librealsense**: `-DFORCE_RSUSB_BACKEND=ON` and every `BUILD_*` example,
  tool and binding off. Only needed to compile against -- there is no camera
  on a dev box.
- **onnxruntime**: its exported cmake targets reference `$PREFIX/lib64`, so
  `ln -s $PREFIX/lib $PREFIX/lib64`.

## Two things that need working around

**CSparse.** g2o's CSparse solver -- which stella_vslam links directly -- wants
CXSparse's 32-bit-index API. The classic `CSparse` in SuiteSparse 7.x is
`int64_t` and will not compile against g2o. SuiteSparse's own CMake build wants
BLAS and a Fortran compiler. Without root, the shortest path is to take the
prebuilt library straight out of the Ubuntu packages, which needs no
privileges:

```sh
apt-get download libsuitesparse-dev libcxsparse4 libsuitesparseconfig7
for d in *.deb; do dpkg -x "$d" extracted; done
cp -a extracted/usr/include/suitesparse/.        $PREFIX/include/suitesparse/
cp -a extracted/usr/lib/x86_64-linux-gnu/libcxsparse*        $PREFIX/lib/
cp -a extracted/usr/lib/x86_64-linux-gnu/libsuitesparseconfig* $PREFIX/lib/
```

**VTK wants MPI.** Ubuntu's PCL config pulls in VTK 9.1, whose exported targets
name `MPI::MPI_C`, but nothing in VTK's config ever calls `find_package(MPI)`.
Without `libopenmpi-dev` installed, `find_package(PCL)` fails outright on a
missing target. Extract the openmpi debs the same way, then declare the target
CMake is looking for in a file of your own and pass it in -- the project's
`CMakeLists.txt` needs no edit:

```cmake
# mpi_shim.cmake
foreach(lang C CXX)
  if(NOT TARGET MPI::MPI_${lang})
    add_library(MPI::MPI_${lang} INTERFACE IMPORTED)
    set_target_properties(MPI::MPI_${lang} PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "$ENV{PREFIX}/include/openmpi"
      INTERFACE_LINK_LIBRARIES "$ENV{PREFIX}/lib/libmpi.so")
  endif()
endforeach()
```

## Configure and build

```sh
cmake -S . -B build \
  -DCMAKE_PREFIX_PATH="$PREFIX;$PREFIX/share/ompl" \
  -DCMAKE_PROJECT_INCLUDE_BEFORE=/path/to/mpi_shim.cmake
cmake --build build -j
(cd build && ctest --output-on-failure)
```

`CMakeLists.txt` hardcodes `set(CMAKE_BUILD_TYPE Debug)`, so `-DCMAKE_BUILD_TYPE`
is ignored. It is worth having an optimised tree as well, because the mapping
pipeline is ~40x slower unoptimised -- `mapping_test` takes 75 s against 1.9 s,
and `slam_map_check` cannot keep up with a 15 Hz frame stream at all:

```sh
cmake -S . -B build-rel -DCMAKE_CXX_FLAGS="-O2 -DNDEBUG" ...   # same as above
```

## Running against the sim

Needs a Rerun viewer listening, because `mario` calls
`connect_grpc(...).exit_on_failure()`:

```sh
pip install rerun-sdk==0.28.1   # in a venv; 24.04 is PEP 668
rerun --serve-web --port 9876
```

Then `sim/run_sim.sh`, or `sim/tools/slam_map_check` for the SLAM and mapping
checks on their own. `--yolo_model` needs a loadable ONNX file; see open issue
9 in `KNOWN_ISSUES.md`.
