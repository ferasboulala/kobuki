mkdir -p build bin
cd build

export CC=/usr/bin/gcc-9
export CXX=/usr/bin/g++-9
cmake ..
make

cd ..
