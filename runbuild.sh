set -e

mkdir -p build
cd build
mkdir -p release debug

cd release
cmake -DCMAKE_BUILD_TYPE=Debug ../../
make -j$(nproc --all)

cd ../debug
cmake -DCMAKE_BUILD_TYPE=Release ../../
make -j$(nproc --all)

cd ../../
