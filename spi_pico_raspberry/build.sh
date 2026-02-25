cd ./build
rm -rf *
cmake -DPICO_BOARD=pico_w  -DPICO_SDK_PATH=../../../pico-sdk/ ..
make