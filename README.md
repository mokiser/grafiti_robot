для компиляции зайти в директорию проекта и ввести 
```
cmake    -DCMAKE_TOOLCHAIN_FILE=toolchain-rpi.cmake -DWITH_STATIC_LIBRARIES=ON -DWITH_TLS=OFF -DBUILD_SHARED_LIBS=OFF -DWITH_CLIENTS=OFF  -DWITH_PIC=ON ..
```
для скачивания компилятора на Linux ввести 
```
sudo apt install g++-arm-linux-gnueabihf
```
# grafiti_robot
