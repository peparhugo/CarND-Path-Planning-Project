<<<<<<< HEAD
#! /bin/bash
brew install openssl libuv cmake zlib
git clone https://github.com/uWebSockets/uWebSockets 
cd uWebSockets
git checkout e94b6e1
patch CMakeLists.txt < ../cmakepatch.txt
mkdir build
export PKG_CONFIG_PATH=/usr/local/opt/openssl/lib/pkgconfig 
cd build
cmake ..
make 
sudo make install
cd ..
cd ..
sudo rm -r uWebSockets
||||||| merged common ancestors
=======
#! /bin/bash
brew install openssl libuv cmake zlib
git clone https://github.com/uWebSockets/uWebSockets 
cd uWebSockets
# git checkout e94b6e1
patch CMakeLists.txt < ../cmakepatch.txt
mkdir build
export PKG_CONFIG_PATH=/usr/local/opt/openssl/lib/pkgconfig 
cd build
cmake ..
make 
sudo make install
cd ..
cd ..
sudo rm -r uWebSockets
>>>>>>> fcc97984c7ff682ac9463e75cc719c7ed1e3ef3e
