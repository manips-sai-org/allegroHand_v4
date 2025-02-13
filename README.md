Allegro Hand V4 - Redis Driver
==========================
This driver interfaces with the Allegro Hand through **Redis** (https://redis.io/)<br />
Note that this project works only for PEAK System CAN interface (chardev) for USB: PCAN-USB

## Resources: 
  - Original Git Repo:
    https://github.com/simlabrobotics/allegro_hand_linux_v4
  - User manual:
    http://wiki.wonikrobotics.com/AllegroHandWiki/index.php/File:V4_AllegroHandUsersManual_1.1.pdf

Inertial Frame and Joints
======================
![hand conventions](https://github.com/manips-sai-org/allegroHand_v4/blob/master/imgs_readme/handConventions.png)

<!-- Inertial Frame and Joints
====================== -->
![joing numbering](https://github.com/manips-sai-org/allegroHand_v4/blob/master/imgs_readme/handJointNumbers.jpg)

Required libraries
======================

1. Install necessary packages.
```
sudo apt-get install cmake gcc g++ libpopt-dev libboost-all-dev libjsoncpp-dev redis-server libhiredis-dev libeigen3-dev

```

2. Download, build, and install PCAN-USB driver for Linux: [peak-linux-driver](http://www.peak-system.com/fileadmin/media/linux/index.htm#download). Note: v8.8.0 tested.
```
tar -xzvf peak-linux-driver-x.x.tar.gz
cd peak-linux-driver-x.x
make NET=NO
sudo make install
sudo modprobe pcan
```

3. Download, build, and install PCAN-Basic API for Linux: [peak-basic-api](http://www.peak-system.com/fileadmin/media/linux/index.htm#download). Note: v4.2.0 is tested.
```
tar -xzvf PCAN_Basic_Linux-x.x.x.tar.gz
cd PCAN_Basic_Linux-x.x.x/pcanbasic
make
sudo make install
```

4. Download, build, and install Grasping Library for Linux, "libBHand": [Grasping_Library_for_Linux](http://wiki.wonikrobotics.com/AllegroHandWiki/index.php/Grasping_Library_for_Linux)
```
unzip LibBHand_{32|64}.zip
cd libBHand_{32|64}
sudo make install
sudo ldconfig
```

Build the driver: "grasp"
======================
Build allegroHand_v4 Project using cmake "out of source build" style.
```
cd allegroHand_v4
mkdir build && cd build && cmake ..
make
```
## Note:
  - You will need to replace the encoder offsets and directions and the motor directions in the array at the top of the main.cpp file. These offsets can be found on the offsets and directions table on your Allegro Hand Wiki page (front page - scroll down): Allegro_Hand_DML_Info
  - The driver executable will be found in bin/

Run the Allegro Hand: 
======================
1. Connect PCAN-USB and Allegro Hand (make sure to power off Allegro Hand)
2. Launch redis driver by typing the following command in any terminal: ```redis-server```
2. Start the grasping program: "grasp"
```
cd bin/
./grasp
```
3. Power on Allegro Hand
4. Start a redis client in a new terminal: ```redis-cli```
4. Use redis keys to move Allegro Hand
