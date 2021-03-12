# Sensor Fusion Self-Driving Car Course Project 1

<img src="media/ObstacleDetectionFPS.gif" width="700" height="400" />

This repo contains the environment I used to learn about ransac, euclidean clustering, etc. while working with real LIDAR data. It was created by Udacity and the instructor Aaron Brown and can be found here https://github.com/udacity/SFND_Lidar_Obstacle_Detection.git. My contributions were to the cityBlock function within environment.cpp as well as most of the content in the other 3 files within the src/ folder.

In order to set up your own environment you may follow the instructions below.

### Ubuntu 

Same steps apply whether you use Udacity's original environment or my copy here.
```bash
$> sudo apt install libpcl-dev
$> cd ~
$> git clone https://github.com/cdurrans/SFND_Lidar_Obstacle_Detection.git
$> cd SFND_Lidar_Obstacle_Detection
$> mkdir build && cd build
$> cmake ..
$> make
$> ./environment
```

### Windows 

http://www.pointclouds.org/downloads/windows.html

### MAC

#### Install via Homebrew
1. install [homebrew](https://brew.sh/)
2. update homebrew 
	```bash
	$> brew update
	```
3. add  homebrew science [tap](https://docs.brew.sh/Taps) 
	```bash
	$> brew tap brewsci/science
	```
4. view pcl install options
	```bash
	$> brew options pcl
	```
5. install PCL 
	```bash
	$> brew install pcl
	```

#### Prebuilt Binaries via Universal Installer
http://www.pointclouds.org/downloads/macosx.html  
NOTE: very old version 

#### Build from Source

[PCL Source Github](https://github.com/PointCloudLibrary/pcl)

[PCL Mac Compilation Docs](http://www.pointclouds.org/documentation/tutorials/compiling_pcl_macosx.php)
