Azure Kinect Install on Ubuntu 18.04
Run all green commands within the terminal
*Note to date (Nov 21, 2019), k4a tools version 1.3 is not compatible with k4abt (body tracker). Thus k4atools and dotnet package are also downloaded as version 1.2

1) Install dependencies

sudo apt-get install -y pkg-config ninja-build doxygen clang gcc-multilib g++-multilib python3 git-lfs nasm cmake libgl1-mesa-dev libsoundio-dev libvulkan-dev libx11-dev libxcursor-dev libxinerama-dev libxrandr-dev libusb-1.0-0-dev libssl-dev libudev-dev mesa-common-dev uuid-dev libopencv-dev

2)  Repo Clone and Build

git clone https://github.com/microsoft/Azure-Kinect-Sensor-SDK

1.	Create a folder named "build" in the root of the git repo and cd into that directory.
mkdir build && cd build
2.	Run CMake from that directory. The preferred build is ninja. All other generators are untested.
cmake .. -G Ninja
3.	Run the build (ninja).
ninja
From cloned location Azure-Kinect-Sensor-SDK/scripts run:
sudo cp 99-k4a.rules /etc/udev/rules.d/

Run the following commands

curl https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -
sudo apt-add-repository https://packages.microsoft.com/ubuntu/18.04/prod
sudo apt-get update
sudo apt install libk4abt1.0-dev
sudo apt install k4a-tools=1.3.0
sudo apt install libk4a1.3-dev

To test the camera, plug the Azure Kinect into the computer (power plug and usb), then run the following command:

k4aviewer

To test the body tracker, plug the Azure Kinect into the computer (power plug and usb), then run the following command:

k4abt_simple_3d_viewer

To record the camera feed, plug the Azure Kinect into the computer (power plug and usb), then run the following command (current length set to five seconds):

k4arecorder -l 5 output.mkv

In order to run C/Java/Python scripts on the Azure Kinect….

Dotnet and Nuget Installation

wget -q https://packages.microsoft.com/config/ubuntu/18.04/packages-microsoft-prod.deb -O packages-microsoft-prod.deb

sudo dpkg -i packages-microsoft-prod.deb
sudo add-apt-repository universe
sudo apt-get update
sudo apt-get install apt-transport-https
sudo apt-get update
sudo apt-get install dotnet-sdk-3.0
dotnet new console
dotnet add package Microsoft.Azure.Kinect.Sensor --version 1.3.0
dotnet add package Microsoft.Azure.Kinect.BodyTracking --version 1.0.1

Compiling C Scripts
Make sure gcc is installed (typically installed already)
sudo apt-get install build-essential

You have to specify the loading of libraries with the "-l" (lowercase L, not uppercase i) option.
Example compile format
gcc program2.c -Wall -lk4a -lk4arecord -o prog






