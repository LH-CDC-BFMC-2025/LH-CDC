# Prepare Connecting and Data Transfer for Autopilot and Full Self-Driving Capability

Dự án bao gồm gói phần mềm Brain và nhưng Data có trong nó, một điểm khởi đầu tốt để bắt đầu phát triển ô tô của bạn với ROS.

Nếu ROS đang chạy trên ô tô, bạn cũng có thể tương tác từ ô tô với trình mô phỏng chính thức bằng cách làm theo hướng dẫn Trình mô phỏng để xuất bản/đăng ký các chủ đề của nó (Máy ảnh, chuyển động, vị trí, v.v.)

## 1. Tải xuống Raspberry Pi OS (phiên bản Desktop hoặc Lite) từ liên kết sau:
[Raspbian](https://www.raspberrypi.org/software/operating-systems/) 

Nếu bạn không quen với Linux và ROS, chúng tôi khuyên bạn nên bắt đầu với phiên bản SW dành cho máy tính để bàn và sau đó chuyển sang phiên bản rút gọn.

## 2. Mount image vào SD với sự hỗ trợ của phần mềm balenaetcher:

[Balenaetcher](https://www.balena.io/etcher/) 

## 3. Thiết lập môi trường. Tại đây bạn có thể chọn phương pháp phát triển:
a. Phương pháp ngoại vi (kết nối bàn phím, màn hình và chuột)
b. Cách tiếp cận từ xa (kết nối VNC hoặc SSH). Các dòng sau đây giải thích cách tiếp cận ssh

i. Bạn có thể thiết lập mạng bằng cách tạo tệp wpa_supplicant.conf khi khởi động (thẻ SD).

ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1
country=RO

network={
        ssid="SSID"
        psk="Passwd"
}

ii.	Bạn có thể kích hoạt kết nối ssh và i2c bằng cách tạo tệp ssh. một tập tin khác i2c. và một máy ảnh tập tin. khi khởi động (thẻ SD).

iii.	Quét mạng để tìm IP mới của bạn khi bạn bật nguồn te RPi

iv.	Bạn có thể kết nối với IP RPi từ bất kỳ thiết bị đầu cuối nào bằng lệnh ssh pi@192.168.*.*


## 4. Thêm repo ROS Debian vào hệ điều hành
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu buster main" > /etc/apt/sources.list.d/ros-noetic.list'
```

## 5. Thêm khóa ROS chính thức
```bash
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

# 6. Lấy tất cả thông tin meta từ các gói noetic ROS
```bash
sudo apt-get update && sudo apt-get upgrade
```

## 7. Cài đặt phần phụ thuộc của bản dựng
```bash
sudo apt-get install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake
```

## 8. Cài đặt pip3
```bash
sudo apt install python3-pip
```

## 9. Cài đặt opencv:
```bash
sudo apt install libopencv-dev python3-opencv
```

## 10. Thiết lập nguồn/repos phụ thuộc ROS
```bash
sudo rosdep init
rosdep update
```

## 11. Tìm nạp và cài đặt các phụ thuộc ROS
```bash
mkdir -p ~/ros_catkin_ws
cd ~/ros_catkin_ws
```

* Phiên bản Lite:
```bash
rosinstall_generator ros_comm sensor_msgs cv_bridge --rosdistro noetic --deps --wet-only --tar > noetic-ros_comm-wet.rosinstall 

wstool init src noetic-ros_comm-wet.rosinstall
```

* Phiên bản máy tính để bàn
```bash
rosinstall_generator desktop --rosdistro noetic --deps --wet-only --tar > noetic-desktop-wet.rosinstall 
wstool init src noetic-desktop-wet.rosinstall

rosdep install -y --from-paths src --ignore-src --rosdistro noetic -r --os=debian:buster
```

## 12. Biên dịch các gói ROS
Vì dự án ROS tiêu tốn nhiều tài nguyên nên chúng tôi cũng khuyên bạn nên tăng bộ nhớ trao đổi lên 1 GB nhưng không bắt buộc. Bạn có thể giảm nó sau đó. Bằng cách làm theo các bước tương tự và đặt lại về 100
```bash
		sudo dphys-swapfile swapoff

		sudoedit /etc/dphys-swapfile

			CONF_SWAPSIZE=1024

		sudo dphys-swapfile setup

		sudo dphys-swapfile swapon

sudo src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/noetic -j1 -DPYTHON_EXECUTABLE=/usr/bin/python3
```

## 13. Xác minh cài đặt
```bash
source /opt/ros/noetic/setup.bash

roscore
```

## 14. Bạn cũng có thể đặt nguồn cung ứng khi khởi động mỗi thiết bị đầu cuối mới.
```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```

## 15. Sửa gói bị thiếu
```bash
sudo apt install libatlas-base-dev
```

## 16. Sao chép dự án cục bộ và di chuyển vào bên trong nó

## 17. Cài đặt phụ thuộc python
```bash
pip3 install -r requirements_rpi.txt

pip3 install numpy --upgrade
```

## 18. Thiết lập giao tiếp i2c cho IMU bằng cách làm theo phần Thiết lập Raspberry Pi từ hướng dẫn này:
https://github.com/RPi-Distro/RTIMULib/tree/master/Linux 

## 19. Để biết thêm các chủ đề, hãy xem tài liệu ROS chính thức cho Raspberry Pi:
http://wiki.ros.org/ROSberryPi/

## 20. Xây dựng và chạy dự án đã chuẩn bị
```bash
catkin_make

source devel/setup.bash

roslaunch utils run_automobile_remote.launch
```

Nó sẽ chỉ chạy cùng một dự án não trong biến thể ROS. Nếu muốn kiểm tra nó từ xa, bạn có thể chạy bộ phát điều khiển từ xa và bộ thu camera từ dự án khởi động (đừng quên chỉnh sửa IP từ CameraTransmitterProcess trên rpi và remotecontroltransmitterProcess từ điều khiển từ xa).


## 21. Nếu bạn muốn cài đặt các gói ROS bổ sung sau khi cài đặt, bạn sẽ phải:
```bash
cd ~/ros_catkin_ws

sudo rm -rf build_isolated/ devel_isolated/ src/

sudo apt-get install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake

rosinstall_generator name_of_new_pkg --deps --exclude RPP > new_pkg.rosisntall

wstool init src new_pkg.rosisntall

sudo -s

nano /root/.bashrc

	add source /opt/ros/noetic/setup.bash

source /root/.bashrc

catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release  --install-space /opt/ros/noetic

nano /root/.bashrc

	remove source /opt/ros/noetic/setup.bash

exit
```


# License
All copyrights belong to Duong Minh Ngoc Phat Corporation. 

Contact us for more details.

Copyright © 2024 Duong Minh Ngoc Phat Corporation. All Rights Reserved


