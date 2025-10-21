# Add ROS 2 repo (Ubuntu 24.04 "noble")
sudo apt update
sudo apt install -y software-properties-common curl gnupg lsb-release
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list

# Install ROS 2 Jazzy desktop + common tools
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-jazzy-rviz2 ros-jazzy-slam-toolbox \
  ros-jazzy-ros-gz ros-jazzy-ros-gz-sim ros-jazzy-teleop-twist-keyboard \
  python3-colcon-common-extensions git

# Optional: If you later see "libgz-transport13.so.13 not found"
# sudo apt install -y libgz-transport13