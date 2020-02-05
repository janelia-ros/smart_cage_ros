Smart Cage ROS Interface
========================

Linux
-----

```bash
sudo apt update && sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-lark-parser \
  python3-pip \
  python-rosdep \
  python3-vcstool \
  wget
```

```bash
mkdir -p ~/smart_cage_ws/src
cd ~/smart_cage_ws
wget https://raw.githubusercontent.com/janelia-ros/smart_cage_ros/master/smart_cage.repos
vcs import src < smart_cage.repos
```
