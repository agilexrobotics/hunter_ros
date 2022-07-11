read -n1 -p "Do you want to switch your package source to China mirror source? [y/n]" input
echo ""
if [ $input = "y" ];then
    echo -e "\e[32mNow switch your package source to China mirror source\e[0m"
    sudo sed -i "s/ports.ubuntu.com/mirrors.ustc.edu.cn/g" /etc/apt/sources.list
    sudo sed -i "s/archive.ubuntu.com/mirrors.ustc.edu.cn/g" /etc/apt/sources.list
    sudo sed -i "s/packages.ros.org/repo.huaweicloud.com/g" /etc/apt/sources.list.d/ros-latest.list
fi

source /opt/ros/${ROS_DISTRO}/setup.bash

apt update
apt install -y python-pip

# ghproxy=''
# read -n1 -p "Do you want to accelerate GitHub by using ghproxy? (For China user)? [y/n]" input
# if [ $input = "y" ];then
#     ghproxy='https://ghproxy.com/'
# fi

read -n1 -p "Do you want to install rosdepc and update rosdep database(For China user)? [y/n]" input
echo ""
if [ $input = "y" ];then
    echo -e "\e[32mNow update rosdep database using rosdepc....\e[0m"
    sudo pip install rosdepc
    sudo rosdepc init
    rosdepc update
fi

echo -e "\e[32mNow download ugv_sdk package....\e[0m"
git submodule update --init --recursive
echo -e "\e[32mNow install packages dependence....\e[0m"
cd .. && rosdepc install --from-paths src --ignore-src -r -y | rosdep install --from-paths src --ignore-src -r -y
source /opt/ros/${ROS_DISTRO}/setup.bash