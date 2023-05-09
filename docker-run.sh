
PROJ_PATH=~/plvinsfusion_ws/src
DATASET_PATH=/media/sungho/hdd_2tb/datasets/

docker rm pl-vinsfusion
docker run -it -p 8888:8888 --gpus all --ipc=host \
    -e ROS_MASTER_URI:${ROS_MASTER_URI} \
    -e ROS_HOSTNAME:${ROS_HOSTNAME} \
    --user=$(id -u $USER):$(id -g $USER) \
    --env="DISPLAY" \
    --workdir=${PROJ_PATH} \
    --volume="/home/$USER:/home/$USER" \
    --volume="/etc/group:/etc/group:ro" \
    --volume="/etc/passwd:/etc/passwd:ro" \
    --volume="/etc/shadow:/etc/shadow:ro" \
    --volume="/etc/sudoers.d:/etc/sudoers.d:ro" \
    --volume="/etc/sudoers:/etc/sudoers:ro" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
		-v ${DATASET_PATH}:/dataset \
    --name pl-vinsfusion \
    pl-vins:latest \
    /bin/bash #-c \
    # "source /opt/ros/kinetic/setup.bash"
    #"rviz"
    # "cd /root/catkin_ws/; catkin_make; source devel/setup.bash; roslaunch plvins_estimator euroc_fix_extrinsic.launch"
