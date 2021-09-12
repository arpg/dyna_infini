#note: don't add anything after `\` even a space or you may meet errors.
#things you need to modify
#1: change the `python ...` to the program you want to run. -c means CMD command in dockerfile
#2: change the container name here to the container name in the make file. Use this to show what happened in docker
#3: change the image name here to the image name in the make file
#4: row `-e DISPALY ...` and `-v /tmp...` are used to visualize the result form the docker. The docker is like a computer without monitor in your computer. Normally you cannot visualize its result even by print or python plot. By te commands below you can print the result from the docker into the host terminal. If you want to visualize the result from python plot, you need type `xhost +` in the host terminal
#5 If you want to see the image display from the docker, open a terminal and type 'xhost +'
#6 '--device=/dev/video0:/dev/video0' make call computer device such as camera avaliable
#7 `test_build_ubuntu_container /bin/bash -c "cd /mnt/shared && python face_recognition/is_my_face.py" #tutorial_lec26.py` last line to execute something
#8 --ipc=host is used to increase shared memory size, https://github.com/pytorch/pytorch#docker-image
exec docker run \
     --name pytorch_test_container \
     --gpus all \
     --user=root \
     --rm \
     --detach=false \
     -e DISPLAY=${DISPLAY} \
     -v /tmp/.X11-unix:/tmp/.X11-unix \
     -v `pwd`:/mnt/shared \
     -it \
     --device=/dev/video0:/dev/video0 \
     -v /home/zhaozhong/dataset/arpg_realsensed455_3DM-GX5-15:/opt/dataset \
     --ipc=host \
     -w /mnt/shared/catkin_ws \
     zhch5450/dynamic_3d_reconstruction_img:1 /bin/bash
