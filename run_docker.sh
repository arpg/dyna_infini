exec docker run \
     --name dyna_infini_container \
     --gpus all \
     --user=root \
     --rm \
     --detach=false \
     -e DISPLAY=${DISPLAY} \
     -v /tmp/.X11-unix:/tmp/.X11-unix \
     -v `pwd`:/mnt/shared \
     -it \
     -v /home/zhaozhong/dataset/arpg_realsensed455_3DM-GX5-15:/opt/dataset/realsense \
     -v /home/zhaozhong/dataset/tum:/opt/dataset/tum \
     --ipc=host \
     -w /mnt/shared/catkin_ws \
     zhch5450/dynamic_3d_reconstruction_img:2 /bin/bash
