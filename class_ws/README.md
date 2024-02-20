
`docker build -t ros-noetic .`

`docker run -it --name ros1 --net=host --privileged --env="QT_X11_NO_MITSHM=1" -e DISPLAY=$DISPLAY -eQT_DEBUG_PLUGINS=1 -v /tmp/.X11-unix:/tmp/.X11-unix --device /dev/video0:/dev/video0 -v $(pwd):/workspace ros-noetic:latest bash`

`docker start ros1`
`xhost + && docker exec -it --user $(id -u):$(id -g) ros1 bas`