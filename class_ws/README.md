To setup the ROS Noetic environment with Docker, create an image from this repo:
```bash
docker build -t ros-noetic .
```

Then run the container, giving display permissions:
`docker run -it --name ros1 --net=host --privileged --env="QT_X11_NO_MITSHM=1" -e DISPLAY=$DISPLAY -eQT_DEBUG_PLUGINS=1 -v /tmp/.X11-unix:/tmp/.X11-unix --device /dev/video0:/dev/video0 -v $(pwd):/workspace ros-noetic:latest bash`

To start the container later:
`docker start ros1`

To run another terminals:
`xhost + && docker exec -it --user $(id -u):$(id -g) ros1 bas`