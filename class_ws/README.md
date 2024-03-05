# MR3001B Minichallenges solutions
### By [afr2903](https://github.com/afr2903/)

This packages were developed using ROS Noetic inside Docker. If you wish a native environment, follow the [official documentation](https://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) and skip the following steps until the section of [Development](#development)

## Requirements
- Docker ([Official documentation guide](https://docs.docker.com/engine/install/))

**Warning:** If using Docker Desktop, some visual tools may require additional post-installation steps.

## Initial environment setup

The ROS Noetic Docker image used has the [Althack's Dockerfiles](https://www.althack.dev/dockerfiles/) as base, which offer further customization.

For the initial setup, build the image inside this directory:
```bash
docker build -t ros-noetic .
```

Then run the container, giving display permissions and workdir mounting:
```bash
xhost +
docker run -it --name ros1 --net=host --privileged --env="QT_X11_NO_MITSHM=1" -e DISPLAY=$DISPLAY -eQT_DEBUG_PLUGINS=1 -v /tmp/.X11-unix:/tmp/.X11-unix --device /dev/video0:/dev/video0 -v $(pwd):/workspace ros-noetic:latest bash
```

This will open an interactive bash shell. Inside `/workspace` folder, you can find the mounted directory.

Additional terminals could be opened with the `docker exec` command, but it is recommended to run `terminator -u` as this will prompt a terminal by the container without the need of repeating the `exec` command for each instance.

## Start and execute container

To start the container after the initial setup and open the interactive shell type:
```bash
docker start ros1
xhost + && docker exec -it --user $(id -u):$(id -g) ros1 bash
```

Then navigate to the `/workspace` folder to find the files.

## Development

Go to the `/class_ws` folder, then run:
```bash
catkin_make
source devel/setup.bash
```

Now the packages are built and recognizable by ROS.