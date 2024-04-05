# AMR project

install docker and docker compose

Docker:
https://docs.docker.com/engine/install/

Docker compose:
https://docs.docker.com/compose/install/

Build the image:

`docker compose build`

Run the container:

`docker compose up`

Connect to the container:

`docker exec -it amr_project-dev-1 bash`

# ROS commands

```bash
catkin build
source devel/setup.bash
```

```bash
roslaunch amr_prj run.launch 
```

```bash
roslaunch uuv_teleop uuv_keyboard_teleop.launch uuv_name:=bluerov2
roslaunch uuv_teleop uuv_keyboard_teleop.launch uuv_name:=ooi
```
