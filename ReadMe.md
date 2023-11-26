# Docker for Velodyne 3D lidars on ROS 2

Author: [Tobit Flatscher](https://github.com/2b-t) (2023)

[![Build](https://github.com/2b-t/velodyne-ros2-docker/actions/workflows/build.yml/badge.svg)](https://github.com/2b-t/velodyne-ros2-docker/actions/workflows/build.yml) [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)



## 0. Overview
This repository contains a Dockerfile and all the documentation required for setting up and launching a [Velodyne 3D lidar](https://velodynelidar.com/surround-lidar/) such as the VLP-16 or VLP-32 with the [Robot Operating System ROS 2](https://docs.ros.org/en/humble/index.html). I have tested this with both VLP-16 and VLP-32 lidars.

The Velodyne lidars are common in two different versions, with an **interface box** or with an **8-pin M12 connector** (M12MP-A) only. The ones with interface boxes are generally quite expensive on the second-hand market while the ones with M12 connector often go comparably cheap.

| ![Hardware integration with Velodyne LiDAR OLD – OxTS Support](https://support.oxts.com/hc/article_attachments/115006706969/mceclip0.png) | ![Hardware Integration with Velodyne LiDAR – OxTS Support](https://support.oxts.com/hc/article_attachments/360017839699/mceclip0.png) |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| Velodyne VLP-16 with interface box                           | Male 8-pin M12 connector                                     |

The interface box already comes with an overcurrent protection and gives you access to an Ethernet port as well as a power connector. For the 8-pin power connector on the other hand you will have to create your own cable. This can though be done with comparably little effort (without cutting the cable). In case you bought one without the interface box have a look at the **[cabling guide](./doc/CablingGuide.md) in this repository for information on making your own cable**.

## 1. Configuring
The set-up is similar to the Velodyne [VLP-16](http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16) and the [HDL-32E](http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20HDL-32E) lidar in ROS. As a first step we will have to **find out which network interface our lidar is connected to**. For this launch the following command 

```bash
$ for d in /sys/class/net/*; do echo "$(basename ${d}): $(cat $d/{carrier,operstate} | tr '\n' ' ')"; done
```

This will output a list of the available interfaces as well as their connection status:

```bash
br-af62670dc1bb: 0 down 
br-eabc8a210172: 0 down 
docker0: 0 down 
eno1: 0 down 
lo: 1 unknown 
wlx9ca2f491591b: 1 up 
```

Now plug-in the lidar and the corresponding network interface (should start in `en*` due to the [network interface naming convention](https://man7.org/linux/man-pages/man7/systemd.net-naming-scheme.7.html)) should change to `up` when launching the same command again:

```bash
br-af62670dc1bb: 0 down 
br-eabc8a210172: 0 down 
docker0: 0 down 
eno1: 1 up 
lo: 1 unknown 
wlx9ca2f491591b: 1 up 
```

This network interfaces corresponds to the device with the MAC-address

```bash
$ cat /sys/class/net/eno1/address
AB:12:c3:45:67:8a
```

Now you can [follow the ROS guide to configure your IP address on the host computer](http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16#Configure_your_computer.2BIBk-s_IP_address_through_the_Gnome_interface) for the corresponding network interface (or alternatively follow the [Ubuntu Net Manual](https://help.ubuntu.com/stable/ubuntu-help/net-manual.html.en)) where the configuration is performed via the graphic menu or continue with the following paragraph that performs the same set-up through the command line. Be sure to replace `eno1` with your network interface in the commands below!

Let us check the available connections with `nmcli`:

```bash
$ nmcli connection show
```

Let's create a **new connection**, assigning us the IP address `192.168.1.100`. Feel free to replace `100` with any number between `1` and `254` apart from the one the Velodyne is configured to (`201` by default).

```bash
$ nmcli connection add con-name velodyne-con ifname eno1 type ethernet ip4 192.168.1.100/24
```

Now let's inspect the connection with:

```bash
$ nmcli connection show velodyne-con
```

Let's bring up the connection with the following command:

```bash
$ nmcli connection up id velodyne-con
```

Let's show the IP address of our device:

```bash
$ ip address show dev eno1
```

**[Temporarily configure yourself an IP address](https://ubuntu.com/server/docs/network-configuration) in the `192.168.3.X` range**:

```bash
$ sudo ip addr add 192.168.3.100 dev eno1
```

Set up a **temporary route to the Velodyne**. In case a different address was configured for the Velodyne replace the address below by its address.

```bash
$ sudo route add 192.168.1.201 dev eno1
```

```bash
$ ip route show dev eno1
```

Now you should be able to open the webpage [http://192.168.1.201](http://192.168.1.201) and change the configuration for your Velodyne if desired/needed.

![Velodyne user interface](./media/velodyne-user-interface.png)

Inside your Dockerfile make sure to use the `network_mode` `host` option:

```yaml
    network_mode: "host"
```

## 2. Launching
Allow the container to display contents on your host machine by typing

```bash
$ xhost +local:root
```

Then build the Docker container with

```shell
$ docker compose -f docker-compose-gui.yml build
```
or directly with the [`devcontainer` in Visual Studio Code](https://code.visualstudio.com/docs/devcontainers/containers). For Nvidia graphic cards the file `docker-compose-gui-nvidia.yml` in combination with the [`nvidia-container-runtime`](https://nvidia.github.io/nvidia-container-runtime/) has to be used instead.
After it is done building, **connect the Velodyne lidar**, start the container

```shell
$ docker compose -f docker-compose-gui.yml up
```
and launch the [corresponding **Velodyne Pointcloud launch file**](https://github.com/ros-drivers/velodyne/tree/ros2/velodyne/launch). E.g. for the VLP-16:
```shell
$ source /opt/ros/humble/setup.bash
$ ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py
```
Make sure that the nodes have started correctly with

```bash
$ ros2 node list
```

and check if the topics are published correctly with

```shell
$ ros2 topic list
```
in combination with

```shell
$ ros2 topic info <topic_name>
```

Finally **visualize the points with Rviz** by launching

```shell
$ ros2 run rviz2 rviz2 -f velodyne
```
in another terminal and display the pointcloud published by the Velodyne.

![Velodyne VLP-16 visualization in Rviz](./media/velodyne-rviz2.png)
