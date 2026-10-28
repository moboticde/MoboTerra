# terraspiro

This repo contains code for Terraspiro platforms. It currenty aims to support manual joystick control of a
4-wheel-steer-drive omnidirectional platform.

## Starting the robot

The power to the robot is given by turning the key switch on the left side of the robot to the ON position.

The Terraspiro platform is set up so that all the necessary software runs automatically on boot (via systemd service
called `terraspiro-drivers`).

The service launches the ROS2 launch file located in `launch/terraspiro.launch.py`.

The launch file start all the ROS2 nodes required for the vehicle to be operated manually.

Part of the launch procedure is automatic enabling of high-voltage output from the battery. It takes ~10 seconds from
boot for the high-voltage to enable ("READY" lamp should be green).

When the high-voltage is enabled, the platform can be controlled.

## Controlling the robot

To wake the joystick, press the START button on the joystick.

To control the robot, you must enable the drives first by using the blue button (X).

It may take a couple of presses of the button for the drives to enable. You will know when the drives are enabled,
because the platform will respond to commands.

The gamepad controls are mapped as follows.

- The **left (analog) toggle is the "reverse throttle"**, i.t., it instructs the vehicle to drive backwards
- The **right (analog) toggle is the "forward throttle"**, i.e., it instructs the vehicle to drive forward
  -- The **left analog joystick controls the rotational velocity component**, i.e., it "redirects" a part of the
  throttle to rotation
- The **right analog joystick controls the lateral velocity component**, i.e., it "redirects" a part of the throttle to
  lateral motion
- The **top-left and top-right pushbuttons (RB, LB) are turbo buttons**, either one of them disables default velocity
  scaling values

When you are no longer controlling the vehicle, you can disable the drives by pressing the orange button (Y). This is
mainly for safety reasons, it's not necessary.

If the drives are not enabling for a while, it could be that they went into error. You can clear existing errors by
pressing the red button (B).

## Components

1. Joystick
   The joystick is a Logitech F710. It's connected to the PC via a USB dongle connected directly to the PC. It does
   sometimes show troubles with the signal when you are far away from the vehicle.
2. NUC
   The NUC PC is configured to auto-start when the vehicle receives power. Each company has it's own user. The
   systemd
   service used to run the stack is currently under the `romb` user.
3. Teltonika
   The Teltonika is connected to the PC via Ethernet cable. The SICK gateway and the LSLidar are also connected. It's
   configured to provide an access point. When connected to the access point, it enable access to it's LAN (192.168.
   *.*). The Teltonika Web-UI is accessible at 192.168.1.1.
4. SICK gateway
   There is a SICK GENT gateway installed on the vehicle. It's hostname on the Teltonika's network is `sick-cpu`. It's
   IP is `192.168.1.142`. You can use an Ethernet cable to connect to it and configure it via SICK SafetyDesigner. The
   project is on GoogleDrive.
5. Vanguard batteries
   The Vanguard batteries (all 4 of them) are controlled via Briggs-Strattons parallel battery controller.

## Developer instructions

### Installation

Follow the standard ROS2 installation procedure, assuming that you already have ROS2 set up and a functioning workspace.
After cloning the repo to the `src` folder of your repo, install the dependencies by invoking `rosdep` from the repo
root.

You have to install the required dependencies (you can use 'rosdep' to do that).

Then build the workspace.

```
colcon build 
```

### Node discovery issues

There are some problems with node discovery when ssh-ing on the vehicle and using `ros2 topic list` and
`ros2 node list`

I investigated the problem. Here are my findings:

1. This problem happens at random, sometimes it just works out-of-the-box , sometimes it doesn't. If I restart my
   computer
   a couple of times, it would eventually happen (1 of 5 times). Both `ros2 node list` and `ros2 topic list` display
   nothing
   and `\rosout , \parameter_events` respectively.
2. The behaviour is **incosistent**, sometimes _--no-daemon_ option helps, sometimes it doesn't. This seems to be a
   problem with `ros2cli`  (which provides the `ros2 node` , `ros2 topic` etc.) and the `ros2 daemon` which
   contains the connection graph these tools use to visualize the information.
3. The nodes still connect together and are functional, but the so-called "ROS2 Introspection" tools are not available.
   These include: _rqt_graph_ , _rqt_tf_tree_ , _ros2 node_ , _ros2 topic_ . However, some tools (like _rviz2_ ) are still
   available... they just won't have topics listed. But if the topic names are entered manually, it will work. The 
   _--no-daemon_ option "usually" works when discovery server is set-up.

The solution was created by following: [ROS2 discovery server](https://docs.ros.org/en/jazzy/Tutorials/Advanced/Discovery-Server/Discovery-Server.html)

There is no complete solution, but the problem can be mitigated:

1. **Network settings**
   a) enable multicast on loopback (lo)
   b) enable traffic on ALL ports on lo

2. **ROS2 settings (FastRTPS middleware)**
   a) enable discovery server (this is good for multi-robot, high number of nodes setup)
   b) add [super_client config file](rmw_config/fast_rtps_supercl_cfg_file.xml) for ros2 daemon and restart it.

I have tested the solutions proposed, and it seems to work more consistently with the following _zsh_ config:

```zsh
export ROS_DOMAIN_ID=0
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
export FASTRTPS_DEFAULT_PROFILES_FILE=<path_to_super_client_xml_cfg_file>
ros2 daemon stop && ros2 daemon start;
```
