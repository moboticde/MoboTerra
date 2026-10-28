## SICK Flexisoft Ethernet Communication

The **sick_flexisoft_eth_comm** package is a **ROS 2 C++ node** designed to establish and manage the TCP/IP
socket connection to a **SICK Flexisoft** CPU. Its primary function is to read safety-relevant
status information transmitted via the **SICK GENT (Ethernet)** gateway.

The node acts as a **TCP server**, listening for incoming connections from the SICK controller to receive and
process this critical status data for use by other ROS nodes. It leverages
the _boost::asio_ library for robust, asynchronous network communication.

-----

## Core Functionalities

- **TCP Server Setup:** Initializes a TCP socket on a configurable **TCP port** to listen for and accept a connection
  from the SICK GENT gateway.
- **Data Parsing and Status Publishing:** Parses the raw safety data received from the SICK gateway and
  publishes the decoded information to a dedicated ROS 2 topic using the **_terraspiro_sick_interface/SickInfo_**
  message type.

-----

## Parameters

The C++ node is configured using the following internal parameter:

| Parameter Name | Type | Description                                                                                                          |
|:---------------|:-----|:---------------------------------------------------------------------------------------------------------------------|
| `port`         | int  | **TCP Port** number on which the node acts as a server, listening for the incoming connection from the GENT gateway. |

-----

### Published Topics

| Topic Name  | Message Type                               | Purpose                                                                  |
|:------------|:-------------------------------------------|:-------------------------------------------------------------------------|
| _sick_info_ | _terraspiro\_sick\_interface/msg/SickInfo_ | **Safety state and information** data parsed from the _SICK_ controller. |

-----

## Run the Node

The node is launched using its dedicated launch file, where the communication port must be specified:

1. Launch the node:

```bash
ros2 launch sick_flexisoft_eth_comm sick_flexisoft_eth_comm.launch.py port:=9100
```

----

## Specifics

1. Currently, the raw safety data parsing is **hard-coded** (by reading the 15-th byte in the SICK status message). If
   more
   modules are added, the methodology behind the data parsing should be revisited. 