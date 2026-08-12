<p align="center">
   <img src="images/resource-monitor-lizard-logo.png" alt="Resource Monitor Lizard Logo" width="30%" />
</p>

# Telegraf Resource Monitor

This repository provides a ROS 2-based resource monitoring solution that leverages [Telegraf](https://www.influxdata.com/time-series-platform/telegraf/) to collect system metrics and publish them as ROS messages, with the possibility of also plugging into ROS2 diagnostics. It is designed to be easily configurable and extensible, allowing users to monitor various system resources such as CPU, memory, disk usage, and more. There are two implementations, one in Python and one in CPP.

## Table of Contents

- [Motivation](#motivation)
- [Installation](#installation)
   - [Prerequisites](#prerequisites)
   - [Installing Telegraf](#installing-telegraf)
   - [Installing Package](#installing-package)
- [Architecture](#architecture)
   - [telegraf_resource_monitor_py/cpp](#telegraf_resource_monitor_py/cpp)
   - [diagnostics_resource_updater](#diagnostics_resource_updater)
   - [resource_monitoring_interfaces](#resource_monitoring_interfaces)
- [Maintainer](#maintainer)
- [Acknowledgments](#acknowledgments)


## Motivation
Monitoring system resources is important for maintaining the health and determining performance of robotic systems. There does not seem to be a well established solution to do this in ROS 2, with these the current ones that can be found easily online:

- https://github.com/AgoraRobotics/ros2-system-monitor
- https://github.com/kei1107/ros2-system-monitor
- https://github.com/ethz-asl/ros-system-monitor
- https://tier4.github.io/autoware.iv/tree/main/system/system_monitor/

This project attempts to fill that gap.

### Telegraf as backbone
Resource monitoring is not a unique problem to robotics, and there are many existing tools that do this well. A well established tool within the cloud native and DevOps communities is Telegraf.
[Telegraf](https://www.influxdata.com/time-series-platform/telegraf/) is an open-source agent for collecting and reporting metrics. It supports a variety of input plugins to gather data from different sources and output plugins to send data to various destinations. By integrating Telegraf with ROS 2, we do not have to reinvent the wheel of resource monitoring and can leverage its more advanced capabilities, such as aggregators and processors.

Telegraf also present the opportunity to build out remote monitoring capabilities of the same resources over the OTLP protocol, which is a common standard for telemetry data. This can be connect to any [opentelemetry collector](https://opentelemetry.io/docs/collector/distributions/) which can then pass it on to whatever remote monitoring environment you wish.


## Installation

### Prerequisites

- ROS 2 Humble (or compatible)
- lm-sensors (for temperature monitoring)

### Installing Telegraf

#### Through Apt

As per https://www.influxdata.com/get-telegraf/

```bash
# Add InfluxDB repository
# influxdata-archive.key GPG fingerprint:
#   Primary key fingerprint: 24C9 75CB A61A 024E E1B6  3178 7C3D 5715 9FC2 F927
#   Subkey fingerprint:      9D53 9D90 D332 8DC7 D6C8  D3B9 D8FF 8E1F 7DF8 B07E
wget -q https://repos.influxdata.com/influxdata-archive.key
gpg --show-keys --with-fingerprint --with-colons ./influxdata-archive.key 2>&1 | grep -q '^fpr:\+24C975CBA61A024EE1B631787C3D57159FC2F927:$' && cat influxdata-archive.key | gpg --dearmor | sudo tee /etc/apt/trusted.gpg.d/influxdata-archive.gpg > /dev/null
echo 'deb [signed-by=/etc/apt/trusted.gpg.d/influxdata-archive.gpg] https://repos.influxdata.com/debian stable main' | sudo tee /etc/apt/sources.list.d/influxdata.list

sudo apt-get update && sudo apt-get install telegraf
```

#### As linux   binary

find the specific version number from the [telegraf release page](https://github.com/influxdata/telegraf/releases) in the format x.xx.x.

Then fill this value accordingly with the following commands in terminal

```bash
wget https://dl.influxdata.com/telegraf/releases/telegraf-x.xx.x_linux_amd64.tar.gz \
    && tar -xzf telegraf-x.xx.x_linux_amd64.tar.gz \
    && rm telegraf-x.xx.x_linux_amd64.tar.gz \
    && mv telegraf-x.xx.x/usr/bin/telegraf /usr/local/bin/telegraf \
    && chmod +x /usr/local/bin/telegraf
```

### Installing Package

1. **Clone the repository** into your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/Bart-van-Ingen/ros-telegraf-monitor.git
   ```

1. **Install dependencies**:
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

1. **Build the package**:
   ```bash
   colcon build
   ```

1. **Source the workspace**:
   ```bash
   source install/setup.bash
   ```


## Architecture


This repository contains three ROS 2 packages:
- `telegraf_resource_monitor_py` and `telegraf_resource_monitor_cpp` 
   Both Python and CPP implementation integrates Telegraf with ROS 2 to monitor system resources and publish them as ROS messages.
- `resource_diagnostics_updater`
   Subscribes to resource topics and updates the ROS 2 diagnostics system with the latest metrics, based on target resources stipulated in a configuration file. 
- `resource_monitoring_interfaces`
   Custom message definitions for resource monitoring. 

The architecture between the packages is illustrated below:
<p align="center">
   <img src="images/architecture_diagram.drawio.svg" alt="Resource Monitor Diagram" width="70%" />
</p>

### telegraf_resource_monitor_py/cpp

The package consists of:

- **Telegraf Configuration**: Custom Telegraf config that outputs metrics to a Unix socket
- **Unix Socket Manager**: Receives JSON data from Telegraf via Unix socket
- **Sensor Message Processor**: Processes incoming sensor data and manages publishers
- **Sensor Message Publisher**: Publishes resource data as ROS 2 messages

#### Data Flow and Buffering

```
Telegraf --> Unix socket --> read thread --> queue --> processor thread --> ROS publishers
             (kernel buffer)  (getline)              (JSON parse, publish)
```

The read thread only reads lines and pushes the raw strings onto the queue inside the node.
Parsing and publishing happen on a seperate processor thread, so the socket is drained
quickly no matter how slow those are.

Note that there are two buffers. The kernel already buffers the Unix socket and blocks Telegraf's `write()` when full rather than dropping
data. The queue is therefore not about data loss. It absorbs **bursts**: Telegraf
writes on its `flush_interval`, not its `interval`, so `interval = "100ms"` with
`flush_interval = "1s"` delivers a whole second of lines at once. Since
`telegraf.conf` belongs to whoever installs the package, the queue keeps the node
tolerant of rates it was not tuned for. It does not help with sustained overload,
where the input rate simply exceeds what the node can publish.


#### Topics Published

The package dynamically creates topics based on the metrics collected by Telegraf. Examples include:

- `/cpu/cpu0` 
- `/cpu/cpu1` 
- `/cpu/cpu2` 
- `/cpu/cpu3` 
- `/cpu/cpu_total` 
- `/disk/root` 
- `/mem` 
- `/procstat/telegraf_resource_monitor` 
- `/sensors/acpitz_acpi_0/temp1` 
- `/sensors/amdgpu_pci_0400/edge` 
- `/sensors/amdgpu_pci_0400/slowppt` 
- `/sensors/amdgpu_pci_0400/vddgfx` 
- `/sensors/amdgpu_pci_0400/vddnb` 
- `/sensors/bat1_acpi_0/in0` 
- `/sensors/iwlwifi_1_virtual_0/temp1` 
- `/sensors/k10temp_pci_00c3/tctl` 
- `/sensors/nvme_pci_0100/composite` 
- `/sensors/nvme_pci_0100/sensor_1` 


Each topic publishes `Resource` messages from the [resource_monitoring_interfaces](#resource_monitoring_interfaces) package.

#### Usage

##### Basic Launch

Run the following command to launch the Telegraf resource monitor with default settings:

```bash
ros2 launch telegraf_resource_monitor_py telegraf_resource_monitor_launch.py
```

##### Launch with Custom Parameters and Logging Level
the following command allows you to specify a custom ROS2 configuration file and set the logging level:

```bash
ros2 launch telegraf_resource_monitor telegraf_resource_monitor_launch.py \
    config_file_path:=/path/to/your/config.yaml \
    log_level:=DEBUG
```

##### Configuration

The package includes a pre-configured Telegraf configuration file at `config/telegraf.conf` that:

- Collects metrics every 100 millisecond (configurable per input)
- Outputs data to Unix socket `/tmp/telegraf.sock`
- Includes processors for data cleanup and tagging
- Monitors CPU, memory, disk, sensors, and ROS processes

Look at the [influx plugins](https://docs.influxdata.com/telegraf/v1/plugins/) to find other plugins that can monitor relevant resources for you.

Currently no configuration is needed on the node side, since it will parse the available fields and use its names to generate the topics accordingly.

### diagnostics_resource_updater

The package consists of:

- **Diagnostics Resource Updater**: Subscribes to specific resource topics and updates the ROS 2 diagnostics system based on specified DiagnosedResource defined during initialization.
- **Diagnostics Resource Updater Node**: Parses a configuration file to determine which resources to monitor and initializes the Diagnostics Resource Updaters accordingly.
- **Diagnostics Publisher**: Publishes aggregated diagnostics information to the `/diagnostics` topic at a regular interval and is an interface to the diagnostics topic for the updaters.

#### Usage

##### Basic Launch

Run the following command in terminal to launch the diagnostics resource updater with the default configuration file:

```bash
ros2 launch resource_diagnostics_updater resource_diagnostics_updater_launch.py
```

##### Launch with Custom Parameters and Logging Level
You can specify a custom configuration file and set the logging level using the following command:

```bash
ros2 launch resource_diagnostics_updater resource_diagnostics_updater_launch.py \ 
config_file_path:=custom_path/resource_diagnostics.yaml \
log_level:=DEBUG
```

##### Configuration
There is a sample configuration file at `config/resource_diagnostics.yaml` that specifies which resources to monitor and their corresponding diagnostic parameters. You can modify this file to suit your monitoring needs or create your own that you then specify during launch.

The configuration file uses the following format:
```yaml
/resource_diagnostics_updater_node:
  ros__parameters:
    diagnosed_resources: |
        - topic: <topic name of resource to monitor>
          name: <name to show in diagnostics>
          field: <field to monitor>
          warning_threshold: <value for warning threshold> 
          error_threshold: <value for error threshold>
```

### resource_monitoring_interfaces
Defines custom ROS 2 message types for message sent by the [telegraf_resource_monitor](#telegraf_resource_monitor), including:
- `Field.msg`: Represents a single metric field with name and value
- `Resource.msg`: Represents a resource with a header and an array of `Field` messages


## Maintainer

**Bart van Ingen**  
Email: van.ingen.bart@gmail.com

## Acknowledgments

- Built on [Telegraf](https://www.influxdata.com/time-series-platform/telegraf/) by InfluxData
- Uses ROS 2 for distributed messaging
