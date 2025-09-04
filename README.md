<p align="center">
   <img src="images/resource-monitor-lizard-logo.png" alt="Resource Monitor Lizard Logo" width="30%" />
</p>

# Telegraf Resource Monitor

A ROS 2 package that integrates [Telegraf](https://www.influxdata.com/time-series-platform/telegraf/) with ROS 2 to monitor system resources and publish them as ROS messages. This package bridges the gap between Telegraf's data collection capabilities and ROS 2's distributed messaging system in an easily configurable way.

## Table of Contents

- [Motivation](#motivation)
- [Telegraf as backbone](#telegraf-as-backbone)
- [Overview](#overview)
- [Architecture](#architecture)
   - [Topics Published](#topics-published)
- [Installation](#installation)
   - [Prerequisites](#prerequisites)
   - [Installing Telegraf](#installing-telegraf)
   - [Installing Package](#installing-package)
- [Usage](#usage)
   - [Basic Launch](#basic-launch)
   - [Launch with Custom Parameters](#launch-with-custom-parameters)
- [Configuration](#configuration)
- [License](#license)
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
Telegraf is an open-source agent for collecting and reporting metrics. It supports a variety of input plugins to gather data from different sources and output plugins to send data to various destinations. By integrating Telegraf with ROS 2, we do not have to reinvent the wheel of resource monitoring and can leverage its more advanced capabilities, such as aggregators and processors.

Telegraf also present the opportunity to build out remote monitoring capabilities of the same resources over the OTLP protocol, which is a common standard for telemetry data. This can be connect to any [opentelemetry collector](https://opentelemetry.io/docs/collector/distributions/) which can then pass it on to whatever remote monitoring environment you wish.

## Architecture

The package consists of several key components:

- **Telegraf Configuration**: Custom Telegraf config that outputs metrics to a Unix socket
- **Unix Socket Manager**: Receives JSON data from Telegraf via Unix socket
- **Sensor Message Processor**: Processes incoming sensor data and manages publishers
- **Sensor Message Publisher**: Publishes resource data as ROS 2 messages
- **Custom Interfaces**: Defines message types for resource data


### Topics Published

The package dynamically creates topics based on the metrics collected by Telegraf. Examples include:

- `/cpu/cpu0` 
- `/cpu/cpu1` 
- `/cpu/cpu2` 
- `/cpu/cpu3` 
- `/cpu/cpu4` 
- `/cpu/cpu5` 
- `/cpu/cpu6` 
- `/cpu/cpu7` 
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


Each topic publishes `custom_interfaces/Resource` messages containing:
- Header with timestamp
- Array of fields with metric names and values of type `custom_interfaces/Field`

## Installation

### Prerequisites

- ROS 2 Humble (or compatible)
- lm-sensors (for temperature monitoring)

### Installing Telegraf

```bash
# Add InfluxDB repository
curl -s https://repos.influxdata.com/influxdata-archive_compat.key | sudo apt-key add -
echo "deb https://repos.influxdata.com/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/influxdata.list

# Install Telegraf
sudo apt update
sudo apt install telegraf
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

## Usage

### Basic Launch

Launch the resource monitor with the default configuration:

```bash
ros2 launch telegraf_resource_monitor telegraf_resource_monitor_launch.py
```

### Launch with Custom Parameters

```bash
ros2 launch telegraf_resource_monitor telegraf_resource_monitor_launch.py \
    config_file_path:=/path/to/your/config.yaml \
    log_level:=DEBUG
```

## Configuration

The package includes a pre-configured Telegraf configuration file at `config/telegraf.conf` that:

- Collects metrics every 1 second (configurable per input)
- Outputs data to Unix socket `/tmp/telegraf.sock`
- Includes processors for data cleanup and tagging
- Monitors CPU, memory, disk, sensors, and ROS processes

Look at the [influx plugins](https://docs.influxdata.com/telegraf/v1/plugins/) to find other plugins that can monitor relevant resources for you.

No configuration is needed on the node side, since it will parse the available fields and use its names to generate the topics accordingly.

## Maintainer

**Bart van Ingen**  
Email: van.ingen.bart@gmail.com

## Acknowledgments

- Built on [Telegraf](https://www.influxdata.com/time-series-platform/telegraf/) by InfluxData
- Uses ROS 2 for distributed messaging
