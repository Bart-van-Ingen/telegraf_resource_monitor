# Motivation

Monitoring system resources is important for maintaining the health and determining performance of robotic systems. There does not seem to be a well established solution to do this in ROS 2, with these the current ones that can be found easily online:

- [AgoraRobotics/ros2-system-monitor](https://github.com/AgoraRobotics/ros2-system-monitor)
- [kei1107/ros2-system-monitor](https://github.com/kei1107/ros2-system-monitor)
- [ethz-asl/ros-system-monitor](https://github.com/ethz-asl/ros-system-monitor)
- [tier4/system_monitor](https://tier4.github.io/autoware.iv/tree/main/system/system_monitor/)

This project attempts to fill that gap.

## Telegraf as backbone

Resource monitoring is not a unique problem to robotics, and there are many existing tools that do this well. A well established tool within the cloud native and DevOps communities is Telegraf.
[Telegraf](https://www.influxdata.com/time-series-platform/telegraf/) is an open-source agent for collecting and reporting metrics. It supports a variety of input plugins to gather data from different sources and output plugins to send data to various destinations. By integrating Telegraf with ROS 2, we do not have to reinvent the wheel of resource monitoring and can leverage its more advanced capabilities, such as aggregators and processors.

Telegraf also present the opportunity to build out remote monitoring capabilities of the same resources over the OTLP protocol, which is a common standard for telemetry data. This can be connect to any [opentelemetry collector](https://opentelemetry.io/docs/collector/distributions/) which can then pass it on to whatever remote monitoring environment you wish.