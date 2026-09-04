# Architecture

This repository contains four ROS 2 packages:

- `telegraf_resource_monitor_py` and `telegraf_resource_monitor_cpp`  
  Both Python and CPP implementation integrates Telegraf with ROS 2 to monitor system resources and publish them as ROS messages. Their architecture is the same, but there are some differences in the details, which are called out below.
- `resource_diagnostics_updater`  
  Subscribes to resource topics and updates the ROS 2 diagnostics system with the latest metrics, based on target resources stipulated in a configuration file.
- `resource_monitoring_interfaces`  
  Custom message definitions for resource monitoring.

The architecture between the packages is illustrated below:

<p align="center">
   <img src="../images/architecture_diagram.drawio.svg" alt="Resource Monitor Diagram" width="70%" />
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
             (kernel buffer)                          (publish)
```

The read thread splits the incoming byte stream into lines and puts them on a queue inside
the node. The processor thread takes them off the queue and publishes them, so the socket is
drained quickly no matter how slow publishing is.

The two implementations split the work slightly differently:

- **C++**: the read thread uses `getline` and pushes the raw string onto the queue. JSON
  parsing happens on the processor thread, in `SensorMessageBuffer::get_message`.
- **Python**: the read thread uses `recv` and splits on newlines itself. JSON parsing happens
  on that same read thread, in `SensorMessageBuffer.add_message`, so the queue holds parsed
  `SensorMessage` objects rather than strings.

Note that there are two buffers. The kernel already buffers the Unix socket and blocks Telegraf's `write()` when full rather than dropping
data. The queue absorbs bursts: Telegraf
writes on its `flush_interval`, not its `interval`, so an `interval` of `100ms` with a
`flush_interval` of `1s` would deliver a whole second of lines at once. Since
`telegraf.conf` belongs to whoever installs the package, the queue keeps the node
tolerant of rates it was not tuned for. It does not help with sustained overload,
where the input rate simply exceeds what the node can publish.

The queues also behave differently under overload:

- **C++**: the queue is bounded by the `max_buffer_size` parameter (default 100). When it is
  full, the oldest message is dropped and a warning is logged.
- **Python**: the queue is unbounded, so it grows instead of dropping.

#### Topics Published

The package dynamically creates topics based on the metrics collected by Telegraf. This is set by the config in src/telegraf_resource_monitor_py/config/telegraf.conf.

**Examples** include:

- `/cpu/cpu0`
- `/cpu/cpu1`
- `/cpu/cpu2`
- `/cpu/cpu3`
- `/cpu/cpu_total`
- `/disk/root`
- `/mem`
- `/procstat/telegraf_resource_monitor_node`
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

No topic configuration is needed on the node side, since it will parse the available fields and use its names to generate the topics accordingly. The nodes do declare a few ROS parameters:

| Parameter         | Default              | Packages   | Description                                                                                                             |
| ----------------- | -------------------- | ---------- | ----------------------------------------------------------------------------------------------------------------------- |
| `socket_path`     | `/tmp/telegraf.sock` | py and cpp | Path of the Unix socket the node creates and Telegraf writes to. Must match `outputs.socket_writer` in `telegraf.conf`. |
| `max_buffer_size` | `100`                | cpp only   | Maximum number of queued lines before the oldest is dropped.                                                            |

### resource_diagnostics_updater

The package consists of:

- **Diagnostics Resource Updater**: Subscribes to specific resource topics and updates the ROS 2 diagnostics system based on specified DiagnosedResource defined during initialization.
- **Diagnostics Resource Updater Node**: Parses a configuration file to determine which resources to monitor and initializes the Diagnostics Resource Updaters accordingly.
- **Diagnostics Publisher**: Publishes aggregated diagnostics information to the `/diagnostics` topic at 1 Hz and is an interface to the diagnostics topic for the updaters. An updater that goes to warning or error level publishes straight away instead of waiting for the next timer tick.

### resource_monitoring_interfaces

Defines custom ROS 2 message types for messages sent by the [telegraf_resource_monitor_py/cpp](#telegraf_resource_monitor_pycpp) packages, including:

- `Field.msg`: Represents a single metric field with name and value
- `Resource.msg`: Represents a resource with a header and an array of `Field` messages