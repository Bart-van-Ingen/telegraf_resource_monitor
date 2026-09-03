# Intra-Process Communication — Summary

Read [composable_nodes.md](composable_nodes.md) first. Containers make
intra-process communication possible, and that page explains containers.


## Overview

### What it is

Two nodes in the same process can pass a message directly. The message stays in
memory. It does not go to the DDS middleware. It is not serialized. It does not
go through the network stack. This is intra-process communication.

The result is lower latency and less CPU load. If one node sends the message and
no other node needs its own version, there is no copy at all. The reader gets
the same message that the writer made.

### How this repo does it

Three conditions must be true. All three are true in this repo.

1. **Both nodes run in one process.** The composed launch file loads the
   telegraf monitor and the diagnostics updater into one container. This is the
   only way the two nodes can share memory.
2. **Both nodes have the setting turned on.** The composed launch file turns
   the setting on for each of the two nodes.
3. **The publisher hands over the message.** The telegraf monitor makes the
   message and gives it away. It keeps no copy of its own. Because of this, the
   framework does not have to make a copy for the reader.

The subscriber side needs no change. The diagnostics updater asks for a message
it can only read. Several readers can therefore share one message.

### When it is off

The setting is off by default. It is also off in two normal situations:

- You start the two nodes as separate processes, with `ros2 run` or with the
  single-node launch file.
- Another process subscribes to the same topic, for example `ros2 topic echo`.

In all of these cases the messages go through DDS, as they always did. Nothing
breaks. The code is the same. Only the transport is slower.

The last case is a mix. The reader in the same process still gets the fast path.
The reader in the other process gets a DDS copy at the same time.

This mixed case is common. It happens each time you look at a topic with
`ros2 topic echo` while the container runs. Our subscriber is built for it. The
subscriber asks only to read the message. It does not ask to own it. Because of
this, the framework can give the same message to the reader in the process and
to DDS.

Do not change the subscriber to ask for ownership. In the mixed case, that costs
one more full copy of each message. The section
[Why the subscriber only reads](../../learnings/intra_process_communication_details.md#why-the-subscriber-only-reads) shows the proof.

### Limits to know

- One setting is not enough. If only one of the two nodes has it, the messages
  go through DDS.
- The QoS settings must fit. The history must keep the last N messages, the
  depth must not be zero, and the durability must be volatile. This repo meets
  all three. If it did not, the node would stop at start-up with a clear error.
- All nodes in a container share one process. This is a trade-off of
  composition, not of this setting. See [composable_nodes.md](composable_nodes.md).

## The bare minimum

This is all the code you need. Nothing else is part of the mechanism.

### 1. Turn the setting on for both nodes

Do this in the launch file, once per node in the container. This is what
[resource_monitor_composed_launch.py](../telegraf_resource_monitor_cpp/launch/resource_monitor_composed_launch.py)
does.

```python
ComposableNode(
    package="my_package",
    plugin="MyNode",
    name="my_node",
    extra_arguments=[{"use_intra_process_comms": True}],
),
```

You can also set it in the node itself. Do this only if the node must always use
the setting. The launch file is the better place, because it keeps the choice
out of the code.

```cpp
node_ = std::make_shared<rclcpp::Node>(
    "my_node", rclcpp::NodeOptions(options).use_intra_process_comms(true));
```

### 2. Give the message away in the publisher

Make the message in a `unique_ptr`. Move it into `publish`. Do not keep a copy.

```cpp
auto msg = std::make_unique<MyMessage>();
msg->data = 42;

publisher_->publish(std::move(msg));
```

### 3. Ask only to read in the subscriber

Take the message as a `ConstSharedPtr`. Do not take it as a `UniquePtr`. See
[Why the subscriber only reads](../../learnings/intra_process_communication_details.md#why-the-subscriber-only-reads).

```cpp
subscription_ = node.create_subscription<MyMessage>(
    "my_topic", 10,
    [this](const MyMessage::ConstSharedPtr& msg) { use_message(*msg); });
```

### What the official demo does that we do not need

Many people read the [official demo][2] as a recipe. Most of the demo is support
code for a small example. It is not part of the mechanism.

| Part of the demo | Do you need it |
| --- | --- |
| `struct Producer : public rclcpp::Node` | No. Our components hold a node. See [composable_nodes.md](composable_nodes.md). |
| A `main()` with an executor and two `add_node` calls | No. The container does this. |
| The `weak_ptr` capture in the timer callback | No. It prevents a reference cycle between the node, the timer and the publisher. The intra-process code does not use it. |
| A `UniquePtr` subscription callback | No, and it would hurt. See [Why the subscriber only reads](../../learnings/intra_process_communication_details.md#why-the-subscriber-only-reads). |
| Publication of a `unique_ptr` | **Yes.** This is the only necessary part. |

## How it works

Every statement above has a proof: the matching lines of the `rclcpp` headers on
the development image, plus a debug log of a real run.

[../../learnings/intra_process_communication_details.md](../../learnings/intra_process_communication_details.md)
holds that proof. It shows, with the source text of each file:

- why the manager belongs to the process, so two processes can never share one
- where `rclcpp` reads the setting, and when
- what the container does with the launch file argument
- why a plain `publish(msg)` copies and `publish(std::move(msg))` does not
- why the subscriber must only read, and what a `UniquePtr` callback would cost
- which three QoS checks run at start-up, and what they throw
- why the single-node executables cannot turn the setting on

[../../learnings/evidence/intra_process_communications/debug_print.md](../../learnings/evidence/intra_process_communications/debug_print.md)
holds the debug log of one run of the composed launch file. The address the
publisher prints and the address the subscriber prints are the same, and the
source is `intra process`.

## Sources

- [Concept: Composition][1]
- [How-to guide: Launch composable nodes][14]
- [Demo: Intra-process communication][2]

The `rclcpp` source files, with line numbers, are listed at the end of
[../../learnings/intra_process_communication_details.md](../../learnings/intra_process_communication_details.md).

[1]: https://docs.ros.org/en/humble/Concepts/Intermediate/About-Composition.html
[2]: https://docs.ros.org/en/humble/Tutorials/Demos/Intra-Process-Communication.html
[14]: https://docs.ros.org/en/humble/How-To-Guides/Launching-composable-nodes.html
