"""Base class for all simulation nodes: fault command handling, degradable links,
workload/cpu-pressure faults and node lifecycle faults."""

import random
import time
from typing import Any, Callable, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .degradation import (
    CpuPressure, DegradableLink, FaultState, LIFECYCLE_FAULTS, busy_work, process_exit,
)

FAULT_TOPIC = '/rca/fault_command'   # excluded from the monitor's graph discovery


class SimNode(Node):
    """rclpy Node with runtime degradation support.

    lifecycle_cb(action) is used by in-process nodes (sensors hosted by the world
    process) so that 'crash'/'stop'/'restart' destroy / re-create the ROS node
    instead of killing the shared process. Process-based nodes leave it None and
    exit the process (crash = abrupt, stop = clean).
    """

    def __init__(self, name: str, lifecycle_cb: Optional[Callable[[str, float], None]] = None, seed: int = 0,
                 use_global_arguments: bool = True):
        # In-process sensor nodes must not inherit the host process' global remaps
        # (e.g. launch's __node:=sim_world), otherwise every node would be renamed.
        super().__init__(name, use_global_arguments=use_global_arguments)
        self.declare_parameter('seed', int(seed))
        self.rng = random.Random(int(self.get_parameter('seed').value) * 1000 + sum(map(ord, name)))
        self.faults = FaultState(name)
        self.cpu_pressure = CpuPressure()
        self._lifecycle_cb = lifecycle_cb
        self.exit_code = None            # set by a graceful stop; spin_node() exits the process
        self._links = []
        self._fault_sub = self.create_subscription(String, FAULT_TOPIC, self._on_fault_command, 10)
        self._drain_timer = self.create_timer(0.01, self._drain_links)
        self._expire_timer = self.create_timer(0.25, self._expire_faults)

    # ----------------------------------------------------------- utilities

    def now(self) -> float:
        return time.time()

    def stamp(self):
        return self.get_clock().now().to_msg()

    def link(self, publisher) -> DegradableLink:
        lk = DegradableLink(publisher.publish, self.faults, self.rng)
        self._links.append(lk)
        return lk

    def _drain_links(self):
        now = time.time()
        for lk in self._links:
            lk.drain(now)

    def workload(self):
        """Extra real processing per callback when a workload fault is active (ms)."""
        if self.faults.is_active('workload'):
            busy_work(self.faults.param('workload', 40.0))
        if self.faults.is_active('processing_delay'):
            time.sleep(self.faults.param('processing_delay', 0.3))

    # ------------------------------------------------------------- faults

    def _on_fault_command(self, msg: String):
        cmd = self.faults.parse(msg.data)
        if cmd is None:
            return
        now = time.time()
        f_type = self.faults.apply(cmd, now)
        if f_type == 'clear':
            self.cpu_pressure.stop()
            for lk in self._links:
                lk.clear()
            self.get_logger().info(f'{self.get_name()}: faults cleared')
            self.on_fault_cleared()
            return
        self.get_logger().warn(
            f'{self.get_name()}: degradation "{f_type}" param={self.faults.param(f_type):.3g} '
            f'duration={cmd.get("duration", 10.0)}s'
        )
        if f_type == 'cpu_pressure':
            self.cpu_pressure.start(int(self.faults.param('cpu_pressure', 2.0)))
        elif f_type in LIFECYCLE_FAULTS:
            self._lifecycle(f_type, self.faults.param(f_type, 5.0))
        self.on_fault(f_type, cmd)

    def _expire_faults(self):
        for t in self.faults.expire(time.time()):
            if t == 'cpu_pressure':
                self.cpu_pressure.stop()
            self.get_logger().info(f'{self.get_name()}: degradation "{t}" expired')
            self.on_fault_expired(t)

    def _lifecycle(self, action: str, restart_after: float):
        if self._lifecycle_cb is not None:
            self._lifecycle_cb(action, restart_after)
            return
        # process-based node: 'restart' relies on the launch respawn mechanism
        self.get_logger().fatal(f'{self.get_name()}: {action} -> exiting process')
        if action == 'crash':
            process_exit(graceful=False)          # abrupt: no clean shutdown, DDS lease applies
        self.exit_code = 0                        # graceful: leave the spin loop, clean shutdown

    # hooks
    def on_fault(self, f_type: str, cmd: dict):
        pass

    def on_fault_expired(self, f_type: str):
        pass

    def on_fault_cleared(self):
        pass


def spin_node(node_factory: Callable[[], Any]):
    import os
    import sys
    rclpy.init()
    node = node_factory()
    code = 0
    try:
        # spin_once loop so a graceful stop requested from inside a callback can
        # terminate the process cleanly (shutting down from within a callback hangs)
        while rclpy.ok() and getattr(node, 'exit_code', None) is None:
            rclpy.spin_once(node, timeout_sec=0.1)
        code = getattr(node, 'exit_code', None) or 0
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
        sys.stdout.flush()
        os._exit(code)
