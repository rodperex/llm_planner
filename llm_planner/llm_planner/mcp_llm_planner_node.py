#!/usr/bin/env python3

# Copyright 2026 Rodrigo Perez-Rodriguez
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import json
import shlex
import subprocess
import threading
import uuid

import rclpy
from rclpy.executors import MultiThreadedExecutor
import yaml

from llm_planner.llm_planner_node import LLMPlannerNode # LLMPlannerAgentNode


class StdioMCPClient:
    # Thin JSON-RPC client over stdio.
    # Each MCP-enabled node owns its own subprocess to keep A/B behavior isolated.
    def __init__(self, cmd: str, timeout_sec: float = 2.0):
        """Create an MCP stdio client.

        Args:
            cmd: Shell command used to spawn the MCP server process.
            timeout_sec: Reserved for timeout handling (not enforced yet).
        """
        self.cmd = cmd
        self.timeout_sec = timeout_sec
        self.proc = None
        self.lock = threading.Lock()

    def start(self):
        """Start MCP subprocess and perform protocol handshake once.

        Handshake sequence (MCP):
          1) initialize (request/response)
          2) notifications/initialized (fire-and-forget)
        """
        if self.proc is not None:
            return
        parts = shlex.split(self.cmd)
        self.proc = subprocess.Popen(
            parts,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        # MCP handshake: initialize -> initialized notification.
        self._rpc("initialize", {"protocolVersion": "2024-11-05", "capabilities": {}, "clientInfo": {"name": "mcp_llm_planner_node", "version": "0.0.1"}})
        self._notify("notifications/initialized", {})

    def stop(self):
        """Stop MCP subprocess with graceful shutdown when possible."""
        if self.proc is None:
            return
        try:
            # Best-effort graceful shutdown. Failures here should not crash node teardown.
            self._rpc("shutdown", {})
            self._notify("exit", {})
        except Exception:
            pass
        try:
            self.proc.terminate()
        except Exception:
            pass
        self.proc = None

    def _notify(self, method: str, params: dict):
        """Send a JSON-RPC notification (no response expected)."""
        if self.proc is None or self.proc.stdin is None:
            return
        msg = {"jsonrpc": "2.0", "method": method, "params": params}
        self.proc.stdin.write(json.dumps(msg) + "\n")
        self.proc.stdin.flush()

    def _rpc(self, method: str, params: dict):
        """Send one JSON-RPC request and wait for the matching response.

        This method is synchronous and blocks until:
            - a response with the same request id is received, or
            - stdout closes / protocol-level error appears.
        """
        if self.proc is None or self.proc.stdin is None or self.proc.stdout is None:
            raise RuntimeError("MCP process is not running")

        req_id = str(uuid.uuid4())
        req = {"jsonrpc": "2.0", "id": req_id, "method": method, "params": params}
        self.proc.stdin.write(json.dumps(req) + "\n")
        self.proc.stdin.flush()

        while True:
            # MCP may emit unrelated messages/notifications.
            # Keep reading until we find our request id.
            line = self.proc.stdout.readline()
            if not line:
                raise RuntimeError("MCP server closed stdout")
            stripped = line.strip()
            if not stripped.startswith('{'):
                # Non-JSON line (e.g. ros2 run startup banners, warnings).
                # Discard silently and keep waiting for the real response.
                continue
            msg = json.loads(stripped)
            if msg.get("id") != req_id:
                continue
            if "error" in msg:
                raise RuntimeError(str(msg["error"]))
            return msg.get("result", {})

    def call_tool(self, name: str, arguments: dict):
        """Call MCP tools/call in a thread-safe way.

        Locking prevents interleaved writes/reads when callbacks run concurrently.
        """
        with self.lock:
            result = self._rpc("tools/call", {"name": name, "arguments": arguments})
            if isinstance(result, dict) and "structuredContent" in result:
                return result["structuredContent"]
            return result


class MCPPlannerNode(LLMPlannerNode): # LLMPlannerAgentNode
    # Extension pattern:
    # - Reuse LLMPlannerNode services/prompts/validation as-is.
    # - Inject MCP context right before calling base callbacks.
    # This keeps legacy planner behavior intact when MCP is disabled/fails open.
    def __init__(self):
        """Initialize base planner and optional MCP enrichment layer."""
        super().__init__()

        self.declare_parameter("mcp_enabled", False)
        self.declare_parameter("mcp_cmd", "")
        self.declare_parameter("mcp_timeout_sec", 2.0)
        self.declare_parameter("mcp_fail_open", True)

        self.mcp_enabled = bool(self.get_parameter("mcp_enabled").value)
        self.mcp_cmd = str(self.get_parameter("mcp_cmd").value)
        self.mcp_timeout_sec = float(self.get_parameter("mcp_timeout_sec").value)
        self.mcp_fail_open = bool(self.get_parameter("mcp_fail_open").value)

        self.mcp_client = None
        if self.mcp_enabled and self.mcp_cmd:
            try:
                self.mcp_client = StdioMCPClient(self.mcp_cmd, self.mcp_timeout_sec)
                self.mcp_client.start()
                self.get_logger().info("MCP enabled for planner node")
            except Exception as exc:
                self.get_logger().error(f"MCP init failed: {exc}")
                # fail_open=True => planner still serves requests without MCP enrichment.
                if not self.mcp_fail_open:
                    raise

    def destroy_node(self):
        """Ensure MCP child process is stopped before node destruction."""
        if self.mcp_client is not None:
            self.mcp_client.stop()
        return super().destroy_node()

    def _build_mcp_block(self, include_failures: bool = False) -> str:
        """Fetch runtime context from MCP tools and format it for prompts.

        Returned text is appended to plan/replan context so the base planner
        can reason with live mission data without changing its core logic.
        """
        # Pull read-only runtime context from MCP tools and append it to prompt context.
        if not self.mcp_client:
            return ""
        try:
            capabilities = self.mcp_client.call_tool("get_capabilities", {})
            snapshot = self.mcp_client.call_tool("get_mission_snapshot", {})
            failures = self.mcp_client.call_tool("get_failure_history", {"limit": 20}) if include_failures else {"items": []}

            return (
                "\n\nMCP_CONTEXT (read-only, runtime):\n"
                f"- capabilities: {json.dumps(capabilities, ensure_ascii=True)}\n"
                f"- mission_snapshot: {json.dumps(snapshot, ensure_ascii=True)}\n"
                f"- recent_failures: {json.dumps(failures, ensure_ascii=True)}\n"
                "Use this context only as additional grounding; do not invent extra skills.\n"
            )
        except Exception as exc:
            self.get_logger().warn(f"MCP query failed: {exc}")
            if self.mcp_fail_open:
                return ""
            raise

    def plan_task_callback(self, request, response):
        """Override plan callback to inject MCP context before delegating."""
        # Preserve base planner flow; only enrich request context before delegating.
        request.context = request.context + self._build_mcp_block(include_failures=False)
        return super().plan_task_callback(request, response)

    def replan_task_callback(self, request, response):
        """Override replan callback to inject MCP context into plan_yaml context.

        Replan prompt is built from serialized plan data, so we enrich the
        plan's context field directly before calling the base implementation.
        """
        # Replan uses plan_yaml as source of truth, so inject MCP block into plan.context.
        mcp_block = self._build_mcp_block(include_failures=True)
        try:
            plan = yaml.safe_load(request.plan_yaml or "{}") or {}
            current_context = str(plan.get("context", ""))
            plan["context"] = current_context + mcp_block
            request.plan_yaml = yaml.safe_dump(plan, sort_keys=False)
        except Exception as exc:
            self.get_logger().warn(f"Could not inject MCP context into replan plan_yaml: {exc}")
            if not self.mcp_fail_open:
                raise
        return super().replan_task_callback(request, response)


def main(args=None):
    """ROS 2 entrypoint: run MCP planner node with multi-threaded executor."""
    rclpy.init(args=args)
    node = MCPPlannerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
