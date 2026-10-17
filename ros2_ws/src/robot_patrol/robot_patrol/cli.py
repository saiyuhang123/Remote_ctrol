"""Small CLI for local patrol maps, points, routes and execution."""
from __future__ import annotations

import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from patrol_interfaces.action import RunRoute
from patrol_interfaces.msg import OperatorHeartbeat, RouteStep
from patrol_interfaces.srv import (
    ControlRun,
    DeletePoint,
    DeleteRoute,
    GetRuns,
    ListMaps,
    ListPoints,
    ListRoutes,
    RegisterMap,
    SetActiveMap,
    UpsertPoint,
    UpsertRoute,
)


def call_service(node: Node, client, request, timeout: float = 10.0):
    if not client.wait_for_service(timeout_sec=timeout):
        raise RuntimeError(f"服务不可用：{client.srv_name}")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done() or future.result() is None:
        raise RuntimeError(f"服务调用超时：{client.srv_name}")
    return future.result()


def print_route(route):
    print(f"route {route.route_id}  {route.name}  map={route.map_id}  rev={route.revision}")
    for index, step in enumerate(route.steps, start=1):
        print(f"  {index}. {step.point_name} ({step.point_id}) "
              f"x={step.x:.3f} y={step.y:.3f} yaw={step.yaw:.3f} dwell={step.dwell_seconds:.1f}s")


def print_status(status):
    print(json.dumps({
        "state": status.state,
        "active": status.active,
        "run_id": status.run_id,
        "route_id": status.route_id,
        "loop": status.loop_index,
        "total_loops": status.total_loops,
        "step": status.step_index,
        "steps": status.step_count,
        "point": status.current_point_name,
        "dwell_remaining": status.dwell_remaining,
        "error_code": status.error_code,
        "message": status.message,
    }, ensure_ascii=False))


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("list-maps")

    p = sub.add_parser("register-map")
    p.add_argument("navigation_yaml")
    p.add_argument("--name", default="")
    p.add_argument("--backend", default="unknown")
    p.add_argument("--localization-file", default="")
    p.add_argument("--map-id", default="")

    p = sub.add_parser("set-active-map")
    p.add_argument("map_id")
    p.add_argument("--revision", type=int, default=0)

    p = sub.add_parser("list-points")
    p.add_argument("--map-id", default="")

    p = sub.add_parser("add-point")
    p.add_argument("map_id")
    p.add_argument("name")
    p.add_argument("x", type=float)
    p.add_argument("y", type=float)
    p.add_argument("yaw", type=float)

    p = sub.add_parser("delete-point")
    p.add_argument("point_id")
    p.add_argument("--revision", type=int, default=0)

    p = sub.add_parser("list-routes")
    p.add_argument("--map-id", default="")

    p = sub.add_parser("create-route")
    p.add_argument("map_id")
    p.add_argument("name")
    p.add_argument("--steps", required=True,
                   help="逗号分隔的 point_id:dwell_seconds，例如 p1:0,p2:5")
    p.add_argument("--route-id", default="")
    p.add_argument("--revision", type=int, default=0)

    p = sub.add_parser("delete-route")
    p.add_argument("route_id")
    p.add_argument("--revision", type=int, default=0)

    p = sub.add_parser("run-route")
    p.add_argument("route_id")
    p.add_argument("map_id")
    p.add_argument("--loops", type=int, default=1,
                   help="1=单次，N=N 圈，0=持续循环")
    p.add_argument("--route-revision", type=int, default=0)
    p.add_argument("--map-revision", type=int, default=0)
    p.add_argument("--owner", default="cli")

    p = sub.add_parser("pause")
    p.add_argument("run_id")
    p = sub.add_parser("resume")
    p.add_argument("run_id")
    p = sub.add_parser("cancel")
    p.add_argument("run_id")

    sub.add_parser("runs")

    parsed = parser.parse_args(args)
    rclpy.init(args=[])
    node = Node("patrol_cli")
    try:
        if parsed.command == "list-maps":
            response = call_service(node, node.create_client(ListMaps, "/patrol/list_maps"), ListMaps.Request())
            if not response.success:
                raise RuntimeError(response.message)
            for item in response.maps:
                active = " ACTIVE" if item.map_id == response.active_map.map_id else ""
                print(f"{item.map_id}{active}  {item.name}  {item.navigation_yaml}  rev={item.revision}")
        elif parsed.command == "register-map":
            request = RegisterMap.Request()
            request.navigation_yaml = parsed.navigation_yaml
            request.name = parsed.name
            request.backend = parsed.backend
            request.localization_file = parsed.localization_file
            request.map_id = parsed.map_id
            response = call_service(node, node.create_client(RegisterMap, "/patrol/register_map"), request)
            if not response.success:
                raise RuntimeError(response.message)
            print(response.map.map_id, response.map.name, response.map.navigation_yaml,
                  f"rev={response.map.revision}")
        elif parsed.command == "set-active-map":
            request = SetActiveMap.Request()
            request.map_id = parsed.map_id
            request.expected_revision = parsed.revision
            response = call_service(node, node.create_client(SetActiveMap, "/patrol/set_active_map"), request)
            if not response.success:
                raise RuntimeError(response.message)
            print("active_map", response.active_map.map_id, response.active_map.name)
        elif parsed.command == "list-points":
            request = ListPoints.Request()
            request.map_id = parsed.map_id
            response = call_service(node, node.create_client(ListPoints, "/patrol/list_points"), request)
            if not response.success:
                raise RuntimeError(response.message)
            for item in response.points:
                print(f"{item.point_id}  {item.name}  x={item.x:.3f} y={item.y:.3f} "
                      f"yaw={item.yaw:.3f} rev={item.revision}")
        elif parsed.command == "add-point":
            request = UpsertPoint.Request()
            request.map_id = parsed.map_id
            request.name = parsed.name
            request.x = parsed.x
            request.y = parsed.y
            request.yaw = parsed.yaw
            response = call_service(node, node.create_client(UpsertPoint, "/patrol/upsert_point"), request)
            if not response.success:
                raise RuntimeError(response.message)
            print(response.point.point_id)
        elif parsed.command == "delete-point":
            request = DeletePoint.Request()
            request.point_id = parsed.point_id
            request.expected_revision = parsed.revision
            response = call_service(node, node.create_client(DeletePoint, "/patrol/delete_point"), request)
            if not response.success:
                raise RuntimeError(response.message)
            print(response.message)
        elif parsed.command == "list-routes":
            request = ListRoutes.Request()
            request.map_id = parsed.map_id
            response = call_service(node, node.create_client(ListRoutes, "/patrol/list_routes"), request)
            if not response.success:
                raise RuntimeError(response.message)
            for route in response.routes:
                print_route(route)
        elif parsed.command == "create-route":
            request = UpsertRoute.Request()
            request.route_id = parsed.route_id
            request.map_id = parsed.map_id
            request.name = parsed.name
            request.expected_revision = parsed.revision
            for item in parsed.steps.split(","):
                point_id, _, dwell = item.partition(":")
                step = RouteStep()
                step.point_id = point_id.strip()
                step.dwell_seconds = float(dwell or 0.0)
                request.steps.append(step)
            response = call_service(node, node.create_client(UpsertRoute, "/patrol/upsert_route"), request)
            if not response.success:
                raise RuntimeError(response.message)
            print_route(response.route)
        elif parsed.command == "delete-route":
            request = DeleteRoute.Request()
            request.route_id = parsed.route_id
            request.expected_revision = parsed.revision
            response = call_service(node, node.create_client(DeleteRoute, "/patrol/delete_route"), request)
            if not response.success:
                raise RuntimeError(response.message)
            print(response.message)
        elif parsed.command == "run-route":
            owner_session = parsed.owner.strip()
            if not owner_session:
                raise RuntimeError("--owner 不能为空；patrol_guard 需要匹配操作端心跳")
            heartbeat_pub = node.create_publisher(
                OperatorHeartbeat, "/patrol/operator_heartbeat", 10)

            def publish_heartbeat():
                message = OperatorHeartbeat()
                message.session_id = owner_session
                message.client_id = "patrol_cli"
                message.enabled = True
                message.stamp = node.get_clock().now().to_msg()
                heartbeat_pub.publish(message)

            node.create_timer(0.2, publish_heartbeat)
            publish_heartbeat()
            client = ActionClient(node, RunRoute, "/patrol/run_route")
            if not client.wait_for_server(timeout_sec=10.0):
                raise RuntimeError("巡航 Action 不可用")
            goal = RunRoute.Goal()
            goal.request_id = "cli_" + str(time.time_ns())
            goal.owner_session_id = owner_session
            goal.route_id = parsed.route_id
            goal.map_id = parsed.map_id
            goal.loop_count = parsed.loops
            goal.route_revision = parsed.route_revision
            goal.map_revision = parsed.map_revision
            send_future = client.send_goal_async(goal, feedback_callback=lambda fb: print_status(fb.feedback.status))
            rclpy.spin_until_future_complete(node, send_future)
            goal_handle = send_future.result()
            if goal_handle is None or not goal_handle.accepted:
                raise RuntimeError("巡航目标被拒绝")
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(node, result_future)
            result = result_future.result().result
            print(json.dumps({
                "success": result.success, "final_state": result.final_state,
                "run_id": result.run_id, "completed_loops": result.completed_loops,
                "completed_steps": result.completed_steps,
                "error_code": result.error_code, "message": result.message,
            }, ensure_ascii=False))
        elif parsed.command in ("pause", "resume", "cancel"):
            request = ControlRun.Request()
            request.run_id = parsed.run_id
            request.owner_session_id = "cli"
            request.command = {
                "pause": ControlRun.Request.PAUSE,
                "resume": ControlRun.Request.RESUME,
                "cancel": ControlRun.Request.CANCEL,
            }[parsed.command]
            response = call_service(node, node.create_client(ControlRun, "/patrol/control"), request)
            print(response.accepted, response.message)
            print_status(response.status)
            if not response.accepted:
                return 1
        elif parsed.command == "runs":
            request = GetRuns.Request()
            request.limit = 20
            response = call_service(node, node.create_client(GetRuns, "/patrol/get_runs"), request)
            if not response.success:
                raise RuntimeError(response.message)
            for item in response.runs:
                print(f"{item.run_id}  state={item.state} success={item.success} "
                      f"loops={item.completed_loops} steps={item.completed_steps} "
                      f"{item.error_code} {item.message}")
    except Exception as error:  # noqa: BLE001 - CLI boundary
        print(f"错误：{error}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
