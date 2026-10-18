import argparse
import json
import rclpy
from vision_interfaces.srv import QueryEvents, SetRecordingContext


def main():
    parser = argparse.ArgumentParser(description='Standalone local vision records; no navigation needed')
    parser.add_argument('command', choices=['list', 'context'])
    parser.add_argument('--type', default='')
    parser.add_argument('--map', default='')
    parser.add_argument('--revision', default='')
    parser.add_argument('--limit', type=int, default=30)
    options = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('vision_cli')
    try:
        if options.command == 'list':
            client = node.create_client(QueryEvents, '/vision/query_events')
            request = QueryEvents.Request(event_type=options.type, limit=options.limit)
        else:
            client = node.create_client(SetRecordingContext, '/vision/set_recording_context')
            request = SetRecordingContext.Request(map_id=options.map, map_revision=options.revision)
        if not client.wait_for_service(timeout_sec=5.):
            raise RuntimeError('vision recorder unavailable')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.)
        if not future.done() or not future.result().success:
            raise RuntimeError('vision request failed/timed out')
        result = future.result()
        print(json.dumps(json.loads(result.events_json), ensure_ascii=False, indent=2)
              if options.command == 'list' else result.message)
    finally:
        node.destroy_node()
        rclpy.shutdown()
