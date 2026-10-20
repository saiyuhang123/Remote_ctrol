"""Keep TF reception independent of long patrol services and action callbacks."""
import threading
import uuid

from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_ros import Buffer, TransformListener


class TfFeedback:
    def __init__(self, parent):
        self.node = Node('patrol_tf_'+uuid.uuid4().hex[:10], context=parent.context,
                         use_global_arguments=False, namespace=parent.get_namespace(),
                         enable_rosout=False, start_parameter_services=False,
                         parameter_overrides=[Parameter('use_sim_time', value=parent.get_parameter('use_sim_time').value)])
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self.node)
        self.executor = SingleThreadedExecutor(context=parent.context)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self._spin, name='patrol-tf-feedback', daemon=True)
        self.thread.start()
        self.closed = False

    def _spin(self):
        try:
            self.executor.spin()
        except ExternalShutdownException:
            pass

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.executor.shutdown(timeout_sec=2.)
        self.thread.join(timeout=2.)
        self.listener.unregister()
        self.node.destroy_node()
