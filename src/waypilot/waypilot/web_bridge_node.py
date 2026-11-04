"""
UI module: serves the web UI and bridges HTTP <-> ROS 2.

HTTP API (JSON)
  GET    /api/status               latest MissionStatus
  GET    /api/events               Server-Sent Events stream of MissionStatus
  POST   /api/waypoints            {"x": float, "y": float}
  DELETE /api/waypoints/<id>
  POST   /api/mission/<command>    start | pause | resume | stop | skip | clear

The node never holds mission state of its own. Writes are forwarded to
mission_manager services; reads come from the /waypilot/status topic.
The HTTP server runs in its own threads while rclpy spins in the main
thread, so blocking HTTP handlers never stall ROS callbacks.
"""

import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote, urlparse

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node

from waypilot.api import (
    ApiError, check_same_origin, parse_command, parse_json_body, parse_waypoint,
    parse_waypoint_id, require_json, status_to_dict)
from waypilot.ros_utils import declare_float, declare_int
from waypilot_interfaces.msg import MissionStatus
from waypilot_interfaces.srv import AddWaypoint, MissionCommand, RemoveWaypoint

CONTENT_TYPES = {
    '.html': 'text/html; charset=utf-8',
    '.js': 'application/javascript; charset=utf-8',
    '.css': 'text/css; charset=utf-8',
    '.svg': 'image/svg+xml',
    '.ico': 'image/x-icon',
}


class RosGateway(Node):
    """Thread-safe facade the HTTP handlers use to talk to ROS."""

    def __init__(self):
        super().__init__('web_bridge')
        self.host = self.declare_parameter('host', '0.0.0.0').value
        self.port = declare_int(self, 'port', 8080)
        self.service_timeout = declare_float(self, 'service_timeout', 3.0)
        static_dir = self.declare_parameter('static_dir', '').value
        self.static_dir = os.path.abspath(
            static_dir or os.path.join(get_package_share_directory('waypilot'), 'web'))

        self._add = self.create_client(AddWaypoint, '/waypilot/add_waypoint')
        self._remove = self.create_client(RemoveWaypoint, '/waypilot/remove_waypoint')
        self._command = self.create_client(MissionCommand, '/waypilot/command')

        self._cond = threading.Condition()
        self._status = None
        self._seq = 0
        self.create_subscription(MissionStatus, '/waypilot/status', self._on_status, 10)

    # ------------------------------------------------------------ status
    def _on_status(self, msg: MissionStatus) -> None:
        data = status_to_dict(msg)
        with self._cond:
            self._status = data
            self._seq += 1
            self._cond.notify_all()

    def latest_status(self):
        with self._cond:
            return self._seq, self._status

    def wait_for_status(self, last_seq: int, timeout: float):
        with self._cond:
            self._cond.wait_for(lambda: self._seq != last_seq, timeout=timeout)
            return self._seq, self._status

    # ---------------------------------------------------------- services
    def _call(self, client, request):
        if not client.service_is_ready():
            raise ApiError(503, 'Mission manager is not available')
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(self.service_timeout):
            client.remove_pending_request(future)
            raise ApiError(504, 'Mission manager did not respond in time')
        response = future.result()
        if response is None:
            raise ApiError(502, 'Service call failed')
        return response

    def add_waypoint(self, x: float, y: float):
        request = AddWaypoint.Request()
        request.x, request.y = x, y
        return self._call(self._add, request)

    def remove_waypoint(self, waypoint_id: int):
        request = RemoveWaypoint.Request()
        request.id = waypoint_id
        return self._call(self._remove, request)

    def command(self, command: str):
        request = MissionCommand.Request()
        request.command = command
        return self._call(self._command, request)


def make_handler(gateway: RosGateway, stop_event: threading.Event):

    class Handler(BaseHTTPRequestHandler):
        server_version = 'WayPilot/1.0'

        # ---------------------------------------------------------- routing
        def do_GET(self):
            path = urlparse(self.path).path
            try:
                if path == '/api/status':
                    _, status = gateway.latest_status()
                    if status is None:
                        raise ApiError(503, 'No status received yet')
                    self._json(200, status)
                elif path == '/api/events':
                    self._stream_events()
                else:
                    self._static(path)
            except ApiError as err:
                self._json(err.status, {'success': False, 'message': err.message})

        def do_POST(self):
            path = urlparse(self.path).path
            try:
                self._check_write_allowed()
                if path == '/api/waypoints':
                    x, y = parse_waypoint(parse_json_body(self._body()))
                    response = gateway.add_waypoint(x, y)
                    self._service_reply(response, {'id': response.id})
                elif path.startswith('/api/mission/'):
                    command = parse_command(path[len('/api/mission/'):])
                    self._body()  # drain (unused)
                    self._service_reply(gateway.command(command))
                else:
                    raise ApiError(404, 'Not found')
            except ApiError as err:
                self._json(err.status, {'success': False, 'message': err.message})

        def do_DELETE(self):
            path = urlparse(self.path).path
            try:
                self._check_write_allowed()
                if not path.startswith('/api/waypoints/'):
                    raise ApiError(404, 'Not found')
                waypoint_id = parse_waypoint_id(path[len('/api/waypoints/'):])
                self._service_reply(gateway.remove_waypoint(waypoint_id))
            except ApiError as err:
                self._json(err.status, {'success': False, 'message': err.message})

        # ---------------------------------------------------------- helpers
        def _check_write_allowed(self) -> None:
            # CSRF protection: a JSON content type forces a CORS preflight
            # (which this server never approves), and a foreign Origin is
            # rejected outright.
            check_same_origin(self.headers.get('Origin'), self.headers.get('Host'))
            require_json(self.headers.get('Content-Type'))

        def _body(self) -> bytes:
            try:
                length = int(self.headers.get('Content-Length', 0))
            except ValueError:
                raise ApiError(400, 'Invalid Content-Length')
            if length > 4096:
                raise ApiError(413, 'Request body too large')
            return self.rfile.read(length) if length > 0 else b''

        def _service_reply(self, response, extra=None):
            body = {'success': response.success, 'message': response.message}
            body.update(extra or {})
            self._json(200 if response.success else 409, body)

        def _json(self, status: int, body) -> None:
            data = json.dumps(body).encode('utf-8')
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(data)

        def _static(self, path: str) -> None:
            if path in ('', '/'):
                path = '/index.html'
            # Normalize ".." lexically (not realpath) so the install tree may
            # contain symlinks (colcon --symlink-install) while requests still
            # cannot escape the static directory.
            full = os.path.normpath(os.path.join(gateway.static_dir, unquote(path).lstrip('/')))
            if not full.startswith(gateway.static_dir + os.sep) or not os.path.isfile(full):
                raise ApiError(404, 'Not found')
            with open(full, 'rb') as f:
                data = f.read()
            self.send_response(200)
            self.send_header('Content-Type', CONTENT_TYPES.get(
                os.path.splitext(full)[1], 'application/octet-stream'))
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            self.wfile.write(data)

        def _stream_events(self) -> None:
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Connection', 'close')
            self.end_headers()
            seq, status = gateway.latest_status()
            last_ping = time.monotonic()
            try:
                if status is not None:
                    self._event(status)
                while not stop_event.is_set():
                    new_seq, status = gateway.wait_for_status(seq, timeout=1.0)
                    if new_seq != seq and status is not None:
                        seq = new_seq
                        self._event(status)
                    elif time.monotonic() - last_ping > 5.0:
                        self.wfile.write(b': ping\n\n')
                        self.wfile.flush()
                        last_ping = time.monotonic()
            except (BrokenPipeError, ConnectionResetError):
                pass  # Browser went away

        def _event(self, data) -> None:
            self.wfile.write(f'data: {json.dumps(data)}\n\n'.encode('utf-8'))
            self.wfile.flush()

        def log_message(self, fmt, *args):
            gateway.get_logger().debug(f'{self.address_string()} {fmt % args}')

    return Handler


def main(args=None):
    rclpy.init(args=args)
    gateway = RosGateway()
    stop_event = threading.Event()
    try:
        server = ThreadingHTTPServer((gateway.host, gateway.port),
                                     make_handler(gateway, stop_event))
    except OSError as err:
        gateway.get_logger().fatal(
            f'Cannot open the web UI on port {gateway.port}: {err.strerror}. '
            f'Another program is probably using it; start with a different '
            f'port, e.g. "port:=8090".')
        gateway.destroy_node()
        rclpy.shutdown()
        raise SystemExit(1)
    server.daemon_threads = True
    http_thread = threading.Thread(target=server.serve_forever, daemon=True)
    http_thread.start()
    gateway.get_logger().info(
        f'Web UI available at http://localhost:{gateway.port} (serving {gateway.static_dir})')
    try:
        rclpy.spin(gateway)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        stop_event.set()
        server.shutdown()
        server.server_close()
        gateway.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
