"""
Web Companion Ground Control Station ROS 2 Node.

Serves the Cockpit Single Page Application over HTTP and streams real-time
20Hz vehicle telemetry, attitude HUD data, obstacle clearance, and mission
state over WebSockets. Dispatches flight commands and autonomy missions.
"""

import asyncio
import json
import math
import os
import threading
from typing import Set, Dict, Any

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from aiohttp import web
import aiohttp

from geometry_msgs.msg import PointStamped, TwistStamped
from sensor_msgs.msg import NavSatFix, BatteryState, Image
from std_msgs.msg import String
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue

# Try importing PX4 messages if available
try:
    from px4_msgs.msg import VehicleCommand, VehicleOdometry, VehicleStatus, BatteryStatus, SensorGps
    PX4_MSGS_AVAILABLE = True
except ImportError:
    PX4_MSGS_AVAILABLE = False

from px4_airsim_gcs.boustrophedon_survey_helper import plan_boustrophedon_survey


class WebGcsNode(Node):
    """ROS 2 Node hosting the Web GCS HTTP and WebSocket servers."""

    def __init__(self):
        super().__init__('web_gcs_node')

        # Parameters
        self.declare_parameter('port', 8080)
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('vehicle_name', 'drone1')
        self.declare_parameter('camera_name', 'cam1')
        self.declare_parameter('camera_host_port', 8000)
        self.declare_parameter('airsim_ip', '127.0.0.1')

        self.port = self.get_parameter('port').value
        self.host = self.get_parameter('host').value
        self.vehicle_name = self.get_parameter('vehicle_name').value
        self.camera_name = self.get_parameter('camera_name').value
        self.camera_host_port = self.get_parameter('camera_host_port').value
        self.airsim_ip = self.get_parameter('airsim_ip').value

        self.get_logger().info(
            f"Initializing Web GCS Node (Host: {self.host}, Port: {self.port}, Vehicle: {self.vehicle_name})"
        )

        # Thread-safe telemetry state buffer
        self._telemetry_lock = threading.Lock()
        self._ws_clients: Set[web.WebSocketResponse] = set()
        self._loop: asyncio.AbstractEventLoop = None

        self._state: Dict[str, Any] = {
            # Odometry & Position
            "connected": True,
            "armed": False,
            "flight_mode": "Offboard (Autonomy)",
            "px4_nav_state": "OFFBOARD",
            "lat": 47.641468,        # Default AirSim Blocks coordinates
            "lon": -122.140165,
            "alt_msl": 120.0,
            "alt_agl": 10.0,
            "pos_enu": [0.0, 0.0, 10.0],
            "vel_enu": [0.0, 0.0, 0.0],
            "groundspeed": 0.0,
            "vertical_speed": 0.0,
            "roll_deg": 0.0,
            "pitch_deg": 0.0,
            "yaw_deg": 0.0,
            "heading_deg": 0.0,

            # Battery & Power
            "battery_pct": 88.0,
            "battery_voltage": 22.8,
            "battery_current": 14.5,
            "smart_rth_margin_pct": 28.5,
            "smart_rth_status": "Nominal",

            # GPS & Satellites
            "satellites": 18,
            "gps_fix_type": "RTK Fixed",
            "eph": 0.08,

            # Autonomy Stack Telemetry
            "active_algorithm": "",
            "autonomy_status": "Standby (Hover Hold)",
            "navigation_tier": "Tier0_RtkFixed",
            "min_depth_m": 25.0,
            "target_detected": False,
            "target_pos_flu": [0.0, 0.0, 0.0],

            # 3D Obstacle Clearance Sector Grid (3x3)
            "obstacle_sectors": [
                [25.0, 25.0, 25.0],   # Upper: Left, Center, Right
                [25.0, 25.0, 25.0],   # Mid:   Left, Center, Right
                [25.0, 25.0, 25.0]    # Lower: Left, Center, Right
            ],
            "obstacle_clearance": {
                "front": 25.0,
                "left": 25.0,
                "right": 25.0,
                "upper": 25.0,
                "lower": 10.0,
                "min": 10.0
            },

            # Home point
            "home_lat": 47.641468,
            "home_lon": -122.140165,
            "home_alt": 120.0,

            # Live camera stream URI
            "camera_stream_url": f"http://{self.airsim_ip}:{self.camera_host_port}/camera/{self.vehicle_name}/{self.camera_name}/scene",
            "timestamp_ms": 0,
        }

        # Subscriptions
        self._init_subscribers()

        # Publishers
        self.target_pub = self.create_publisher(PointStamped, '/autonomy/target', 10)
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/autonomy/cmd_vel_manual', 10)

        if PX4_MSGS_AVAILABLE:
            self.vehicle_cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)

        # Autonomy Parameter Client
        self.param_client = self.create_client(SetParameters, '/autonomous_flight_mode_node/set_parameters')

        # Telemetry stream timer (20 Hz broadcast to WebSockets)
        self.telemetry_timer = self.create_timer(0.05, self._on_telemetry_tick)

        # Start web server thread
        self._server_thread = threading.Thread(target=self._run_web_server, daemon=True)
        self._server_thread.start()

        # AirSim direct RPC bridge for physical simulation flight
        self._airsim = None
        self._airsim_lock = threading.Lock()
        self._ground_z = 2.707
        self._init_airsim_client()

    def _init_airsim_client(self):
        try:
            import sys
            for p in ['/mnt/c/Users/ADMIN/Documents/AirSim/PythonClient', 'c:/Users/ADMIN/Documents/AirSim/PythonClient']:
                if p not in sys.path:
                    sys.path.insert(0, p)
            import airsim
            self._airsim = airsim.MultirotorClient(ip=self.airsim_ip)
            self._airsim.confirmConnection()
            state = self._airsim.getMultirotorState()
            self._ground_z = state.kinematics_estimated.position.z_val
            self.get_logger().info(f"Connected to AirSim RPC at {self.airsim_ip} (Ground Z: {self._ground_z:.2f}m)")
        except Exception as e:
            self._airsim = None
            self.get_logger().info(f"AirSim RPC not connected directly ({e}), relying on ROS 2 telemetry")

    def _init_subscribers(self):
        """Initialize ROS 2 telemetry subscriptions."""
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # 1. Autonomy heartbeat and status
        self.create_subscription(
            String,
            '/autonomy/status',
            self._on_autonomy_status,
            10
        )

        # 2. Commanded velocity diagnostics
        self.create_subscription(
            TwistStamped,
            '/autonomy/cmd_vel',
            self._on_autonomy_cmd_vel,
            10
        )

        # 3. GPS NavSatFix
        self.create_subscription(
            NavSatFix,
            f'/airsim_node/{self.vehicle_name}/global_gps',
            self._on_navsat_fix,
            qos_sensor
        )

        # 4. Battery State
        self.create_subscription(
            BatteryState,
            f'/airsim_node/{self.vehicle_name}/battery',
            self._on_battery_state,
            10
        )

    def _on_autonomy_status(self, msg: String):
        """Parse status string emitted by AutonomousFlightModeNode."""
        raw = msg.data
        with self._telemetry_lock:
            # Example format: [Heartbeat] Mode: ACTIVE | Mission: photogrammetry_survey | NavTier: Tier0_RtkFixed | MinDepth: 18.5m | TargetDetected: NO
            if "Mission: " in raw:
                try:
                    parts = raw.split(" | ")
                    for part in parts:
                        if "Mission: " in part:
                            self._state["active_algorithm"] = part.replace("Mission: ", "").strip()
                        elif "NavTier: " in part:
                            self._state["navigation_tier"] = part.replace("NavTier: ", "").strip()
                        elif "MinDepth: " in part:
                            d_str = part.replace("MinDepth: ", "").replace("m", "").strip()
                            self._state["min_depth_m"] = float(d_str)
                        elif "TargetDetected: " in part:
                            self._state["target_detected"] = "YES" in part
                except Exception:
                    pass
            elif raw.startswith("["):
                # Algorithm status message e.g. [dynamic_avoidance] Cruising (Clearance: 12.5m)
                self._state["autonomy_status"] = raw

    def _on_autonomy_cmd_vel(self, msg: TwistStamped):
        """Record commanded velocities."""
        with self._telemetry_lock:
            self._state["vel_enu"] = [
                round(msg.twist.linear.x, 2),
                round(msg.twist.linear.y, 2),
                round(msg.twist.linear.z, 2)
            ]
            self._state["groundspeed"] = round(
                math.hypot(msg.twist.linear.x, msg.twist.linear.y), 2
            )
            self._state["vertical_speed"] = round(msg.twist.linear.z, 2)

    def _on_navsat_fix(self, msg: NavSatFix):
        """Record global GPS fix."""
        if not math.isnan(msg.latitude) and not math.isnan(msg.longitude):
            with self._telemetry_lock:
                self._state["lat"] = msg.latitude
                self._state["lon"] = msg.longitude
                self._state["alt_msl"] = msg.altitude

    def _on_battery_state(self, msg: BatteryState):
        """Record battery telemetry."""
        with self._telemetry_lock:
            self._state["battery_voltage"] = round(msg.voltage, 1)
            self._state["battery_pct"] = round(msg.percentage * 100.0, 1) if msg.percentage <= 1.0 else round(msg.percentage, 1)

    def _on_telemetry_tick(self):
        """Broadcast 20Hz telemetry packet to all connected WebSocket clients."""
        if not self._ws_clients or self._loop is None:
            return

        if self._airsim:
            try:
                with self._airsim_lock:
                    state = self._airsim.getMultirotorState()
                    pos = state.kinematics_estimated.position
                    vel = state.kinematics_estimated.linear_velocity
                    rotors = self._airsim.getRotorStates().rotors
                is_armed = any(r.get('speed', 0.0) > 10.0 for r in rotors)
                alt_agl = max(0.0, round(self._ground_z - pos.z_val, 2))
                with self._telemetry_lock:
                    self._state["pos_enu"] = [round(pos.y_val, 2), round(pos.x_val, 2), round(alt_agl, 2)]
                    self._state["alt_agl"] = alt_agl
                    self._state["groundspeed"] = round(math.hypot(vel.x_val, vel.y_val), 2)
                    self._state["vertical_speed"] = round(-vel.z_val, 2)
                    self._state["armed"] = is_armed
            except Exception:
                pass

        with self._telemetry_lock:
            self._state["timestamp_ms"] = int(self.get_clock().now().nanoseconds / 1e6)
            data_json = json.dumps(self._state)

        # Thread-safe schedule on async event loop
        asyncio.run_coroutine_threadsafe(self._broadcast_ws(data_json), self._loop)

    async def _broadcast_ws(self, message: str):
        """Asynchronously send message to all active WebSockets."""
        stale = set()
        for ws in self._ws_clients:
            try:
                await ws.send_str(message)
            except Exception:
                stale.add(ws)
        self._ws_clients.difference_update(stale)

    # =========================================================================
    # AIOHTTP WEB SERVER & REST API HANDLERS
    # =========================================================================
    def _run_web_server(self):
        """Target for background server thread."""
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)

        app = web.Application()
        app.router.add_get('/ws/telemetry', self._handle_ws_telemetry)
        app.router.add_post('/api/flight/arm', self._handle_api_arm)
        app.router.add_post('/api/flight/disarm', self._handle_api_disarm)
        app.router.add_post('/api/flight/takeoff', self._handle_api_takeoff)
        app.router.add_post('/api/flight/land', self._handle_api_land)
        app.router.add_post('/api/flight/rth', self._handle_api_rth)
        app.router.add_post('/api/flight/hold', self._handle_api_hold)
        app.router.add_post('/api/autonomy/set_algorithm', self._handle_api_set_algorithm)
        app.router.add_post('/api/autonomy/target', self._handle_api_target)
        app.router.add_post('/api/mission/plan_survey', self._handle_api_plan_survey)
        app.router.add_get('/api/system/status', self._handle_api_status)

        # Static file serving (HTML, CSS, JS)
        try:
            from ament_index_python.packages import get_package_share_directory
            static_dir = os.path.join(get_package_share_directory('px4_airsim_gcs'), 'static')
        except Exception:
            static_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'static')

        if not os.path.exists(static_dir):
            dev_static = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'static')
            if os.path.exists(dev_static):
                static_dir = dev_static

        static_dir = os.path.realpath(static_dir)
        app.router.add_static('/static', path=static_dir, name='static', follow_symlinks=True)

        async def _handle_root(request):
            index_path = os.path.join(static_dir, 'index.html')
            if os.path.exists(index_path):
                return web.FileResponse(index_path)
            return web.HTTPFound('/static/index.html')

        app.router.add_get('/', _handle_root)

        runner = web.AppRunner(app)
        self._loop.run_until_complete(runner.setup())
        site = web.TCPSite(runner, self.host, self.port)
        self._loop.run_until_complete(site.start())

        self.get_logger().info(f"==> Web GCS cockpit live at: http://{self.host}:{self.port}/")
        self._loop.run_forever()

    async def _handle_ws_telemetry(self, request):
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        self._ws_clients.add(ws)
        self.get_logger().info(f"WebSocket client connected. Total clients: {len(self._ws_clients)}")

        try:
            async for msg in ws:
                if msg.type == aiohttp.WSMsgType.TEXT:
                    # Echo or client heartbeats
                    pass
                elif msg.type == aiohttp.WSMsgType.ERROR:
                    break
        finally:
            self._ws_clients.discard(ws)
            self.get_logger().info(f"WebSocket client disconnected. Total clients: {len(self._ws_clients)}")
        return ws

    async def _handle_api_status(self, request):
        with self._telemetry_lock:
            return web.json_response(self._state)

    async def _handle_api_set_algorithm(self, request):
        try:
            data = await request.json()
            algorithm_name = data.get("algorithm", "")

            # Call ROS 2 parameter service to set algorithm on autonomous_flight_mode_node
            req = SetParameters.Request()
            param = Parameter()
            param.name = "algorithm"
            param.value = ParameterValue(
                type=ParameterType.PARAMETER_STRING,
                string_value=algorithm_name
            )
            req.parameters = [param]

            if self.param_client.service_is_ready():
                future = self.param_client.call_async(req)
                with self._telemetry_lock:
                    self._state["active_algorithm"] = algorithm_name
                return web.json_response({"success": True, "algorithm": algorithm_name})
            else:
                with self._telemetry_lock:
                    self._state["active_algorithm"] = algorithm_name
                return web.json_response({
                    "success": True,
                    "algorithm": algorithm_name,
                    "note": "Parameter client not connected, updated internal cache"
                })
        except Exception as e:
            return web.json_response({"success": False, "error": str(e)}, status=400)

    async def _handle_api_target(self, request):
        try:
            data = await request.json()
            x = float(data.get("x", 0.0))
            y = float(data.get("y", 0.0))
            z = float(data.get("z", 0.0))
            frame_id = data.get("frame_id", "map")

            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = frame_id
            msg.point.x = x
            msg.point.y = y
            msg.point.z = z

            self.target_pub.publish(msg)

            if self._airsim:
                try:
                    with self._airsim_lock:
                        target_ned_z = self._ground_z - max(2.0, z)
                        self._airsim.moveToPositionAsync(float(x), float(y), float(target_ned_z), 3.0)
                except Exception as e:
                    self.get_logger().warn(f"AirSim moveToPosition error: {e}")

            with self._telemetry_lock:
                self._state["target_detected"] = True
                self._state["target_pos_flu"] = [x, y, z]

            return web.json_response({"success": True, "target": [x, y, z]})
        except Exception as e:
            return web.json_response({"success": False, "error": str(e)}, status=400)

    async def _handle_api_plan_survey(self, request):
        try:
            data = await request.json()
            polygon = data.get("polygon", [])
            altitude = float(data.get("altitude", 30.0))
            speed = float(data.get("speed", 4.0))
            forward_overlap = float(data.get("forward_overlap", 0.75))
            side_overlap = float(data.get("side_overlap", 0.65))

            plan = plan_boustrophedon_survey(
                polygon_gps=polygon,
                altitude_m=altitude,
                speed_m_s=speed,
                forward_overlap=forward_overlap,
                side_overlap=side_overlap
            )
            return web.json_response(plan)
        except Exception as e:
            return web.json_response({"success": False, "error": str(e)}, status=400)

    async def _send_vehicle_command(self, command_id: int, param1: float = 0.0, param2: float = 0.0,
                                    param3: float = 0.0, param4: float = 0.0, param5: float = 0.0,
                                    param6: float = 0.0, param7: float = 0.0):
        """Send standard MAVLink command through PX4 uORB VehicleCommand topic."""
        if not PX4_MSGS_AVAILABLE:
            return False

        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = command_id
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.param4 = float(param4)
        msg.param5 = float(param5)
        msg.param6 = float(param6)
        msg.param7 = float(param7)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_cmd_pub.publish(msg)
        return True

    async def _handle_api_arm(self, request):
        if self._airsim:
            try:
                with self._airsim_lock:
                    self._airsim.enableApiControl(True)
                    self._airsim.armDisarm(True)
            except Exception as e:
                self.get_logger().warn(f"AirSim arm error: {e}")
        # MAV_CMD_COMPONENT_ARM_DISARM = 400, param1 = 1 (Arm)
        await self._send_vehicle_command(400, param1=1.0)
        with self._telemetry_lock:
            self._state["armed"] = True
        return web.json_response({"success": True, "action": "arm"})

    async def _handle_api_disarm(self, request):
        if self._airsim:
            try:
                with self._airsim_lock:
                    self._airsim.armDisarm(False)
                    self._airsim.enableApiControl(False)
            except Exception as e:
                self.get_logger().warn(f"AirSim disarm error: {e}")
        # MAV_CMD_COMPONENT_ARM_DISARM = 400, param1 = 0 (Disarm)
        await self._send_vehicle_command(400, param1=0.0)
        with self._telemetry_lock:
            self._state["armed"] = False
        return web.json_response({"success": True, "action": "disarm"})

    async def _handle_api_takeoff(self, request):
        data = await request.json() if request.can_read_body else {}
        alt = float(data.get("altitude", 10.0))
        if self._airsim:
            try:
                with self._airsim_lock:
                    self._airsim.enableApiControl(True)
                    self._airsim.armDisarm(True)
                    self._airsim.takeoffAsync()
            except Exception as e:
                self.get_logger().warn(f"AirSim takeoff error: {e}")
        # MAV_CMD_NAV_TAKEOFF = 22, param7 = altitude
        await self._send_vehicle_command(22, param7=alt)
        return web.json_response({"success": True, "action": "takeoff", "altitude": alt})

    async def _handle_api_land(self, request):
        if self._airsim:
            try:
                with self._airsim_lock:
                    self._airsim.cancelLastTask()
                    state = self._airsim.getMultirotorState()
                    px = state.kinematics_estimated.position.x_val
                    py = state.kinematics_estimated.position.y_val
                    self._airsim.moveToPositionAsync(float(px), float(py), float(self._ground_z), 2.0)
            except Exception as e:
                self.get_logger().warn(f"AirSim land error: {e}")
        # MAV_CMD_NAV_LAND = 21
        await self._send_vehicle_command(21)
        return web.json_response({"success": True, "action": "land"})

    async def _handle_api_rth(self, request):
        if self._airsim:
            try:
                with self._airsim_lock:
                    self._airsim.cancelLastTask()
                    self._airsim.moveToPositionAsync(0.0, 0.0, self._ground_z - 3.0, 3.0)
            except Exception as e:
                self.get_logger().warn(f"AirSim RTH error: {e}")
        # MAV_CMD_NAV_RETURN_TO_LAUNCH = 20
        await self._send_vehicle_command(20)
        return web.json_response({"success": True, "action": "rth"})

    async def _handle_api_hold(self, request):
        if self._airsim:
            try:
                with self._airsim_lock:
                    self._airsim.cancelLastTask()
                    self._airsim.hoverAsync()
            except Exception as e:
                pass
        # Switch to Standby hover hold in autonomy node
        with self._telemetry_lock:
            self._state["active_algorithm"] = ""
            self._state["autonomy_status"] = "Hold (Zero Velocity Station Keeping)"
        return web.json_response({"success": True, "action": "hold"})


def main(args=None):
    rclpy.init(args=args)
    node = WebGcsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
