#!/usr/bin/env python3
"""
Nav2 status WebSocket bridge with RTK monitoring and obstacle alert.

功能:
  - 订阅 /nav2/status，通过 WebSocket 转发 JSON（含 rtk_status、gps_hz）
  - 订阅 /gps/fix，30s 窗口判定 RTK 稳定性
  - 订阅 /odometry/gps，计算实时话题频率
  - obstacle_ahead（激光）与 follow_path_failed（recovery/卡住）分字段、分通道处理

运行方式:
  python3 nav2_status_socket_bridge.py
  python3 nav2_status_socket_bridge.py --gps-monitor   # 仅 RTK 日志模式
"""

import argparse
import asyncio
import json
import math
import subprocess
import threading
from collections import deque
from pathlib import Path
from typing import Set

import rclpy
import websockets
from nav2_status_monitor.msg import Nav2Status
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from websockets.asyncio.client import ClientConnection, connect
from websockets.asyncio.server import ServerConnection, serve

DEFAULT_OBSTACLE_WAV = Path(__file__).resolve().parent / 'alarm' / 'obstacle.wav'


# ==================== RTK 稳定性检测 ====================

def haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Haversine distance between two WGS84 points in meters."""
    earth_radius = 6371000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)

    a = (
        math.sin(dphi / 2) ** 2
        + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    )
    return earth_radius * 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))


class RtkStabilityMonitor:
    """30s sliding window RTK stability check."""

    def __init__(
        self,
        window_seconds: float = 30.0,
        min_hz: float = 5.0,
        max_speed: float = 0.4,
        speed_margin: float = 1.5,
        jump_buffer: float = 0.15,
        dynamic_speed_threshold: float = 0.05,
    ) -> None:
        self.window_seconds = window_seconds
        self.min_hz = min_hz
        self.max_speed = max_speed
        self.speed_margin = speed_margin
        self.jump_buffer = jump_buffer
        self.dynamic_speed_threshold = dynamic_speed_threshold
        self._gps_buffer: deque[dict] = deque()

    def add_fix(self, timestamp: float, latitude: float, longitude: float) -> None:
        self._gps_buffer.append({
            't': timestamp,
            'lat': latitude,
            'lon': longitude,
        })
        self._trim_buffer(timestamp)

    def is_stable(self, now: float | None = None) -> bool:
        if now is not None:
            self._trim_buffer(now)

        valid = self._valid_entries()
        if len(valid) < 2:
            return False

        span = valid[-1]['t'] - valid[0]['t']
        if span <= 0.0:
            return False

        freq = (len(valid) - 1) / span
        if freq < self.min_hz:
            return False

        if self._has_dynamic_jump(valid):
            return False

        return True

    def _trim_buffer(self, now: float) -> None:
        while self._gps_buffer and (now - self._gps_buffer[0]['t']) > self.window_seconds:
            self._gps_buffer.popleft()

    def _valid_entries(self) -> list[dict]:
        return [
            entry for entry in self._gps_buffer
            if not (entry['lat'] == 0.0 and entry['lon'] == 0.0)
        ]

    def _estimate_average_speed(self, valid: list[dict]) -> float:
        span = valid[-1]['t'] - valid[0]['t']
        if span <= 0.0:
            return 0.0

        path_length = 0.0
        for i in range(len(valid) - 1):
            path_length += haversine(
                valid[i]['lat'], valid[i]['lon'],
                valid[i + 1]['lat'], valid[i + 1]['lon'],
            )
        return path_length / span

    def _has_dynamic_jump(self, valid: list[dict]) -> bool:
        avg_speed = self._estimate_average_speed(valid)
        if avg_speed < self.dynamic_speed_threshold:
            return False

        for i in range(len(valid) - 1):
            dt = valid[i + 1]['t'] - valid[i]['t']
            if dt <= 0.0:
                continue

            dist = haversine(
                valid[i]['lat'], valid[i]['lon'],
                valid[i + 1]['lat'], valid[i + 1]['lon'],
            )
            allowed = self.max_speed * dt * self.speed_margin + self.jump_buffer
            if dist > allowed:
                return True
        return False


class TopicRateTracker:
    """Sliding-window topic rate estimator in Hz."""

    def __init__(self, window_seconds: float = 2.0) -> None:
        self.window_seconds = window_seconds
        self._timestamps: deque[float] = deque()

    def add_sample(self, timestamp: float) -> None:
        self._timestamps.append(timestamp)
        while self._timestamps and (timestamp - self._timestamps[0]) > self.window_seconds:
            self._timestamps.popleft()

    def get_hz(self) -> float:
        if len(self._timestamps) < 2:
            return 0.0

        span = self._timestamps[-1] - self._timestamps[0]
        if span <= 0.0:
            return 0.0
        return (len(self._timestamps) - 1) / span


def _create_rtk_monitor(node: Node) -> RtkStabilityMonitor:
    return RtkStabilityMonitor(
        window_seconds=node.get_parameter('rtk_window_seconds').get_parameter_value().double_value,
        min_hz=node.get_parameter('rtk_min_hz').get_parameter_value().double_value,
        max_speed=node.get_parameter('rtk_max_speed').get_parameter_value().double_value,
        speed_margin=node.get_parameter('rtk_speed_margin').get_parameter_value().double_value,
        jump_buffer=node.get_parameter('rtk_jump_buffer').get_parameter_value().double_value,
        dynamic_speed_threshold=(
            node.get_parameter('rtk_dynamic_speed_threshold').get_parameter_value().double_value
        ),
    )


def _declare_rtk_parameters(node: Node) -> None:
    node.declare_parameter('gps_topic', '/gps/fix')
    node.declare_parameter('rtk_window_seconds', 30.0)
    node.declare_parameter('rtk_min_hz', 5.0)
    node.declare_parameter('rtk_max_speed', 0.4)
    node.declare_parameter('rtk_speed_margin', 1.5)
    node.declare_parameter('rtk_jump_buffer', 0.15)
    node.declare_parameter('rtk_dynamic_speed_threshold', 0.05)


# ==================== GPS 监控（独立模式） ====================

class GPSMonitor(Node):
    """仅输出 RTK 正常 / 不稳定日志。"""

    def __init__(self) -> None:
        super().__init__('gps_monitor')

        _declare_rtk_parameters(self)
        self.declare_parameter('analyze_period_sec', 10.0)

        self._rtk_monitor = _create_rtk_monitor(self)
        gps_topic = self.get_parameter('gps_topic').get_parameter_value().string_value

        self.create_subscription(NavSatFix, gps_topic, self._gps_callback, 10)
        self.create_timer(
            self.get_parameter('analyze_period_sec').get_parameter_value().double_value,
            self._analyze_window,
        )

        self.get_logger().info(
            f'RTK监控启动 | 窗口={self._rtk_monitor.window_seconds}s | '
            f'最低频率={self._rtk_monitor.min_hz}Hz | '
            f'最大速度={self._rtk_monitor.max_speed}m/s'
        )

    def _gps_callback(self, msg: NavSatFix) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        self._rtk_monitor.add_fix(now, msg.latitude, msg.longitude)

    def _analyze_window(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9


# ==================== WebSocket Bridge ====================

class Nav2StatusSocketBridge(Node):
    """Bridge Nav2Status ROS messages to WebSocket clients or a remote server."""

    def __init__(self) -> None:
        super().__init__('nav2_status_socket_bridge')

        self.declare_parameter('status_topic', '/nav2/status')
        self.declare_parameter('socket_mode', 'server')  # server | client
        self.declare_parameter('socket_host', '0.0.0.0')
        self.declare_parameter('socket_port', 9091)
        self.declare_parameter('ws_path', '/nav2/status')
        self.declare_parameter('reconnect_interval_sec', 2.0)
        self.declare_parameter('obstacle_alert_enabled', True)
        self.declare_parameter('obstacle_wav_path', str(DEFAULT_OBSTACLE_WAV))
        self.declare_parameter('obstacle_ahead_repeat_hold_sec', 0.5)
        self.declare_parameter('obstacle_alert_repeat_interval_sec', 10.0)
        self.declare_parameter('obstacle_long_episode_play_sec', 10.0)
        self.declare_parameter('obstacle_voice_clear_grace_sec', 3.0)
        self.declare_parameter('follow_path_failed_alert_enabled', False)
        self.declare_parameter('rtk_log_period_sec', 10.0)
        self.declare_parameter('odometry_gps_topic', '/odometry/gps')
        self.declare_parameter('gps_hz_window_seconds', 2.0)
        _declare_rtk_parameters(self)

        self._mode = self.get_parameter('socket_mode').get_parameter_value().string_value
        self._host = self.get_parameter('socket_host').get_parameter_value().string_value
        self._port = self.get_parameter('socket_port').get_parameter_value().integer_value
        self._ws_path = self.get_parameter('ws_path').get_parameter_value().string_value
        if not self._ws_path.startswith('/'):
            self._ws_path = f'/{self._ws_path}'
        self._reconnect_interval = (
            self.get_parameter('reconnect_interval_sec').get_parameter_value().double_value
        )
        status_topic = self.get_parameter('status_topic').get_parameter_value().string_value
        self._obstacle_alert_enabled = (
            self.get_parameter('obstacle_alert_enabled').get_parameter_value().bool_value
        )
        self._obstacle_wav_path = Path(
            self.get_parameter('obstacle_wav_path').get_parameter_value().string_value
        )
        gps_topic = self.get_parameter('gps_topic').get_parameter_value().string_value
        odometry_gps_topic = (
            self.get_parameter('odometry_gps_topic').get_parameter_value().string_value
        )
        self._rtk_monitor = _create_rtk_monitor(self)
        self._gps_hz_tracker = TopicRateTracker(
            window_seconds=(
                self.get_parameter('gps_hz_window_seconds').get_parameter_value().double_value
            )
        )

        self._lock = threading.Lock()
        self._running = True
        self._obstacle_ahead_repeat_hold_sec = (
            self.get_parameter('obstacle_ahead_repeat_hold_sec').get_parameter_value().double_value
        )
        self._obstacle_alert_repeat_interval_sec = (
            self.get_parameter('obstacle_alert_repeat_interval_sec').get_parameter_value().double_value
        )
        self._obstacle_long_episode_play_sec = (
            self.get_parameter('obstacle_long_episode_play_sec').get_parameter_value().double_value
        )
        self._obstacle_voice_clear_grace_sec = (
            self.get_parameter('obstacle_voice_clear_grace_sec').get_parameter_value().double_value
        )
        self._follow_path_failed_alert_enabled = (
            self.get_parameter('follow_path_failed_alert_enabled').get_parameter_value().bool_value
        )
        # 激光障碍预警（obstacle_ahead）
        self._laser_since: float | None = None
        self._laser_last_play_at: float | None = None
        self._laser_play_count = 0
        self._laser_clear_since: float | None = None
        # 路径失败 / recovery（follow_path_failed）
        self._follow_path_since: float | None = None
        self._follow_path_last_play_at: float | None = None
        self._follow_path_play_count = 0
        self._follow_path_clear_since: float | None = None
        self._alert_audio_playing = False

        self._ws_loop: asyncio.AbstractEventLoop | None = None
        self._ws_thread: threading.Thread | None = None
        self._ws_stop: asyncio.Event | None = None

        self._ws_client: ClientConnection | None = None
        self._ws_peers: Set[ServerConnection] = set()

        if self._mode == 'client':
            self._start_ws_client()
            mode_desc = f'client -> {self._ws_url()}'
        elif self._mode == 'server':
            self._start_ws_server()
            mode_desc = f'server listening on {self._ws_url()}'
        else:
            raise ValueError(f'Unsupported socket_mode: {self._mode!r}')

        self.create_subscription(Nav2Status, status_topic, self._status_callback, 10)
        self.create_subscription(NavSatFix, gps_topic, self._gps_callback, 10)
        self.create_subscription(Odometry, odometry_gps_topic, self._odometry_gps_callback, 10)

        rtk_log_period = self.get_parameter('rtk_log_period_sec').get_parameter_value().double_value
        if rtk_log_period > 0.0:
            self.create_timer(rtk_log_period, self._log_rtk_status)

        self.get_logger().info(
            f'Subscribed to {status_topic}, {gps_topic}, and {odometry_gps_topic}, {mode_desc}'
        )
        if self._obstacle_alert_enabled:
            if self._obstacle_wav_path.is_file():
                self.get_logger().info(f'Obstacle alert wav: {self._obstacle_wav_path}')
            else:
                self.get_logger().warning(f'Obstacle alert wav not found: {self._obstacle_wav_path}')
            self.get_logger().info(
                'Laser obstacle voice: 1st=immediate hold=%.1fs interval=%.1fs long=%.1fs grace=%.1fs'
                % (
                    self._obstacle_ahead_repeat_hold_sec,
                    self._obstacle_alert_repeat_interval_sec,
                    self._obstacle_long_episode_play_sec,
                    self._obstacle_voice_clear_grace_sec,
                )
            )
            self.get_logger().info(
                'FollowPath recovery voice: enabled=%s'
                % self._follow_path_failed_alert_enabled
            )

    def _ws_url(self) -> str:
        return f'ws://{self._host}:{self._port}{self._ws_path}'

    def _start_ws_loop(self, coro_factory) -> None:
        self._ws_loop = asyncio.new_event_loop()
        self._ws_stop = asyncio.Event()

        def run_loop() -> None:
            asyncio.set_event_loop(self._ws_loop)
            self._ws_loop.run_until_complete(coro_factory())

        self._ws_thread = threading.Thread(target=run_loop, daemon=True)
        self._ws_thread.start()

    def _start_ws_server(self) -> None:
        bridge = self

        async def run_server() -> None:
            async def handler(websocket: ServerConnection) -> None:
                path = websocket.request.path
                if path != bridge._ws_path:
                    bridge.get_logger().warning(
                        f'Rejected WebSocket path {path!r} (expected {bridge._ws_path!r})'
                    )
                    await websocket.close(1008, 'invalid path')
                    return

                with bridge._lock:
                    bridge._ws_peers.add(websocket)
                peer = websocket.remote_address
                bridge.get_logger().info(f'WebSocket client connected from {peer[0]}:{peer[1]}')

                try:
                    await websocket.wait_closed()
                finally:
                    with bridge._lock:
                        bridge._ws_peers.discard(websocket)
                    bridge.get_logger().info('WebSocket client disconnected')

            async with serve(handler, bridge._host, bridge._port):
                bridge.get_logger().info(f'WebSocket server ready at {bridge._ws_url()}')
                assert bridge._ws_stop is not None
                await bridge._ws_stop.wait()

        self._start_ws_loop(run_server)

    def _start_ws_client(self) -> None:
        bridge = self

        async def run_client() -> None:
            while bridge._running:
                try:
                    async with connect(bridge._ws_url()) as websocket:
                        with bridge._lock:
                            bridge._ws_client = websocket
                        bridge.get_logger().info(f'Connected to {bridge._ws_url()}')
                        await websocket.wait_closed()
                except Exception as exc:
                    bridge.get_logger().warning(
                        f'WebSocket connect failed: {exc} '
                        f'(retry every {bridge._reconnect_interval:.1f}s)'
                    )
                finally:
                    with bridge._lock:
                        bridge._ws_client = None

                if not bridge._running:
                    break
                await asyncio.sleep(bridge._reconnect_interval)

        self._start_ws_loop(run_client)

    def _send(self, payload: dict) -> bool:
        if self._ws_loop is None:
            return False

        data = json.dumps(payload, ensure_ascii=False)
        future = asyncio.run_coroutine_threadsafe(self._async_send(data), self._ws_loop)
        try:
            return future.result(timeout=2.0)
        except Exception as exc:
            self.get_logger().warning(f'WebSocket send failed: {exc}')
            return False

    async def _async_send(self, data: str) -> bool:
        if self._mode == 'client':
            return await self._async_send_client(data)
        return await self._async_send_server(data)

    async def _async_send_client(self, data: str) -> bool:
        with self._lock:
            websocket = self._ws_client
        if websocket is None:
            return False
        try:
            await websocket.send(data)
            return True
        except Exception as exc:
            self.get_logger().warning(f'WebSocket send failed: {exc}')
            return False

    async def _async_send_server(self, data: str) -> bool:
        with self._lock:
            peers = list(self._ws_peers)
        if not peers:
            return False

        dead_peers: list[ServerConnection] = []
        sent = False
        for websocket in peers:
            try:
                await websocket.send(data)
                sent = True
            except Exception:
                dead_peers.append(websocket)

        if dead_peers:
            with self._lock:
                for websocket in dead_peers:
                    self._ws_peers.discard(websocket)
            self.get_logger().info('WebSocket client disconnected')

        return sent

    def _gps_callback(self, msg: NavSatFix) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        self._rtk_monitor.add_fix(now, msg.latitude, msg.longitude)

    def _odometry_gps_callback(self, _msg: Odometry) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        self._gps_hz_tracker.add_sample(now)

    def _log_rtk_status(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9

    def _status_to_dict(self, msg: Nav2Status) -> dict:
        stamp = msg.header.stamp
        now = self.get_clock().now().nanoseconds / 1e9
        executing = self._is_navigation_executing(msg)
        laser_active = executing and msg.obstacle_ahead_active
        follow_path_active = executing and msg.follow_path_failed_active
        return {
            'stamp': {
                'sec': stamp.sec,
                'nanosec': stamp.nanosec,
            },
            'frame_id': msg.header.frame_id,
            'nav2_available': msg.nav2_available,
            'navigation_state': msg.navigation_state,
            'task_source': msg.task_source,
            'task_status': msg.task_status,
            'subtask_source': msg.subtask_source,
            'subtask_status': msg.subtask_status,
            'current_waypoint': msg.current_waypoint,
            'in_recovery': msg.in_recovery,
            'recovery_count': msg.recovery_count,
            'failure_category': msg.failure_category,
            'failure_detail': msg.failure_detail,
            'failed_bt_node': msg.failed_bt_node,
            # 拆分字段：便于区分激光障碍 vs 控制器/recovery 失败
            'obstacle_ahead_active': laser_active,
            'obstacle_ahead_detail': 'obstacle_ahead' if laser_active else '',
            'obstacle_ahead_bt_node': 'LaserScan' if laser_active else '',
            'follow_path_failed_active': follow_path_active,
            'follow_path_failed_detail': 'follow_path_failed' if follow_path_active else '',
            'follow_path_failed_bt_node': 'FollowPath' if follow_path_active else '',
            'active_behaviors': list(msg.active_behaviors),
            'rtk_status': self._rtk_monitor.is_stable(now),
            'gps_hz': round(self._gps_hz_tracker.get_hz(), 2),
        }

    def _is_navigation_executing(self, msg: Nav2Status) -> bool:
        return msg.task_status == 'executing'

    def _is_laser_obstacle_voice_active(self, msg: Nav2Status) -> bool:
        return self._is_navigation_executing(msg) and msg.obstacle_ahead_active

    def _is_follow_path_recovery_voice_active(self, msg: Nav2Status) -> bool:
        if not self._follow_path_failed_alert_enabled:
            return False
        return self._is_navigation_executing(msg) and msg.follow_path_failed_active

    def _play_alert_wav(self, wav_path: Path, label: str) -> None:
        if not wav_path.is_file():
            self.get_logger().warning('%s alert wav not found: %s' % (label, wav_path))
            return

        with self._lock:
            if self._alert_audio_playing:
                return
            self._alert_audio_playing = True

        wav = str(wav_path)

        def play() -> None:
            try:
                for player in ('aplay', 'paplay'):
                    try:
                        subprocess.run(
                            [player, '-q', wav],
                            check=True,
                            stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL,
                        )
                        break
                    except (FileNotFoundError, subprocess.CalledProcessError):
                        continue
                else:
                    self.get_logger().warning('No audio player available (aplay/paplay)')
            finally:
                with self._lock:
                    self._alert_audio_playing = False

        threading.Thread(target=play, daemon=True).start()

    def _play_laser_obstacle_alert(self) -> None:
        if not self._obstacle_alert_enabled:
            return
        self._play_alert_wav(self._obstacle_wav_path, 'Laser obstacle')

    def _play_follow_path_recovery_alert(self) -> None:
        if not self._obstacle_alert_enabled:
            return
        self._play_alert_wav(self._obstacle_wav_path, 'FollowPath recovery')

    def _should_play_alert_voice(
        self,
        now: float,
        episode_since: float | None,
        last_play_at: float | None,
        play_count: int,
        repeat_hold_sec: float,
        repeat_interval_sec: float,
        long_episode_sec: float,
    ) -> bool:
        if episode_since is None:
            return False

        if play_count == 0:
            return True

        if (now - episode_since) < repeat_hold_sec:
            return False

        if last_play_at is not None and (now - last_play_at) >= repeat_interval_sec:
            return True

        if long_episode_sec > 0.0:
            milestone = int((now - episode_since) / long_episode_sec)
            if milestone <= 0:
                return False
            if last_play_at is None:
                return (now - episode_since) >= long_episode_sec
            last_milestone = int((last_play_at - episode_since) / long_episode_sec)
            if milestone > last_milestone:
                return True

        return False

    def _step_alert_voice_track(
        self,
        *,
        now: float,
        active: bool,
        since: float | None,
        last_play_at: float | None,
        play_count: int,
        clear_since: float | None,
        grace_sec: float,
        repeat_hold_sec: float,
        repeat_interval_sec: float,
        long_episode_sec: float,
        play_callback,
        log_label: str,
    ) -> tuple[float | None, float | None, int, float | None]:
        if active:
            if since is None:
                since = now
            if (
                not self._alert_audio_playing
                and self._should_play_alert_voice(
                    now,
                    since,
                    last_play_at,
                    play_count,
                    repeat_hold_sec,
                    repeat_interval_sec,
                    long_episode_sec,
                )
            ):
                self.get_logger().info(
                    'Playing %s alert (#%d, episode=%.1fs)'
                    % (log_label, play_count + 1, now - since)
                )
                play_callback()
                last_play_at = now
                play_count += 1
            return since, last_play_at, play_count, None

        if since is None:
            return since, last_play_at, play_count, clear_since

        if clear_since is None:
            return since, last_play_at, play_count, now

        if (now - clear_since) < grace_sec:
            return since, last_play_at, play_count, clear_since

        return None, None, 0, None

    def _update_laser_obstacle_voice(self, msg: Nav2Status) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        self._laser_since, self._laser_last_play_at, self._laser_play_count, self._laser_clear_since = (
            self._step_alert_voice_track(
                now=now,
                active=self._is_laser_obstacle_voice_active(msg),
                since=self._laser_since,
                last_play_at=self._laser_last_play_at,
                play_count=self._laser_play_count,
                clear_since=self._laser_clear_since,
                grace_sec=self._obstacle_voice_clear_grace_sec,
                repeat_hold_sec=self._obstacle_ahead_repeat_hold_sec,
                repeat_interval_sec=self._obstacle_alert_repeat_interval_sec,
                long_episode_sec=self._obstacle_long_episode_play_sec,
                play_callback=self._play_laser_obstacle_alert,
                log_label='laser obstacle',
            )
        )

    def _update_follow_path_recovery_voice(self, msg: Nav2Status) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        (
            self._follow_path_since,
            self._follow_path_last_play_at,
            self._follow_path_play_count,
            self._follow_path_clear_since,
        ) = self._step_alert_voice_track(
            now=now,
            active=self._is_follow_path_recovery_voice_active(msg),
            since=self._follow_path_since,
            last_play_at=self._follow_path_last_play_at,
            play_count=self._follow_path_play_count,
            clear_since=self._follow_path_clear_since,
            grace_sec=self._obstacle_voice_clear_grace_sec,
            repeat_hold_sec=self._obstacle_ahead_repeat_hold_sec,
            repeat_interval_sec=self._obstacle_alert_repeat_interval_sec,
            long_episode_sec=self._obstacle_long_episode_play_sec,
            play_callback=self._play_follow_path_recovery_alert,
            log_label='follow_path recovery',
        )

    def _update_obstacle_voice(self, msg: Nav2Status) -> None:
        self._update_laser_obstacle_voice(msg)
        self._update_follow_path_recovery_voice(msg)

    def _status_callback(self, msg: Nav2Status) -> None:
        self._update_obstacle_voice(msg)
        self._send(self._status_to_dict(msg))

    def destroy_node(self) -> None:
        self._running = False

        if self._ws_loop is not None and self._ws_stop is not None:
            self._ws_loop.call_soon_threadsafe(self._ws_stop.set)

        with self._lock:
            self._ws_peers.clear()
            self._ws_client = None

        if self._ws_thread is not None:
            self._ws_thread.join(timeout=2.0)

        super().destroy_node()


# ==================== 入口 ====================

def main(args=None) -> None:
    parser = argparse.ArgumentParser(description='Nav2 status bridge / GPS RTK monitor')
    parser.add_argument(
        '--gps-monitor',
        action='store_true',
        help='仅运行 RTK 监控模式（等同原 gps_monitor.py）',
    )
    known_args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = GPSMonitor() if known_args.gps_monitor else Nav2StatusSocketBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
