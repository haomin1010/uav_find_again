from __future__ import annotations

from datetime import datetime
from pathlib import Path
from typing import Dict, Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImageVideoRecorder(Node):
    def __init__(self) -> None:
        super().__init__("image_video_recorder")
        self.declare_parameter("output_dir", "runs_ros2")
        self.declare_parameter("run_name", "")
        self.declare_parameter("fps", 20.0)
        self.declare_parameter("image_topics", ["/uav_1/front_camera/image"])
        self.declare_parameter("video_names", ["uav_1_front_camera.mp4"])
        self.declare_parameter("no_frame_warn_sec", 5.0)

        output_root = Path(str(self.get_parameter("output_dir").value))
        run_name = str(self.get_parameter("run_name").value)
        if not run_name:
            run_name = datetime.now().strftime("%Y%m%d_%H%M%S_camera_record")
        self.video_dir = output_root / run_name / "videos"
        self.video_dir.mkdir(parents=True, exist_ok=True)

        self.fps = float(self.get_parameter("fps").value)
        self.no_frame_warn_sec = float(self.get_parameter("no_frame_warn_sec").value)
        image_topics = list(self.get_parameter("image_topics").value)
        video_names = list(self.get_parameter("video_names").value)
        if len(video_names) != len(image_topics):
            raise ValueError("video_names length must match image_topics length")

        self.writers: Dict[str, cv2.VideoWriter] = {}
        self.frame_counts: Dict[str, int] = {topic: 0 for topic in image_topics}
        self.last_warn_time = self.get_clock().now()
        self.video_paths = {
            topic: self.video_dir / video_name
            for topic, video_name in zip(image_topics, video_names)
        }

        for topic in image_topics:
            self.create_subscription(Image, topic, lambda msg, t=topic: self.on_image(t, msg), 10)
            self.get_logger().info(f"recording {topic} -> {self.video_paths[topic]}")
        self.create_timer(1.0, self.on_status_timer)

    def on_image(self, topic: str, msg: Image) -> None:
        frame = self.image_to_bgr(msg)
        if frame is None:
            return
        self.frame_counts[topic] = self.frame_counts.get(topic, 0) + 1

        writer = self.writers.get(topic)
        if writer is None:
            height, width = frame.shape[:2]
            path = str(self.video_paths[topic])
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            writer = cv2.VideoWriter(path, fourcc, self.fps, (width, height))
            if not writer.isOpened():
                self.get_logger().error(f"failed to open video writer: {path}")
                return
            self.writers[topic] = writer
            self.get_logger().info(f"opened video writer: {path} ({width}x{height}@{self.fps})")

        writer.write(frame)

    def on_status_timer(self) -> None:
        now = self.get_clock().now()
        elapsed = (now - self.last_warn_time).nanoseconds / 1e9
        if elapsed < self.no_frame_warn_sec:
            return
        self.last_warn_time = now
        missing_topics = [topic for topic, count in self.frame_counts.items() if count == 0]
        if missing_topics:
            self.get_logger().warning(
                "waiting for camera frames on: "
                + ", ".join(missing_topics)
                + ". Check record_video bridge logs and `ros2 topic hz <topic>`."
            )

    def image_to_bgr(self, msg: Image) -> Optional[np.ndarray]:
        if msg.height == 0 or msg.width == 0:
            return None

        encoding = msg.encoding.lower()
        channels = self.encoding_channels(encoding)
        if channels is None:
            self.get_logger().warning(f"unsupported image encoding: {msg.encoding}")
            return None

        expected_step = msg.width * channels
        if msg.step < expected_step:
            self.get_logger().warning(
                f"unexpected image step for {msg.encoding}: step={msg.step}, expected>={expected_step}"
            )
            return None

        raw = np.frombuffer(msg.data, dtype=np.uint8)
        try:
            image = raw.reshape((msg.height, msg.step))[:, :expected_step]
            image = image.reshape((msg.height, msg.width, channels))
        except ValueError as exc:
            self.get_logger().warning(f"failed to reshape image: {exc}")
            return None

        if encoding in ("rgb8", "r8g8b8"):
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if encoding in ("bgr8", "b8g8r8"):
            return image
        if encoding in ("rgba8", "r8g8b8a8"):
            return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
        if encoding in ("bgra8", "b8g8r8a8"):
            return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        if encoding in ("mono8", "8uc1"):
            return cv2.cvtColor(image[:, :, 0], cv2.COLOR_GRAY2BGR)
        self.get_logger().warning(f"unsupported image encoding: {msg.encoding}")
        return None

    @staticmethod
    def encoding_channels(encoding: str) -> Optional[int]:
        if encoding in ("rgb8", "bgr8", "r8g8b8", "b8g8r8"):
            return 3
        if encoding in ("rgba8", "bgra8", "r8g8b8a8", "b8g8r8a8"):
            return 4
        if encoding in ("mono8", "8uc1"):
            return 1
        return None

    def destroy_node(self) -> bool:
        for writer in self.writers.values():
            writer.release()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = ImageVideoRecorder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
