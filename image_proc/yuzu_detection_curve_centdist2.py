import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from cv_bridge import CvBridge
from ultralytics import YOLO
import torch
import cv2
import numpy as np
import math
import time

from pathlib import Path
from scipy.interpolate import splprep, splev

# ==========================
# YOLO Model
# ==========================
device = torch.device(
    "cuda:0" if torch.cuda.is_available()
    else "cpu"
)

model = YOLO(
    str(Path(__file__).resolve().parent / "best_yuzu_seg.pt")
).to(device)


class YuzuDetector(Node):
    def __init__(self):
        super().__init__("yuzu_detector")

        self.bridge = CvBridge()
        # ==========================
        # Camera Subscriber
        # ==========================
        self.image_sub = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.image_callback,
            10
        )

        # ==========================
        # Yellow Dots Offset Publisher
        # ==========================
        # Publishes flattened [angle, dx_mm, dy_mm, ...] for the 11 yellow dots
        self.yellow_dots_pub = self.create_publisher(
            Float32MultiArray,
            "/yuzu/yellow_dot_offsets",
            10
        )

        # ==========================
        # Save Result
        # ==========================
        self.save_path = Path(
            "~/Desktop/yuzu_proj/detect_result/1009/radius_result_30deg_1920.txt"
        ).expanduser()

        self.save_path.parent.mkdir(parents=True, exist_ok=True)

        with open(self.save_path, "w") as f:
            f.write("Yuzu radius result\n")

        self.save_count = 0
        self.max_save_count = 5

        cv2.namedWindow(
            "yuzu",
            cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO
        )

        cv2.resizeWindow("yuzu", 1280, 720)

        self.get_logger().info("Yuzu detector start")


    # ==================================================
    # Intersection of the ray and the contour
    # ==================================================
    def get_angle_points(self, cx, cy, contour):
        result = []

        pts = contour[:, 0, :]

        for angle in range(0, 360, 30):
            theta = np.deg2rad(angle - 90)

            dx = np.cos(theta)
            dy = np.sin(theta)

            min_dist = float("inf")
            hit = None

            for i in range(len(pts)):
                x1, y1 = pts[i]
                x2, y2 = pts[(i + 1) % len(pts)]

                vx = x2 - x1
                vy = y2 - y1

                det = (dx * (-vy) - dy * (-vx))

                if abs(det) < 1e-6:
                    continue

                t = ((x1 - cx) * (-vy) - (y1 - cy) * (-vx)) / det
                u = ((x1 - cx) * (-dy) - (y1 - cy) * (-dx)) / det

                if t > 0 and 0 <= u <= 1:
                    x = int(cx + t * dx)
                    y = int(cy + t * dy)

                    dist = np.sqrt((x - cx)**2 + (y - cy)**2)

                    if dist < min_dist:
                        min_dist = dist
                        hit = (x, y)

            if hit is not None:
                result.append((angle, hit))

        return result


    # ==================================================
    # Pink dot at frame center
    # ==================================================
    def draw_frame_center(self, img):
        h, w = img.shape[:2]

        cv2.circle(
            img,
            (w // 2, h // 2),
            7,
            (180, 105, 255),
            -1
        )


    def image_callback(self, msg):
        start = time.perf_counter()

        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        results = model(
            img,
            imgsz=1920,
            conf=0.7,
            verbose=False
        )

        result = results[0]

        if result.masks is None:
            self.draw_frame_center(img)
            cv2.imshow("yuzu", img)
            cv2.waitKey(1)
            return

        for seg in result.masks.xy:
            contour = np.array(
                seg,
                dtype=np.int32
            ).reshape((-1, 1, 2))

            # --------------------------
            # center of gravity (blue dot)
            # --------------------------
            M = cv2.moments(contour)

            if M["m00"] == 0:
                continue

            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            cv2.circle(
                img,
                (cx, cy),
                5,
                (255, 0, 0),
                -1
            )

            # --------------------------
            # 3mm inner contour
            # --------------------------
            mm_per_pixel = 50 / 232
            move_px = 3 / mm_per_pixel

            moved = []

            for p in contour:
                x, y = p[0]

                dx = cx - x
                dy = cy - y

                d = np.sqrt(dx**2 + dy**2)

                if d > 0:
                    nx = int(x + dx / d * move_px)
                    ny = int(y + dy / d * move_px)
                    moved.append([nx, ny])

            moved = np.array(
                moved,
                dtype=np.int32
            ).reshape((-1, 1, 2))

            points = self.get_angle_points(cx, cy, moved)


            elapsed = time.perf_counter() - start

            if self.save_count < self.max_save_count:
                with open(self.save_path, "a") as f:
                    f.write(f"\nDetection {self.save_count+1}\n")

                    for angle, p in points:
                        px, py = p

                        x_mm = (px - cx) * mm_per_pixel
                        y_mm = (cy - py) * mm_per_pixel

                        radius = math.sqrt(x_mm * x_mm + y_mm * y_mm)

                        text = (
                            f"time={elapsed:.6f}s, "
                            f"angle={angle}, "
                            f"radius={radius:.3f}, "
                            f"x={px}, "
                            f"y={py}\n"
                        )

                        f.write(text)
                        print(text.strip())

                self.save_count += 1
                print(f"保存 {self.save_count}/{self.max_save_count}")

            # splprep fails on repeated points, so drop duplicates first
            unique_pts = []

            for _, p in points:
                if p not in unique_pts:
                    unique_pts.append(p)

            tck = None

            if len(unique_pts) >= 4:
                pts = np.array(unique_pts, dtype=np.float32)
                pts = np.vstack([pts, pts[0]])

                x = pts[:, 0]
                y = pts[:, 1]

                try:
                    tck, u = splprep([x, y], s=0, per=True)
                except ValueError:
                    self.get_logger().warn(
                        "Spline fit failed, skipping curve for this frame"
                    )

            if tck is not None:
                u_new = np.linspace(0, 1, 300)
                x_new, y_new = splev(u_new, tck)

                spline_pts = np.vstack((x_new, y_new)).T.astype(np.int32)

                cv2.polylines(
                    img,
                    [spline_pts],
                    True,
                    (255, 0, 255),
                    2
                )

            h, w = img.shape[:2]

            cv2.line(img, (cx, 0), (cx, h), (255, 255, 255), 1)
            cv2.line(img, (0, cy), (w, cy), (255, 255, 255), 1)

            # --------------------------
            # Draw dots, putText XY coords (all 12 dots), & collect 11 yellow dot offsets
            # --------------------------
            yellow_offsets_data = []

            for angle, p in points:
                px, py = p

                # Coordinate from yuzu centroid (blue dot) to each dot
                # +X = right of centroid, +Y = above centroid (in mm)
                dx_mm = float((px - cx) * mm_per_pixel)
                dy_mm = float((cy - py) * mm_per_pixel)

                if angle == 0:
                    color = (0, 0, 255)    # Red dot (0 deg)
                else:
                    color = (0, 255, 255)  # Yellow dots (30..330 deg)
                    yellow_offsets_data.extend([float(angle), dx_mm, dy_mm])

                cv2.circle(img, (px, py), 5, color, -1)

                # Larger text settings for both red and yellow dots
                coord_label = f"({dx_mm:+.1f},{dy_mm:+.1f})"
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.7
                thickness = 2

                (tw, th), _ = cv2.getTextSize(coord_label, font, font_scale, thickness)

                # Push label outward along the ray so larger text doesn't cover the dot
                theta = np.deg2rad(angle - 90)
                offset_dist = 38
                tx = int(px + np.cos(theta) * offset_dist - tw / 2)
                ty = int(py + np.sin(theta) * offset_dist + th / 2)

                # Background rectangle for readability
                cv2.rectangle(
                    img,
                    (tx - 3, ty - th - 3),
                    (tx + tw + 3, ty + 3),
                    (0, 0, 0),
                    -1
                )

                cv2.putText(
                    img,
                    coord_label,
                    (tx, ty),
                    font,
                    font_scale,
                    color,
                    thickness,
                    cv2.LINE_AA
                )

            # Publish the 11 yellow dots on a single topic
            if yellow_offsets_data:
                offset_msg = Float32MultiArray()
                num_dots = len(yellow_offsets_data) // 3
                offset_msg.layout.dim = [
                    MultiArrayDimension(label="yellow_dots", size=num_dots, stride=len(yellow_offsets_data)),
                    MultiArrayDimension(label="angle_dx_dy_mm", size=3, stride=3)
                ]
                offset_msg.data = yellow_offsets_data
                self.yellow_dots_pub.publish(offset_msg)

            cv2.putText(
                img,
                f"saved:{self.save_count}/{self.max_save_count}",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2
            )

            # --------------------------
            # Y offset: red dot (0 deg) -> frame center
            # --------------------------
            for angle, p in points:
                if angle != 0:
                    continue

                px, py = p

                fcx = w // 2
                fcy = h // 2

                y_offset_mm = (fcy - py) * mm_per_pixel

                orange = (0, 165, 255)

                cv2.line(img, (px, py), (px, fcy), orange, 2)
                cv2.line(img, (px, fcy), (fcx, fcy), orange, 1)

                label = f"{y_offset_mm:+.1f} mm"

                tx = px + 10
                ty = (py + fcy) // 2

                (tw, th), _ = cv2.getTextSize(
                    label,
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    2
                )

                cv2.rectangle(
                    img,
                    (tx - 4, ty - th - 6),
                    (tx + tw + 4, ty + 6),
                    (0, 0, 0),
                    -1
                )

                cv2.putText(
                    img,
                    label,
                    (tx, ty),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    orange,
                    2
                )

        self.draw_frame_center(img)

        cv2.imshow("yuzu", img)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    node = YuzuDetector()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
