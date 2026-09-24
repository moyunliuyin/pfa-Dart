import argparse
from pathlib import Path

import cv2
import numpy as np

from app_config import PROJECT_ROOT, load_config
from opencv_green_detection import detect_green_targets, get_debug_images, set_detection_config


class ImageDebugTool:
    def __init__(self) -> None:
        self.original_image: np.ndarray | None = None
        self.last_result_image: np.ndarray | None = None

    def load_image(self, image_path: str) -> bool:
        image = cv2.imread(image_path)
        if image is None:
            print(f"无法读取图片: {image_path}")
            return False
        self.original_image = image
        print(f"已加载图片: {image_path}")
        return True

    def render_overlays(self, frame_bgr: np.ndarray, targets: list[dict]) -> np.ndarray:
        result = frame_bgr.copy()
        if not targets:
            return result

        colors = {"near": (0, 255, 0), "far": (0, 165, 255)}
        lines = [f"Target Count: {len(targets)}"]
        for target in targets:
            x, y, w, h = target["bbox"]
            center = target["center"]
            circle = target.get("circle")
            metrics = target.get("metrics", {})
            color = colors.get(target["role"], (255, 255, 0))

            cv2.rectangle(result, (x, y), (x + w, y + h), color, 2)
            cv2.drawContours(result, [target["contour"]], -1, color, 2)
            cv2.circle(result, center, 4, (0, 0, 255), -1)
            if circle is not None:
                cv2.circle(result, (circle[0], circle[1]), circle[2], (0, 255, 255), 2)
            cv2.putText(result, target["role"].upper(), (x, max(20, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            lines.append(
                f"{target['role'].upper()} xy={center} area={target['area']:.1f} score={metrics.get('score', 0)}"
            )
            lines.append(
                f"{target['role'].upper()} bright={metrics.get('mean_brightness', 0)} circ={metrics.get('circularity', 0)}"
            )

        y_pos = 30
        for line in lines:
            cv2.putText(result, line, (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            y_pos += 28
        return result

    def run(self, image_path: str) -> None:
        if not self.load_image(image_path):
            return

        assert self.original_image is not None
        rgb_image = cv2.cvtColor(self.original_image, cv2.COLOR_BGR2RGB)
        targets = detect_green_targets(rgb_image)
        debug_mask, _ = get_debug_images()

        self.last_result_image = self.render_overlays(self.original_image, targets)
        cv2.imshow("Original Image", self.original_image)
        cv2.imshow("Detection Result", self.last_result_image)
        if debug_mask is not None:
            cv2.imshow("HSV Mask", debug_mask)

        if targets:
            print(f"检测命中 {len(targets)} 个目标")
            for target in targets:
                print(
                    f"  - {target['role']}: xy={target['center']} offset={target.get('offset')} "
                    f"area={target['area']:.1f}, metrics={target.get('metrics', {})}"
                )
        else:
            print("未检测到目标")

        cv2.waitKey(0)
        cv2.destroyAllWindows()


def main() -> None:
    parser = argparse.ArgumentParser(description="单图绿光检测调试工具")
    parser.add_argument("image", help="图片路径")
    args = parser.parse_args()

    config = load_config(PROJECT_ROOT / "config.yaml")
    set_detection_config(config.get("detection", {}))
    ImageDebugTool().run(args.image)


if __name__ == "__main__":
    main()
