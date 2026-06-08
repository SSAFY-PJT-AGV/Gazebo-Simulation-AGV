import os
import cv2
import numpy as np

# 저장 폴더
OUTPUT_DIR = "models/aruco_markers/images"

# 0~999 ID를 사용할 수 있는 ArUco Dictionary
ARUCO_DICT = cv2.aruco.DICT_4X4_1000

# 테스트용 마커 ID
MARKER_IDS = [900, 901, 902]

# 마커 이미지 크기
MARKER_SIZE = 400


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    dictionary = cv2.aruco.Dictionary_get(ARUCO_DICT)

    for marker_id in MARKER_IDS:
        marker_img = np.zeros((MARKER_SIZE, MARKER_SIZE), dtype=np.uint8)

        cv2.aruco.drawMarker(
            dictionary,
            marker_id,
            MARKER_SIZE,
            marker_img,
            1
        )

        filename = f"marker_{marker_id}.png"
        filepath = os.path.join(OUTPUT_DIR, filename)

        cv2.imwrite(filepath, marker_img)

        print(f"Created: {filepath}")

    print("ArUco marker generation complete.")


if __name__ == "__main__":
    main()