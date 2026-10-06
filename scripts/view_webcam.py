#!/usr/bin/env python3
"""웹캠 화면 확인용. 이름으로 장치를 찾아서 띄운다 (번호가 바뀌어도 OK).

사용: python3 scripts/view_webcam.py [이름 일부, 기본 C270]
종료: q 또는 ESC
"""
import glob
import sys

import cv2


def find_device(keyword: str) -> str:
    for path in sorted(glob.glob("/sys/class/video4linux/video*")):
        with open(f"{path}/name") as f:
            name = f.read().strip()
        with open(f"{path}/index") as f:
            index = int(f.read().strip())
        # 한 카메라가 video 노드를 여러 개 만든다. index 0이 실제 영상 스트림.
        if keyword.lower() in name.lower() and index == 0:
            return f"/dev/{path.rsplit('/', 1)[-1]}"
    sys.exit(f"'{keyword}' 카메라를 못 찾음")


def main() -> None:
    keyword = sys.argv[1] if len(sys.argv) > 1 else "C270"
    device = find_device(keyword)
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    if not cap.isOpened():
        sys.exit(f"{device} 열기 실패")

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        cv2.imshow(f"{keyword} ({device})", frame)
        if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
