#!/usr/bin/env python3
"""모드 3 예측 노드 확인 — 로봇은 움직이지 않는다. 던질 때마다 flight_predictor 가 무엇을 냈는지 출력.

    source scripts/robot.sh env
    python3 scripts/mode3_predict_check.py            # Ctrl+C 로 끝, 끝에 요약

전제: 패드를 쥔 채 대기 자세(손목 카메라가 투구자 쪽), 터미널 2 가 MODE3=1 로 떠 있음(카메라·flight_predictor).
보는 것 (catch_ball 이 쓰는 토픽 그대로, INTERFACES 2.2):
  /rally/perception/landing_info  [교차 시각, 남은 시간, std u, std w, 관측 수, 놓은 뒤 경과] — 예측 하나마다
  /rally/perception/landing       패드 면 교차 예측점 (base_link)
  /rally/perception/ball_on_pad   측면 카메라가 본 패드 위 공
  카메라 두 대 수신 속도 (역직렬화 없이 raw 로 세서 측정 자체가 부하를 거의 안 줌)
투구 구분: 예측이 1 s 넘게 끊기면 다음 투구.
catch_ball 판정 기준과 같이 표시: 공 놓은 뒤 decide_after_s(0.16 s) 까지 예측이 있었나, std ≤ 0.10 m · 관측 ≥ 4 인가.
"""
import math
import time

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray

DECIDE_S, STD_MAX, NOBS_MIN, GAP_S = 0.16, 0.10, 4, 1.0


def main():
    rclpy.init()
    n = rclpy.create_node("rally_mode3_predict_check")
    st = dict(throw=None, throws=[], rx=[0, 0], pad=0, last_pt=None)

    def on_info(m):
        d = list(m.data)
        if len(d) < 6:
            return
        now = time.monotonic()
        t = st["throw"]
        if t is None or now - t["last"] > GAP_S:
            t = st["throw"] = dict(start=now, last=now, n=0, first_since=d[5], ok_at_decide=False, best=None)
            st["throws"].append(t)
            print(f"\n=== 투구 {len(st['throws'])} — 첫 예측 (놓은 뒤 {d[5]:.2f} s)")
        t["last"], t["n"] = now, t["n"] + 1
        std = max(d[2], d[3])
        good = std <= STD_MAX and d[4] >= NOBS_MIN
        if d[5] <= DECIDE_S + 0.05 and good:
            t["ok_at_decide"] = True
        if t["best"] is None or std < t["best"]:
            t["best"] = std
        pt = st["last_pt"]
        ps = "" if pt is None else f" · 교차점 ({pt[0]:.3f}, {pt[1]:.3f}, {pt[2]:.3f}) m"
        print(f"  놓은 뒤 {d[5]:5.2f} s · 교차까지 {d[1]:5.2f} s · std {d[2] * 100:4.1f}/{d[3] * 100:4.1f} cm · 관측 {d[4]:.0f}"
              f" · {'좋음' if good else '품질 미달'}{ps}")

    def on_landing(m):
        st["last_pt"] = (m.point.x, m.point.y, m.point.z)

    def on_pad(m):
        st["pad"] += 1

    def rx(i):
        def f(_):
            st["rx"][i] += 1
        return f

    n.create_subscription(Float64MultiArray, "/rally/perception/landing_info", on_info, qos_profile_sensor_data)
    n.create_subscription(PointStamped, "/rally/perception/landing", on_landing, qos_profile_sensor_data)
    n.create_subscription(PointStamped, "/rally/perception/ball_on_pad", on_pad, qos_profile_sensor_data)
    n.create_subscription(Image, "/camera/color/image_raw", rx(0), qos_profile_sensor_data, raw=True)
    n.create_subscription(Image, "/side/side/color/image_raw", rx(1), qos_profile_sensor_data, raw=True)
    print("모드 3 예측 확인 — 로봇은 안 움직임. 패드를 향해 던지세요. Ctrl+C 로 끝")
    t_rate = time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.05)
            if time.monotonic() - t_rate >= 5.0:
                dt = time.monotonic() - t_rate
                print(f"[카메라] 손목 {st['rx'][0] / dt:.0f} Hz · 측면 {st['rx'][1] / dt:.0f} Hz · 패드 위 공 {st['pad']}"
                      + ("" if st["rx"][0] and st["rx"][1] else "  ← 카메라 안 들어옴 (터미널 2 MODE3=1?)"))
                st["rx"], t_rate = [0, 0], time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        print("\n===== 요약")
        if not st["throws"]:
            print("예측 0개 — 공을 못 봤거나 두 카메라 짝짓기 실패. flight_predictor 로그의 '수신 · 추론 Hz'·'재투영 탈락' 확인")
        for i, t in enumerate(st["throws"], 1):
            best = "-" if t["best"] is None or math.isinf(t["best"]) else f"{t['best'] * 100:.1f} cm"
            print(f"투구 {i}: 예측 {t['n']}개 · 첫 예측 놓은 뒤 {t['first_since']:.2f} s · 최소 std {best} · "
                  f"0.16 s 안에 좋은 예측 {'있음 → 로봇이 받으러 감' if t['ok_at_decide'] else '없음 → 로봇 안 움직임'}")
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
