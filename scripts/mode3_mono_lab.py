#!/usr/bin/env python3
"""모드 3 한 카메라 관측 실험실 — bag 에서 두 카메라 공 검출을 한 번 뽑아 두고(extract), 예측 방식을 바꿔 가며 빠르게 비교(eval). 로봇 없이.

    source scripts/robot.sh env
    # 1) 검출 뽑기 (YOLO, 예측 노드와 같은 모델·conf·StaticFilter). 호스트 venv + numpy 1:
    OMP_NUM_THREADS=1 PYTHONPATH=$HOME/.local/rally_np1:$PYTHONPATH ~/venvs/yolo_bench/bin/python scripts/mode3_mono_lab.py extract \\
        --bag data/bags/mode3_dt_1013_073837 --out data/measure/mono_lab/dets_0738.npz
    # 2) 비교 (순수 파이썬, 몇 초)
    python3 scripts/mode3_mono_lab.py eval --dets data/measure/mono_lab/dets_0738.npz

eval 기준 (실제 교차점은 측정할 수 없으므로 '나중에 다 본 궤적'을 정답으로):
  정답 = 투구마다 두 카메라 3D 점 전체를 한 번에 맞춘 궤적(배치 최소제곱, 중력 + 공기 저항)의 패드 평면 교차점.
  측면 화면 움직임 시작 = 측면 픽셀이 프레임당 6 px 넘게 움직이기 시작한 시각 (실제 투구 시작에 가장 가까운 관측).
  방식마다: 기준(검출 전부 쓴 지금 방식의 놓음 판정) 뒤 0.15·0.20·0.25·0.30 s 시점의 예측 오차(정답과 거리)·std·관측 수, std ≤ 10 cm 첫 시각.
"""
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "rally_vision"))

WRIST_TOPIC, SIDE_TOPIC = "/camera/color/image_raw", "/side/side/color/image_raw"
WRIST_INFO, SIDE_INFO = "/camera/color/camera_info", "/side/side/color/camera_info"
TCP_TOPIC = "/rally/robot/tcp_pose"


# ------------------------------------------------------------------ extract
def extract(opt):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CameraInfo
    from geometry_msgs.msg import PoseStamped
    from ultralytics import YOLO
    from rally_vision.flight_predictor_node import find_file, MODEL_NAMES, image_from_cdr, parse_image_cdr

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=opt.bag, storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    model = YOLO(opt.model or find_file(MODEL_NAMES), task="segment")
    frames = {"side": [], "wrist": []}       # (stamp, [(u, v, conf), ...])
    K = {}
    tcp = []
    js = []                                  # (stamp, q1..q6 rad) — 로봇이 움직인 bag 에서 손목 카메라 자세를 프레임마다
    pend = {"side": None, "wrist": None}
    from sensor_msgs.msg import JointState
    JN = [f"joint_{i}" for i in range(1, 7)]

    def run_pair():
        (ts, bs), (tw, bw) = pend["side"], pend["wrist"]
        res = model.predict([image_from_cdr(bs), image_from_cdr(bw)], imgsz=1024, conf=opt.conf, half=True, verbose=False)
        for name, t, r in (("side", ts, res[0]), ("wrist", tw, res[1])):
            c = [] if r.boxes is None or not len(r.boxes) else \
                [((x1 + x2) / 2, (y1 + y2) / 2, float(cf)) for (x1, y1, x2, y2), cf in zip(r.boxes.xyxy.tolist(), r.boxes.conf.tolist())]
            frames[name].append((t, c))
        pend["side"] = pend["wrist"] = None

    n = 0
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic in (SIDE_TOPIC, WRIST_TOPIC):
            name = "side" if topic == SIDE_TOPIC else "wrist"
            if pend[name] is not None:                 # 짝 없이 같은 카메라가 또 옴 → 혼자라도 돌림 (다른 쪽은 이전 것 재사용 안 함)
                other = "wrist" if name == "side" else "side"
                pend[other] = pend[other] or pend[name]
                run_pair()
                frames[other].pop()                    # 가짜 짝 결과는 버림
            pend[name] = (parse_image_cdr(data)[0], bytes(data))
            if pend["side"] is not None and pend["wrist"] is not None:
                run_pair()
                n += 1
                if n % 500 == 0:
                    print(f"  {n} 쌍", flush=True)
        elif topic in (SIDE_INFO, WRIST_INFO) and topic not in K:
            K[topic] = np.array(deserialize_message(data, CameraInfo).k, float).reshape(3, 3)
        elif topic == "/dsr01/joint_states":
            m = deserialize_message(data, JointState)
            dd = dict(zip(m.name, m.position))
            if all(j in dd for j in JN):
                js.append([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9] + [dd[j] for j in JN])
        elif topic == TCP_TOPIC:
            m = deserialize_message(data, PoseStamped)
            p, q = m.pose.position, m.pose.orientation
            tcp.append([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.x, p.y, p.z, q.x, q.y, q.z, q.w])
    os.makedirs(os.path.dirname(os.path.abspath(opt.out)), exist_ok=True)
    pack = lambda fr: (np.array([t for t, _ in fr]), np.array([len(c) for _, c in fr]),  # noqa: E731
                       np.array([x for _, c in fr for x in c], float).reshape(-1, 3))
    st, sn, sc = pack(frames["side"])
    wt, wn, wc = pack(frames["wrist"])
    np.savez(opt.out, side_t=st, side_n=sn, side_c=sc, wrist_t=wt, wrist_n=wn, wrist_c=wc,
             K_side=K.get(SIDE_INFO), K_wrist=K.get(WRIST_INFO), tcp=np.array(tcp), js=np.array(js))
    print(f"저장 {opt.out}: 측면 {len(st)} · 손목 {len(wt)} 프레임 · tcp_pose {len(tcp)}")


# ------------------------------------------------------------------ eval 공통
def load_dets(path):
    d = np.load(path, allow_pickle=False)

    def unpack(t, n, c):
        out, i = [], 0
        for tt, k in zip(t, n):
            out.append((float(tt), [tuple(x) for x in c[i:i + k]]))
            i += k
        return out
    return d, unpack(d["side_t"], d["side_n"], d["side_c"]), unpack(d["wrist_t"], d["wrist_n"], d["wrist_c"])


def setup(d, calib_path, skills_path, dt_side):
    import yaml
    from rally_vision import projection as pj
    from rally_vision.flight_tracking import SideCalib, projection_matrix
    calib = SideCalib.load(calib_path)
    m3 = yaml.safe_load(open(skills_path))["mode3"]
    he = pj.load_hand_eye(pj.default_hand_eye_path())
    T_tcp_cam = np.array(he["T_tcp_cam_mm"], float)
    T_tcp_cam[:3, 3] /= 1000.0
    tc = d["tcp"][len(d["tcp"]) // 2]                         # 아침 bag: 로봇은 대기 자세에 서 있음
    T_base_tcp = pj.pose_to_matrix(*tc[1:8])
    P_w = projection_matrix(d["K_wrist"], np.linalg.inv(T_base_tcp @ T_tcp_cam))
    Pw_at = lambda t: P_w  # noqa: E731
    if "js" in d.files and len(d["js"]) > 10:                 # 로봇이 움직인 bag: joint_states 보간 + URDF 순기구학 (노드의 tf link_6 와 같은 사슬)
        sys.path.insert(0, str(ROOT / "scripts"))
        from ejt_wrench import Chain
        chain, js = Chain(), d["js"]
        T_fl_tcp = np.eye(4)
        T_fl_tcp[:3, 3] = he["tcp_offset_m"]
        moving = float(np.max(np.ptp(js[:, 1:], axis=0)))

        def Pw_at(t):
            q = np.array([np.interp(t, js[:, 0], js[:, i]) for i in range(1, 7)])
            _, T = chain.fk_frames(q)
            return projection_matrix(d["K_wrist"], np.linalg.inv(T @ T_fl_tcp @ T_tcp_cam))
        P_w = Pw_at(float(np.median(js[:, 0])))
        print(f"손목 카메라 자세: joint_states {len(js)} 개로 프레임마다 (관절 변화 최대 {np.degrees(moving):.0f}°)")
    P_s = projection_matrix(d["K_side"], calib.T_cam_from_base)
    plane_p, plane_n = np.array(m3["pad_center_base_m"], float), np.array(m3["pad_normal"], float)
    return dict(P_w=P_w, Pw_at=Pw_at, P_s=P_s, dt=calib.dt_side_s if dt_side is None else dt_side, k=calib.k_drag,
                plane_p=plane_p, plane_n=plane_n / np.linalg.norm(plane_n))


def picked(frames, static_kw):
    from rally_vision.flight_tracking import StaticFilter
    sf = StaticFilter(**static_kw)
    return [(t, sf.pick(c)) for t, c in frames]


def stream(side, wrist, dt):
    """시각 순서(손목 시계)로 ('side'|'wrist', t, uv) 이벤트."""
    ev = [("side", t + dt, uv) for t, uv in side] + [("wrist", t, uv) for t, uv in wrist]
    ev.sort(key=lambda e: e[1])
    return ev


def stereo_points(ev, S, max_reproj=12.0, max_gap=0.04):
    """지금 노드와 같은 짝짓기 — 결과 [(t, X)]."""
    from rally_vision.flight_tracking import StereoAssociator, WristDet
    a = StereoAssociator(S["P_s"], 0.0, max_gap_s=max_gap, max_reproj_px=max_reproj)
    out = []
    for cam, t, uv in ev:
        if cam == "wrist":
            out += a.add_wrist(WristDet(t=t, uv=uv, P=S["Pw_at"](t)))
        else:
            out += a.add_side(t, uv)
    return sorted(out, key=lambda x: x[0])


# ------------------------------------------------------------------ 정답 궤적 (배치)
def batch_fit(pts, k, t0):
    from scipy.optimize import least_squares
    from rally_vision.flight_predictor import propagate
    ts = np.array([t for t, _ in pts])
    X = np.array([p for _, p in pts])
    v0 = (X[-1] - X[0]) / max(ts[-1] - ts[0], 1e-3)
    x0 = np.r_[X[0], v0]

    def res(s):
        r = []
        x, tp = s.copy(), ts[0]
        for t, p in zip(ts, X):
            if t > tp:
                x = propagate(x, k, t - tp)
                tp = t
            r.append(x[:3] - p)
        return np.concatenate(r)
    sol = least_squares(res, x0, loss="soft_l1", f_scale=0.03)
    return ts[0], sol.x, float(np.sqrt(np.median(sol.fun.reshape(-1, 3) ** 2 * 3)))


def crossing_of(x, t, k, plane_p, plane_n):
    from rally_vision.flight_predictor import _rk4
    h = 0.002
    s_prev = float((x[:3] - plane_p) @ plane_n)
    tc = 0.0
    while tc < 2.0 and s_prev > 0:
        x2 = _rk4(x, k, h)
        s = float((x2[:3] - plane_p) @ plane_n)
        if s <= 0:
            a = s_prev / (s_prev - s)
            return t + tc + a * h, x[:3] + a * (x2[:3] - x[:3])
        x, s_prev, tc = x2, s, tc + h
    return None


# ------------------------------------------------------------------ eval
def evaluate(opt):
    sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "rally_vision"))
    from rally_vision.flight_tracking import ThrowSession
    d, side_f, wrist_f = load_dets(opt.dets)
    S = setup(d, opt.calib, opt.skills, opt.dt)
    kw = dict(cell_px=24.0, window=120, frac=0.5, min_frames=30)
    side, wrist = picked(side_f, kw), picked(wrist_f, kw)
    ev = stream(side, wrist, S["dt"])
    pts = stereo_points(ev, S, opt.reproj)
    print(f"측면 검출 {sum(uv is not None for _, uv in side)}/{len(side)} · 손목 {sum(uv is not None for _, uv in wrist)}/{len(wrist)} · 3D 점 {len(pts)} (dt {S['dt'] * 1000:+.0f} ms)")

    # 투구 구간: 지금 방식 ThrowSession 으로 찾고, 그 구간 3D 점 전체로 정답 궤적
    sess = ThrowSession(plane_p=S["plane_p"], plane_n=S["plane_n"], k_drag=S["k"], min_speed=opt.min_speed)
    throws, cur = [], None
    for t, X in pts:
        st0 = sess.state
        sess.add(t, X)
        if st0 == "idle" and sess.state == "flight":
            cur = dict(t_rel=sess.t_release, pts=[])
            throws.append(cur)
        if sess.state == "flight" and cur is not None:
            cur["pts"].append((t, X))
    side_xy = [(t + S["dt"], uv) for t, uv in side if uv is not None]
    print(f"투구 {len(throws)}")
    rows = []
    for i, th in enumerate(throws):
        if len(th["pts"]) < 6:
            print(f"  {i + 1:2d}: 3D 점 {len(th['pts'])} — 정답 궤적 못 만듦, 건너뜀")
            continue
        t0, x0, rms = batch_fit(th["pts"], S["k"], th["pts"][0][0])
        gt = crossing_of(x0, t0, S["k"], S["plane_p"], S["plane_n"])
        if gt is None:
            print(f"  {i + 1:2d}: 정답 궤적이 패드 평면을 안 지남, 건너뜀")
            continue
        # 측면 화면 움직임 시작
        mv = None
        near = [(t, uv) for t, uv in side_xy if th["t_rel"] - 0.5 < t <= th["t_rel"] + 0.05]
        for (ta, a), (tb, b) in zip(near, near[1:]):
            if tb - ta < 0.04 and np.hypot(b[0] - a[0], b[1] - a[1]) > 6.0:
                mv = ta
                break
        rows.append(dict(i=i + 1, t_rel=th["t_rel"], mv=mv, gt=gt, rms=rms, n=len(th["pts"])))
        print(f"  {i + 1:2d}: 3D 점 {len(th['pts']):3d} · 맞춤 잔차 {rms * 100:.1f} cm · 교차 {np.round(gt[1], 3).tolist()} "
              f"(패드에서 {np.linalg.norm(gt[1] - S['plane_p']) * 100:.0f} cm) · 놓음 판정 − 측면 움직임 = "
              f"{'-' if mv is None else f'{(th['t_rel'] - mv) * 1000:+.0f} ms'}")
    np.save(os.path.join(os.path.dirname(opt.dets), "throws_gt.npy"), np.array(rows, dtype=object), allow_pickle=True)
    rows = [r for r in rows if r["rms"] < 0.02 and np.linalg.norm(r["gt"][1] - S["plane_p"]) < 0.6]
    print(f"비교에 쓰는 투구 (맞춤 잔차 < 2 cm · 패드에서 60 cm 안): {[r['i'] for r in rows]}")
    if opt.wrist_drop_early > 0:                # 실로봇 10/13 처럼 던지는 순간 손목이 공을 놓침 — 놓음 판정 앞 0.2 s ~ 뒤 wrist_drop_early s 손목 검출 지움
        wins = [(r["t_rel"] - 0.2, r["t_rel"] + opt.wrist_drop_early) for r in rows]
        wrist = [(t, None if any(a <= t <= b for a, b in wins) else uv) for t, uv in wrist]
        ev = stream(side, wrist, S["dt"])
        print(f"던지는 순간 손목 검출 지움 (놓음 −0.2 ~ +{opt.wrist_drop_early:.2f} s) → 3D 점 {len(stereo_points(ev, S, opt.reproj))}")
    if opt.wrist_keep < 1.0:                    # 실로봇 10/13 오전처럼 손목 검출이 드문 상황 흉내 — 손목 검출을 무작위로 지움
        rng = np.random.default_rng(opt.seed)
        wrist = [(t, uv if uv is None or rng.random() < opt.wrist_keep else None) for t, uv in wrist]
        ev = stream(side, wrist, S["dt"])
        print(f"손목 검출 {opt.wrist_keep * 100:.0f} % 만 남김 → {sum(uv is not None for _, uv in wrist)} 프레임 · 3D 점 {len(stereo_points(ev, S, opt.reproj))}")
    for name, runner in METHODS.items():
        if opt.methods and name not in opt.methods:
            continue
        report(name, runner(ev, S, opt), rows, S)


def report(name, preds, rows, S):
    """preds: [(t, point, std(2), n_obs, t_release)] 시간순."""
    marks = (0.15, 0.20, 0.25, 0.30, 0.35)
    tab = {m: [] for m in marks}
    first_good = []
    for r in rows:
        ref = r["t_rel"]                                # 손목 검출을 다 쓴 지금 방식의 놓음 판정 (투구마다 고정 기준)
        ps = [p for p in preds if ref - 0.1 <= p[0] <= r["gt"][0]]
        g = [p[0] - ref for p in ps if max(p[2]) <= 0.10 and p[3] >= 4]
        first_good.append(min(g) if g else np.nan)
        for m in marks:
            q = [p for p in ps if p[0] <= ref + m]
            if q:
                p = q[-1]
                tab[m].append((np.linalg.norm(p[1] - r["gt"][1]), max(p[2]), p[3], r["gt"][0] - p[0]))
            else:
                tab[m].append((np.nan, np.nan, np.nan, np.nan))
    # 받기 스킬 결정 흉내 (catch_ball: 각자 놓음 판정 뒤 decide_after 0.25 s 에 std ≤ 13 cm 면 결정, 아니면 0.40 s 까지 기다렸다 결정·미달이면 안 움직임)
    dec = []
    for r in rows:
        ref = r["t_rel"]
        ps = [p for p in preds if ref - 0.3 <= p[0] <= r["gt"][0]]
        hit = None
        for p in ps:
            since = p[0] - p[4]
            if since >= 0.25 and (max(p[2]) <= 0.13 or since >= 0.40):
                hit = p
                break
        if hit is None:
            dec.append((np.nan, np.nan, np.nan, False))
        else:
            dec.append((hit[0] - ref, np.linalg.norm(hit[1] - r["gt"][1]), r["gt"][0] - hit[0], max(hit[2]) <= 0.13))
    # 다른 규칙: 예측 교차까지 남은 시간이 go_s 이하가 되는 첫 예측에서 결정 (느린 투구는 더 오래 보고 결정)
    for go_s in (0.55, 0.45):
        dd = []
        for r in rows:
            ref = r["t_rel"]
            ps = [p for p in preds if ref - 0.3 <= p[0] <= r["gt"][0]]
            hit = next((p for p in ps if p[0] - p[4] >= 0.25 and max(p[2]) <= 0.13 and (p[5] - p[0] <= go_s or p[0] - p[4] >= 0.6)), None) \
                if ps and len(ps[0]) > 5 else None
            dd.append(np.nan if hit is None else np.linalg.norm(hit[1] - r["gt"][1]))
            dd[-1] = (dd[-1], np.nan if hit is None else hit[0] - ref, np.nan if hit is None else r["gt"][0] - hit[0])
        e = np.array([x[0] for x in dd])
        print(f"  [규칙: 놓은 뒤 0.25 s 이상 · std ≤ 13 cm · 예측 교차까지 {go_s:.2f} s 이하(또는 놓은 뒤 0.6 s)] 결정 {int(np.sum(np.isfinite(e)))}/{len(dd)} · 오차 투구별 {[round(x * 100) if np.isfinite(x) else None for x in e]} cm · "
              f"결정 시각 중앙 {np.nanmedian([x[1] for x in dd]) if np.any(np.isfinite(e)) else np.nan:.2f} s · 실제 남은 시간 중앙 {np.nanmedian([x[2] for x in dd]) if np.any(np.isfinite(e)) else np.nan:.2f} s")
    med = lambda v: np.nanmedian(v) if np.any(np.isfinite(v)) else np.nan  # noqa: E731
    dt_ = np.array([x[0] for x in dec]); de = np.array([x[1] for x in dec]); dr = np.array([x[2] for x in dec])
    ok = sum(1 for x in dec if x[3])
    print(f"\n=== {name}  결정(받기 스킬 규칙): 움직임 {ok}/{len(dec)} · 결정 시각(진짜 놓음 기준) 중앙 {med(dt_):.2f} s · "
          f"결정 때 오차 중앙 {med(de) * 100:.1f} cm (투구별 {[round(x * 100) if np.isfinite(x) else None for x in de]}) · 결정 뒤 교차까지 중앙 {med(dr):.2f} s")
    print(f"=== {name}  (기준 = 검출 전부 쓴 지금 방식 놓음 판정, 투구 {len(rows)})")
    print(f"  좋은 예측(std ≤ 10 cm) 첫 시각 중앙 {med(first_good):.3f} s · 못 낸 투구 {int(np.sum(~np.isfinite(first_good)))}")
    for m in marks:
        e = np.array([x[0] for x in tab[m]])
        s = np.array([x[1] for x in tab[m]])
        n = np.array([x[2] for x in tab[m]])
        ttg = np.array([x[3] for x in tab[m]])
        print(f"  +{m:.2f} s: 예측 있음 {int(np.sum(np.isfinite(e)))}/{len(e)} · 오차 중앙 {med(e) * 100:5.1f} cm (90% {np.nanpercentile(e, 90) * 100 if np.any(np.isfinite(e)) else np.nan:5.1f})"
              f" · std 중앙 {med(s) * 100:5.1f} cm · 관측 중앙 {med(n):4.1f} · 교차까지 {med(ttg):.2f} s")


def run_current(ev, S, opt):
    """지금 노드와 같음: 짝지은 3D 점만 ThrowSession 에."""
    from rally_vision.flight_tracking import ThrowSession
    sess = ThrowSession(plane_p=S["plane_p"], plane_n=S["plane_n"], k_drag=S["k"], min_speed=opt.min_speed)
    out = []
    for t, X in stereo_points(ev, S, opt.reproj):
        c = sess.add(t, X)
        if c is not None:
            out.append((t, c.point, c.std_m, sess.fp.n, sess.t_release, c.t))
    return out


def run_mono(ev, S, opt):
    """짝지은 3D 점 + 한 카메라 픽셀 관측(광선)을 같은 EKF 에. 시작 때 직전 측면 픽셀로 놓음 시각을 앞당김."""
    from rally_vision.flight_tracking import MonoThrowSession
    sess = MonoThrowSession(plane_p=S["plane_p"], plane_n=S["plane_n"], k_drag=S["k"], min_speed=opt.min_speed,
                            P_side=S["P_s"], pix_std=opt.pix_std)
    from rally_vision.flight_tracking import StereoAssociator, WristDet
    a = StereoAssociator(S["P_s"], 0.0, max_gap_s=0.04, max_reproj_px=opt.reproj)
    out = []
    for cam, t, uv in ev:
        res = []
        if cam == "wrist":
            Pw = S["Pw_at"](t)
            res += sess.add_ray(t, Pw, uv, "wrist")
            res += [("3d", tt, X) for tt, X in a.add_wrist(WristDet(t=t, uv=uv, P=Pw))]
        else:
            res += sess.add_ray(t, S["P_s"], uv, "side")
            res += [("3d", tt, X) for tt, X in a.add_side(t, uv)]
        for kind, tt, X in sorted([r for r in res if r[0] == "3d"], key=lambda r: r[1]):
            sess.add(tt, X)
        c = sess.latest()
        if c is not None:
            out.append((t, c.point, c.std_m, sess.fp.n, sess.t_release, c.t))
    return out


def run_ray_notrim(ev, S, opt):
    return run_ray(ev, S, opt, lead_px=1e9)


def run_ray(ev, S, opt, **kw):
    """3D 점 없이 카메라별 픽셀·시각으로 시작 판정(배치 맞춤)부터 EKF 까지 (flight_rays.RayThrowSession)."""
    from rally_vision.flight_rays import RayThrowSession
    sess = RayThrowSession(plane_p=S["plane_p"], plane_n=S["plane_n"], k_drag=S["k"], **kw)
    out = []
    for cam, t, uv in ev:
        sess.add(t, cam, S["P_s"] if cam == "side" else S["Pw_at"](t), uv)
        c = sess.latest()
        if c is not None:
            out.append((t, c.point, c.std_m, sess.fp.n, sess.t_release, c.t))
    print(f"  (배치 맞춤 {sess.n_fit} 번)")
    return out


METHODS = {"지금 (3D 점만)": run_current, "한 카메라 관측 추가": run_mono, "타임스탬프 (앞쪽 자르기 없음)": run_ray_notrim,
           "타임스탬프만 (3D 점 없음)": run_ray}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    e = sub.add_parser("extract")
    e.add_argument("--bag", required=True)
    e.add_argument("--out", required=True)
    e.add_argument("--model", default="")
    e.add_argument("--conf", type=float, default=0.15)
    v = sub.add_parser("eval")
    v.add_argument("--dets", required=True)
    v.add_argument("--calib", default=str(ROOT / "models" / "mode3_side_calib.json"))
    v.add_argument("--skills", default=str(ROOT / "ros2_ws" / "src" / "rally_bringup" / "config" / "skills.yaml"))
    v.add_argument("--dt", type=float, default=None, help="측면 → 손목 시계 s (기본 calib)")
    v.add_argument("--reproj", type=float, default=12.0)
    v.add_argument("--min-speed", type=float, default=2.5)
    v.add_argument("--pix-std", type=float, default=6.0)
    v.add_argument("--wrist-keep", type=float, default=1.0, help="손목 검출을 이 비율만 남김 (실로봇처럼 드문 상황 흉내)")
    v.add_argument("--seed", type=int, default=0)
    v.add_argument("--wrist-drop-early", type=float, default=0.0, help="던지는 순간 손목 검출을 놓음 뒤 이 s 까지 지움")
    v.add_argument("--methods", nargs="*", default=None)
    opt = ap.parse_args()
    extract(opt) if opt.cmd == "extract" else evaluate(opt)


if __name__ == "__main__":
    main()
