#!/usr/bin/env bash
# 모드 3 YOLO 추론 지연 시험 — 로봇·인식 PC(정정인 노트북)에서 실행. 로봇은 움직이지 않는다.
# 담당: 박성현 (10/11)
#
# 사용법 (터미널에 그대로 붙여넣기 가능):
#   bash robotpc_yolo_bench.sh            # 1·2·3단계 (더미 영상으로 .pt / TensorRT 속도)
#   LIVE=1 bash robotpc_yolo_bench.sh     # + 4단계: RealSense 2대 실영상 (ROS realsense 노드는 먼저 끄기)
#
# 모델: ~/rally/models/velcro_ball_yolo11s-seg_1024_test.pt (박성현 PC 에서 USB 로 복사).
#   없으면 같은 구조의 COCO yolo11s-seg.pt 로 속도만 잰다 (지연은 구조·입력 크기로 정해져 거의 같음. 공 검출 확인은 못 함).
# 결과: ~/rally/data/mode3_bench_<날짜시각>/ — summary.txt 를 박성현에게 보내기.
set -e
MODEL=${MODEL:-$HOME/rally/models/velcro_ball_yolo11s-seg_1024_test.pt}
IMGSZ=${IMGSZ:-1024}
LIVE=${LIVE:-0}
VENV=$HOME/venvs/yolo_bench
OUT=$HOME/rally/data/mode3_bench_$(date +%m%d_%H%M)
mkdir -p "$OUT"

echo "[1/4] 파이썬 환경 ($VENV)"
if [ ! -x "$VENV/bin/python" ]; then
  python3 -m venv --system-site-packages "$VENV" || { echo "venv 실패 → sudo apt install python3.12-venv 후 다시"; exit 1; }
fi
"$VENV/bin/pip" install -q --upgrade pip >/dev/null
"$VENV/bin/pip" install -q "ultralytics>=8.3" "numpy<2" "opencv-python<4.12" onnx onnxslim tensorrt pyrealsense2 2>&1 | tail -3
nvidia-smi --query-gpu=name,driver_version,temperature.gpu,power.draw --format=csv | tee "$OUT/gpu_before.csv"

MODEL="$MODEL" IMGSZ="$IMGSZ" LIVE="$LIVE" OUT="$OUT" "$VENV/bin/python" - <<'PY'
import json, os, time
from pathlib import Path
import numpy as np, torch
from ultralytics import YOLO

model_path, imgsz, live, out = os.environ["MODEL"], int(os.environ["IMGSZ"]), os.environ["LIVE"] == "1", Path(os.environ["OUT"])
res = {"gpu": torch.cuda.get_device_name(0), "torch": torch.__version__, "cuda": torch.version.cuda, "imgsz": imgsz}
if not Path(model_path).exists():
    print(f"!! {model_path} 없음 → COCO yolo11s-seg.pt 로 속도만 잰다")
    model_path = "yolo11s-seg.pt"
res["model"] = model_path
rng = np.random.default_rng(0)
imgs = [rng.integers(0, 255, (480, 848, 3), dtype=np.uint8) for _ in range(2)]   # 카메라 2대 = 848×480

def bench(m, batch, n=200):
    src = imgs[:batch] if batch > 1 else imgs[0]
    for _ in range(20):
        m.predict(src, imgsz=imgsz, half=True, verbose=False)
    t = []
    for _ in range(n):
        torch.cuda.synchronize(); t0 = time.perf_counter()
        m.predict(src, imgsz=imgsz, half=True, verbose=False)
        torch.cuda.synchronize(); t.append((time.perf_counter() - t0) * 1000)
    t = np.array(t)
    return {"mean_ms": round(t.mean(), 2), "p50_ms": round(np.median(t), 2), "p95_ms": round(np.percentile(t, 95), 2), "max_ms": round(t.max(), 2)}

print("[2/4] .pt (PyTorch fp16) 속도")
m = YOLO(model_path)
res["pt_batch1"] = bench(m, 1); print("  1장:", res["pt_batch1"])
res["pt_batch2"] = bench(m, 2); print("  2장 묶음:", res["pt_batch2"])
res["pt_seq2_ms"] = round(2 * res["pt_batch1"]["mean_ms"], 2)

print("[3/4] TensorRT 변환 (처음 한 번 수 분) + 속도")
t0 = time.time()
try:
    eng = YOLO(model_path).export(format="engine", half=True, imgsz=imgsz, batch=2, device=0, verbose=False)
    res["engine_export_s"] = round(time.time() - t0, 1); res["engine"] = str(eng)
    me = YOLO(eng, task="segment")
    res["engine_batch2"] = bench(me, 2); print("  2장 묶음:", res["engine_batch2"])
except Exception as e:
    res["engine_error"] = repr(e)[:400]; print("  !! TensorRT 실패:", res["engine_error"])

if live:
    print("[4/4] RealSense 2대 실영상 (공을 손목·측면 카메라 앞에서 움직여 주세요, 약 15 s)")
    import pyrealsense2 as rs, cv2
    use = me if "engine_batch2" in res else m
    serials = {"wrist": "140122076335", "side": "147122075430"}
    pipes = {}
    for name, sn in serials.items():
        p, c = rs.pipeline(), rs.config()
        c.enable_device(sn); c.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 60)
        prof = p.start(c)
        s = prof.get_device().first_color_sensor()
        s.set_option(rs.option.enable_auto_exposure, 0); s.set_option(rs.option.exposure, 20); s.set_option(rs.option.gain, 128)
        s.set_option(rs.option.global_time_enabled, 1)
        pipes[name] = p
    lat, inf, det, shots = [], [], {k: 0 for k in serials}, 0
    for i in range(900):
        fr = {k: p.wait_for_frames().get_color_frame() for k, p in pipes.items()}
        cap = min(f.get_timestamp() for f in fr.values())        # 촬영 시각 (global time, ms)
        frames = [np.asanyarray(fr[k].get_data()) for k in serials]
        t0 = time.time() * 1000
        r = use.predict(frames, imgsz=imgsz, half=True, conf=0.25, verbose=False)
        t1 = time.time() * 1000
        if i >= 30:
            inf.append(t1 - t0); lat.append(t1 - cap)
            for k, rr in zip(serials, r):
                det[k] += int(len(rr.boxes) > 0)
                if len(rr.boxes) and shots < 6 and i % 20 == 0:
                    cv2.imwrite(str(out / f"det_{k}_{i}.jpg"), rr.plot()); shots += 1
    for p in pipes.values():
        p.stop()
    lat, inf = np.array(lat), np.array(inf)
    res["live"] = {"frames": len(lat), "infer_mean_ms": round(inf.mean(), 2), "infer_p95_ms": round(np.percentile(inf, 95), 2),
                   "capture_to_result_mean_ms": round(lat.mean(), 2), "capture_to_result_p95_ms": round(np.percentile(lat, 95), 2),
                   "ball_frames": det}
    print("  ", res["live"])
else:
    print("[4/4] 실영상 건너뜀 (LIVE=1 로 실행하면 함)")

(out / "result.json").write_text(json.dumps(res, ensure_ascii=False, indent=1))
budget = 1000 / 60
best = res.get("engine_batch2", res["pt_batch2"])["p95_ms"]
lines = [f"GPU {res['gpu']} · 모델 {Path(res['model']).name} · 입력 {imgsz}",
         f".pt 1장 {res['pt_batch1']['mean_ms']} ms · 2장 묶음 {res['pt_batch2']['mean_ms']} ms (p95 {res['pt_batch2']['p95_ms']})",
         f"TensorRT 2장 묶음 {res['engine_batch2']['mean_ms']} ms (p95 {res['engine_batch2']['p95_ms']})" if "engine_batch2" in res else f"TensorRT 실패: {res.get('engine_error')}",
         f"60 fps 예산 {budget:.1f} ms 안? → {'예' if best < budget else '아니오'} (p95 {best} ms)"]
if "live" in res:
    L = res["live"]; lines.append(f"실영상: 추론 {L['infer_mean_ms']} ms · 촬영→결과 {L['capture_to_result_mean_ms']} ms (p95 {L['capture_to_result_p95_ms']}) · 공 검출 프레임 {L['ball_frames']} / {L['frames']}")
(out / "summary.txt").write_text("\n".join(lines) + "\n")
print("\n=== 요약 (박성현에게 보내기) ===\n" + "\n".join(lines) + f"\n결과 폴더: {out}")
PY
nvidia-smi --query-gpu=temperature.gpu,power.draw --format=csv | tee "$OUT/gpu_after.csv"
