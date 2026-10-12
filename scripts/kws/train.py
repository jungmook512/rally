#!/usr/bin/env python3
"""비상어("그만"·"멈춰") + 호출어("랠리야") 키워드 판별기 학습·평가 (소규모 시험용).

구조: openWakeWord 의 음성 특징 추출기(멜 스펙트로그램 → 96 차원 임베딩, 80 ms 마다)는 그대로 쓰고,
      마지막 16 프레임(1.28 s)을 보고 "그만"/"멈춰"/"랠리야"/아님 을 고르는 작은 판별기만 우리 녹음으로 학습한다.
      출력은 둘로 묶어 쓴다: 비상 = P(그만)+P(멈춰) → 정지, 호출 = P(랠리야) → 녹음·STT 시작.
      랠리야 녹음이 8개 미만이면 비상어만 학습한다.
      openWakeWord 자체 학습도 같은 구조 (특징 추출기는 고정, 끝단만 학습).
평가:
  - 키워드 인식률: 일부를 떼어 두고(학습에 안 씀) 맞히는 비율. "랠리야" 가 정지로, "그만" 이 호출로 잘못 잡히는지도
  - 오탐: 주변 소리(ambient)를 80 ms 씩 밀며 흘려 봤을 때 비상·호출 각각 시간당 몇 번 울리나 (학습에 안 쓴 구간)
  - 지연: 비상어 말이 끝난 뒤(녹음 음량으로 잡음, 클립 끝 아님) 판별까지 걸리는 시간 (80 ms 단위)

사용:
    python3 scripts/kws/train.py                  # 학습 + 평가 + models/kws/emergency_kws.joblib 저장
    python3 scripts/kws/train.py --threshold 0.8
데이터: data/datasets/kws/{geuman,meomchwo,rally,other,ambient}/*.wav (scripts/kws/record.py)
"""
import argparse
import random
import wave
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
DATA = ROOT / "data" / "datasets" / "kws"
OUT = ROOT / "models" / "kws"
RATE = 16000
WIN = 16                      # 판별에 쓰는 임베딩 프레임 수 (16 × 80 ms = 1.28 s)
CLIP = int(2.0 * RATE)        # 특징을 뽑을 때 쓰는 길이 (앞에 0.5 s 여유를 붙여 1.28 s 창이 꽉 차게)
FRAME_S = 1280 / RATE         # 판정 한 번 = 80 ms
KEYWORDS = ["geuman", "meomchwo", "rally"]   # 라벨 번호 1, 2, 3 (0 = 아님)
EMERGENCY = ("geuman", "meomchwo")
WAKE = ("rally",)


def read_wav(p: Path) -> np.ndarray:
    with wave.open(str(p)) as w:
        return np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16).astype(np.float32)


def is_truncated(p: Path, edge_s: float = 0.03) -> bool:
    """말소리가 클립 맨 앞부터 이미 있으면 "지금!" 보다 먼저 말해 앞부분이 잘렸을 가능성이 큼 → 학습에서 뺀다."""
    x = read_wav(p)
    peak = np.abs(x).max()
    if peak < 1:
        return False
    win = max(1, int(0.01 * RATE))
    env = np.convolve(np.abs(x), np.ones(win) / win, mode="same")
    above = np.where(env > peak * 0.15)[0]
    return len(above) > 0 and above[0] < edge_s * RATE


def word_end_s(x: np.ndarray) -> float:
    """녹음 안에서 말이 끝난 시각 (10 ms 음량이 최고의 20 % 를 넘는 마지막 지점). 클립 끝이 아니라 이걸 지연 기준으로."""
    env = np.convolve(np.abs(x), np.ones(160) / 160, mode="same")
    return float(np.where(env >= 0.2 * env.max())[0][-1]) / RATE


def fit_len(x: np.ndarray, n: int, offset: int = 0) -> np.ndarray:
    """길이 n 으로 맞춤. offset 만큼 뒤로 밀고(앞을 0 으로), 넘치면 앞을 자름."""
    y = np.concatenate([np.zeros(max(0, offset), np.float32), x])
    if len(y) >= n:
        return y[-n:]
    return np.concatenate([np.zeros(n - len(y), np.float32), y])


def augment(x: np.ndarray, noises: list[np.ndarray], rng: random.Random) -> np.ndarray:
    """위치 이동 · 크기 · 주변 소음 섞기 (SNR 5~25 dB). 실제 환경 흔들림을 흉내."""
    y = fit_len(x, CLIP, offset=rng.randint(0, int(0.35 * RATE)))
    y = y * rng.uniform(0.5, 1.5)
    if noises and rng.random() < 0.8:
        n = rng.choice(noises)
        s = rng.randint(0, max(0, len(n) - CLIP))
        n = n[s:s + CLIP]
        if len(n) == CLIP:
            ps, pn = np.mean(y ** 2) + 1e-6, np.mean(n ** 2) + 1e-6
            snr = rng.uniform(5, 25)
            y = y + n * np.sqrt(ps / (pn * 10 ** (snr / 10)))
    return np.clip(y, -32768, 32767)


def embed(feat, clips: list[np.ndarray]) -> np.ndarray:
    """(N, CLIP) → (N, WIN*96). 마지막 WIN 프레임."""
    X = np.stack(clips).astype(np.int16)
    e = feat.embed_clips(X, batch_size=64)              # (N, frames, 96)
    return e[:, -WIN:, :].reshape(len(clips), -1)


def group_score(proba: np.ndarray, labels: list[str], group: tuple[str, ...]) -> np.ndarray:
    """(N, 클래스) 확률 → 묶음(비상·호출) 확률 합. 묶음 라벨이 학습에 없으면 0."""
    idx = [labels.index(k) for k in group if k in labels]
    return proba[:, idx].sum(axis=1) if idx else np.zeros(len(proba))


def stream_proba(feat, clf, audio: np.ndarray) -> np.ndarray:
    """긴 소리를 80 ms(1280 샘플)씩 흘려 넣으며 매 프레임 클래스 확률 (프레임, 클래스)."""
    feat.reset()
    out = []
    for i in range(0, len(audio) - 1280, 1280):
        feat(audio[i:i + 1280].astype(np.int16))
        f = feat.get_features(WIN)                       # (1, WIN, 96)
        out.append(clf.predict_proba(f.reshape(1, -1))[0])
    return np.array(out)


def count_hits(score: np.ndarray, threshold: float, consec: int = 1) -> int:
    """consec 프레임 연속으로 임계를 넘어야 한 번. 1 s 안 재발은 무시 (kws.py KwsDecider 와 같은 규칙)."""
    hits, last, run = 0, -99, 0
    for i, s in enumerate(score):
        run = run + 1 if s >= threshold else 0
        if run >= consec and i - last > 12:
            hits += 1
            last = i
    return hits


def first_hit(score: np.ndarray, threshold: float, consec: int = 1) -> int | None:
    """처음으로 consec 프레임 연속 임계를 넘은 프레임 번호."""
    run = 0
    for i, s in enumerate(score):
        run = run + 1 if s >= threshold else 0
        if run >= consec:
            return i
    return None


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--threshold", type=float, default=0.7)
    ap.add_argument("--aug", type=int, default=15, help="키워드 녹음 하나당 만드는 변형 수")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--mine", type=int, default=3, help="오탐 구간 재학습 반복 수 (학습용 주변 소리만 씀, 0 = 끔)")
    ap.add_argument("--consec", type=int, default=1, help="몇 프레임(80 ms) 연속 임계를 넘어야 울릴지. 모델에 저장돼 kws.py 가 씀")
    ap.add_argument("--out", default="emergency_kws.joblib", help="models/kws/ 아래 저장 이름")
    ap.add_argument("--hidden", type=int, default=64, help="판별기 은닉층 크기")
    a = ap.parse_args()
    rng = random.Random(a.seed)
    from openwakeword.utils import AudioFeatures
    from sklearn.neural_network import MLPClassifier
    from sklearn.preprocessing import StandardScaler
    from sklearn.pipeline import make_pipeline

    files = {k: sorted((DATA / k).glob("*.wav")) for k in KEYWORDS + ["other", "ambient"]}
    for k in KEYWORDS + ["other"]:
        before = len(files[k])
        files[k] = [p for p in files[k] if not is_truncated(p)]
        dropped = before - len(files[k])
        if dropped:
            print(f"  {k}: 시작 부분이 잘린 것으로 보이는 녹음 {dropped}개 제외 ({before} → {len(files[k])})")
    print("녹음:", {k: len(v) for k, v in files.items()})
    if min(len(files["geuman"]), len(files["meomchwo"])) < 8 or not files["ambient"]:
        raise SystemExit("녹음이 모자람 (그만·멈춰 각 8개 이상 + 주변 소리 1개 이상). scripts/kws/record.py")
    kws = [k for k in KEYWORDS if len(files[k]) >= 8]
    if "rally" not in kws:
        print("랠리야 녹음 8개 미만 → 비상어만 학습")
    labels = ["none"] + kws

    # 사람·순서 기준으로 20 % 를 평가용으로 떼어 둠
    def split(fs):
        fs = fs[:]
        rng.shuffle(fs)
        k = max(2, len(fs) // 5)
        return fs[k:], fs[:k]
    tr, te = {}, {}
    for k in kws + ["other"]:
        tr[k], te[k] = split(files[k])
    # 파일마다 70/30 을 나눠 합침 (파일 하나를 통째로 한쪽에만 넣으면 그 파일의 특성에 평가가 쏠림)
    amb = [read_wav(p) for p in files["ambient"]]
    amb_tr = np.concatenate([a[:int(len(a) * 0.7)] for a in amb])
    amb_te = np.concatenate([a[int(len(a) * 0.7):] for a in amb])
    noises = [amb_tr]

    feat = AudioFeatures()
    X, y = [], []
    for li, k in enumerate(kws, start=1):
        for p in tr[k]:
            x = read_wav(p)
            for _ in range(a.aug):
                X.append(augment(x, noises, rng))
                y.append(li)
    for p in tr["other"]:
        x = read_wav(p)
        for _ in range(max(3, a.aug // 3)):
            X.append(augment(x, noises, rng))
            y.append(0)
    n_amb = max(len(X) // 2, int(len(amb_tr) / RATE * 3))   # 주변 소리 조각 '아님': 1초당 3개 (길게 녹음할수록 더 씀)
    for _ in range(n_amb):
        s = rng.randint(0, len(amb_tr) - CLIP)
        X.append(amb_tr[s:s + CLIP] * rng.uniform(0.5, 1.5))
        y.append(0)
    print(f"학습 예시 {len(X)}개 (키워드 {sum(1 for v in y if v)} · 아님 {sum(1 for v in y if not v)}) → 특징 추출 중")
    Xe, Y = embed(feat, X), np.array(y)

    def fit(Xe: np.ndarray, Y: np.ndarray):
        c = make_pipeline(StandardScaler(), MLPClassifier(hidden_layer_sizes=(a.hidden,), alpha=1e-3, max_iter=300,
                                                          early_stopping=True, random_state=a.seed))
        return c.fit(Xe, Y)
    clf = fit(Xe, Y)

    # 오탐 구간 재학습: 학습용 주변 소리(앞 70 %)를 흘려 키워드로 헷갈린 순간을 찾아 '아님' 으로 더 가르친다.
    # 평가용(뒤 30 %)은 쓰지 않는다. wake word 학습에서 흔히 쓰는 방법 (hard negative mining)
    for r in range(a.mine):
        kw = 1.0 - stream_proba(feat, clf, amb_tr)[:, 0]
        ends = [(i + 1) * 1280 for i in np.where(kw >= 0.5)[0] if (i + 1) * 1280 >= CLIP]
        if len(ends) > 1500:
            ends = rng.sample(ends, 1500)
        print(f"  재학습 {r + 1}/{a.mine}: 학습용 주변 소리 {len(amb_tr) / RATE:.0f} s 에서 헷갈린 순간 {len(ends)}개 → '아님' 으로 추가")
        if not ends:
            break
        hard = [amb_tr[e - CLIP:e] * rng.uniform(0.6, 1.4) for e in ends for _ in range(2)]
        Xe = np.vstack([Xe, embed(feat, hard)])
        Y = np.concatenate([Y, np.zeros(len(hard), dtype=Y.dtype)])
        clf = fit(Xe, Y)

    # 1) 떼어 둔 녹음으로 인식률. 각 녹음이 비상·호출 어느 쪽으로 잡히는지 둘 다 센다
    print(f"\n[떼어 둔 녹음]  임계 {a.threshold}    비상 감지 / 호출 감지")
    for k in kws + ["other"]:
        Xt = embed(feat, [fit_len(read_wav(p), CLIP, offset=int(0.2 * RATE)) for p in te[k]])
        p = clf.predict_proba(Xt)
        em = group_score(p, labels, EMERGENCY) >= a.threshold
        wk = group_score(p, labels, WAKE) >= a.threshold
        want = "비상" if k in EMERGENCY else "호출" if k in WAKE else "없음"
        print(f"  {k:9s} {int(em.sum()):2d} / {int(wk.sum()):2d}   (총 {len(p)}, 맞는 쪽: {want})")

    # 2) 주변 소리 흘려서 오탐
    pr = stream_proba(feat, clf, amb_te)
    hours = len(amb_te) / RATE / 3600
    fp_score = {"비상": group_score(pr, labels, EMERGENCY), "호출": group_score(pr, labels, WAKE)}
    has_wake = any(k in kws for k in WAKE)
    print(f"\n[주변 소리 {len(amb_te) / RATE:.0f} s] 잘못 감지 (목표: 10분에 1회 이하 = 시간당 6회)  연속 {a.consec} 프레임")
    for name in ["비상", "호출"] if has_wake else ["비상"]:
        hits = count_hits(fp_score[name], a.threshold, a.consec)
        print(f"  {name} {hits}회 → 시간당 {hits / hours:.0f}회")

    # 3) 떼어 둔 키워드를 주변 소리 사이에 넣고 흘려 봄 → 실제 상시 감지처럼 잡히는지·녹음 끝에서 몇 프레임 뒤에 넘는지
    streams = []                                         # (라벨, 비상 점수, 호출 점수, 말 끝 시각 s, 파일)
    for k in kws:
        for p in te[k]:
            x = read_wav(p)
            s = rng.randint(0, len(amb_te) - 3 * RATE)
            bg = amb_te[s:s + 3 * RATE] * 0.3
            audio = np.concatenate([bg[:RATE], x, bg[RATE:2 * RATE]])
            sp = stream_proba(feat, clf, audio)
            streams.append((k, group_score(sp, labels, EMERGENCY), group_score(sp, labels, WAKE),
                            1.0 + word_end_s(x), p))

    def stream_eval(group: tuple[str, ...], col: int, thr: float, consec: int) -> tuple[int, int, list[float]]:
        """감지 수 · 전체 · 말 끝 → 감지 지연(s). 점수는 그 80 ms 조각을 다 들은 뒤 나오므로 (프레임+1)×0.08."""
        rows = [r for r in streams if r[0] in group]
        found = [(first_hit(r[col], thr, consec), r[3]) for r in rows]
        delays = [(f + 1) * FRAME_S - end for f, end in found if f is not None]
        return len(delays), len(rows), delays

    n, total, delays = stream_eval(EMERGENCY, 1, a.threshold, a.consec)
    if delays:
        print(f"\n[비상어 지연] 말 끝 → 감지 중간값 {np.median(delays):+.2f} s · 90 % {np.percentile(delays, 90):+.2f} s "
              f"(목표 0.3 s, 정지 명령·로봇 정지 시간은 별도). 감지 {n}/{total}")
    missed = [(r[4], r[1 if r[0] in EMERGENCY else 2]) for r in streams
              if first_hit(r[1 if r[0] in EMERGENCY else 2], a.threshold, a.consec) is None]
    for p, sc in missed:
        print(f"  놓침: {p.parent.name}/{p.name}  최고 점수 {sc.max():.2f}  임계 넘은 프레임 {int((sc >= a.threshold).sum())}개")

    # 4) 임계·연속 프레임은 학습에 안 쓰여서 한 번 학습한 모델로 조합을 다 비교할 수 있다
    print("\n[임계 × 연속 프레임]  흘려 넣은 키워드 감지 · 주변 소리 오탐(시간당) · 비상어 말 끝 → 감지 중간값")
    print("  임계  연속 |  비상 감지  오탐  지연  |  호출 감지  오탐")
    for thr in (0.7, 0.8, 0.9, 0.95):
        for c in (1, 2, 3):
            en, et, ed = stream_eval(EMERGENCY, 1, thr, c)
            efp = count_hits(fp_score["비상"], thr, c) / hours
            line = f"  {thr:.2f}  {c}    |  {en:2d}/{et:<2d}  {efp:6.0f}  {np.median(ed) if ed else float('nan'):+.2f}"
            if has_wake:
                wn, wt, _ = stream_eval(WAKE, 2, thr, c)
                line += f"  |  {wn:2d}/{wt:<2d}  {count_hits(fp_score['호출'], thr, c) / hours:6.0f}"
            print(line)

    OUT.mkdir(parents=True, exist_ok=True)
    import joblib
    joblib.dump({"clf": clf, "win": WIN, "labels": labels, "emergency": list(EMERGENCY), "wake": list(WAKE),
                 "threshold": a.threshold, "consec": a.consec}, OUT / a.out)
    print(f"\n저장: {OUT / a.out}  (임계 {a.threshold}, 연속 {a.consec} 프레임)")


if __name__ == "__main__":
    main()
