#!/usr/bin/env python3
"""
reid_embedding_eval.py

"어떤 외형 기술자가 이 현장에서 실제로 사람을 구분하는가"를 로스백으로 직접 재는 도구.
reid_tracking_node의 외형 매칭 임계값을 감으로 정하지 않고 이 출력에 근거해 정하기 위한 것.

[라벨 만드는 법 - 수동 라벨링 없이 확실한 GT를 얻는 트릭]
  음성쌍(다른 사람): 같은 프레임에 동시에 잡힌 서로 다른 검출. 한 사람이 같은 순간
      두 곳에 있을 수 없으므로 100% 확실하다.
  양성쌍(같은 사람): 인접 프레임에서 YOLO 트래커가 같은 track_id를 준 검출. 짧은 간격의
      동일 id는 거의 확실히 같은 사람이다(긴 간격은 id가 바뀌므로 --max-frame-gap으로 제한).
  주의: track_id를 "긴 시간에 걸친" 신원 GT로 쓰면 안 된다 - 그건 지금 고치려는 대상이라
  오염된 라벨이 된다. 인접 프레임으로 제한하는 이유가 이것이다.

[판정 지표]
  AUC가 핵심이다. 양성/음성 표본 수가 크게 불균형(동시검출 프레임이 드물다)하므로
  단순 정확도는 "전부 양성"으로도 높게 나와 의미가 없다.

[실측 결과 - 2026-08-08, my_new_bag3/4/5 전체 (양성 802쌍 / 음성 59쌍)]
  기술자              같은사람  다른사람   AUC     최적임계
  YOLO 백본 특징      0.9718   0.8905   0.9359   0.943
  HSV 상하체 히스토그램 0.9752   0.8071   0.9639   0.945
  yolo26n-reid.onnx   0.7393   0.2297   0.9771   0.473

  AUC만 보면 셋 다 높지만 **임계값의 견고함이 완전히 다르다**. 백본/HSV는 양성·음성이
  0.8~0.97 좁은 구간에 몰려 있어(양성 5%분위 0.916 vs 음성 95%분위 0.954로 역전) 임계를
  0.01 단위로 맞춰야 하고, 조명/장소가 바뀌면 그대로 무너진다. 전용 ReID는 0.23 대 0.74로
  분포가 넓게 갈라져 임계 0.473이 넓은 골짜기 한가운데 놓인다.
  => 실사용에는 전용 ReID 모델을 쓴다. 예전 tracktrack_reid.yaml의 model:auto가 짧은
     공백만 메우고 긴 공백은 못 메웠던 것도 auto가 이 백본 특징을 쓰기 때문으로 보인다.

[사용법]
  python3 reid_embedding_eval.py --bags rosbag/my_new_bag3 rosbag/my_new_bag4 \
      --reid-model yolo26n-reid.onnx
"""

import argparse
import sqlite3
import sys
from itertools import product

import cv2
import numpy as np


def _rgb_messages(bag_dir, topic):
    from pathlib import Path
    db = sorted(Path(bag_dir).glob('*.db3'))
    if not db:
        raise FileNotFoundError(f'{bag_dir}에 .db3 파일이 없습니다')
    conn = sqlite3.connect(str(db[0]))
    cur = conn.cursor()
    cur.execute('SELECT id FROM topics WHERE name=?', (topic,))
    row = cur.fetchone()
    if row is None:
        conn.close()
        raise KeyError(f'{bag_dir}에 토픽 {topic}이 없습니다')
    cur.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp', (row[0],))
    data = [r[0] for r in cur.fetchall()]
    conn.close()
    return data


def _hsv_hist(crop):
    """상/하체 HSV 히스토그램 - 옷 색 기반 저비용 기술자(비교군)."""
    h = crop.shape[0]
    feats = []
    for part in (crop[:h // 2], crop[h // 2:]):
        if part.size == 0:
            feats.append(np.zeros(64, dtype=np.float32))
            continue
        hsv = cv2.cvtColor(part, cv2.COLOR_BGR2HSV)
        hh = cv2.calcHist([hsv], [0], None, [32], [0, 180]).flatten()
        hs = cv2.calcHist([hsv], [1], None, [32], [0, 256]).flatten()
        feats.append(np.concatenate([hh, hs]))
    f = np.concatenate(feats).astype(np.float32)
    n = np.linalg.norm(f)
    return f / n if n > 0 else f


def _normalize(v):
    v = np.asarray(v).flatten().astype(np.float32)
    n = np.linalg.norm(v)
    return v / n if n > 0 else v


def collect(bags, topic, pose_model_path, reid_model_path, conf, stride):
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CompressedImage
    from ultralytics import YOLO

    model = YOLO(pose_model_path)
    embed_layer = len(model.model.model) - 2
    encoder = None
    if reid_model_path:
        from ultralytics.trackers.utils.reid import ReID
        encoder = ReID(reid_model_path)

    frames = []
    for bag in bags:
        model.predictor = None  # bag이 바뀌면 트래커 상태를 리셋해야 id가 섞이지 않는다
        for i, data in enumerate(_rgb_messages(bag, topic)):
            if i % stride:
                continue
            msg = deserialize_message(data, CompressedImage)
            frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                continue
            boxes = model.track(frame, persist=True, classes=[0], conf=conf,
                                verbose=False)[0].boxes
            if boxes is None or boxes.id is None:
                continue
            # ReID 인코더는 원본 프레임 + xywh 박스를 받아 내부에서 크롭한다.
            # 직접 크롭해 넘기면 안 된다(내부에서 다시 크롭하려다 실패).
            reid_embs = (encoder(frame, boxes.xywh.cpu().numpy().astype(np.float32))
                         if encoder is not None else None)
            dets = []
            for j in range(len(boxes)):
                x1, y1, x2, y2 = (int(v) for v in boxes.xyxy[j].cpu().numpy())
                crop = frame[max(0, y1):y2, max(0, x1):x2]
                if crop.size == 0 or crop.shape[0] < 20 or crop.shape[1] < 10:
                    continue
                entry = {
                    'id': int(boxes.id[j]),
                    'hsv': _hsv_hist(crop),
                    'backbone': _normalize(
                        model.predict(crop, embed=[embed_layer], verbose=False)[0].cpu().numpy()),
                }
                if reid_embs is not None:
                    entry['reid'] = _normalize(reid_embs[j])
                dets.append(entry)
            if dets:
                frames.append((bag, i, dets))
    return frames


def evaluate(frames, key, max_frame_gap):
    cos = lambda a, b: float(np.dot(a, b))  # noqa: E731
    neg = [cos(a[key], b[key])
           for _, _, dets in frames if len(dets) >= 2
           for i, a in enumerate(dets) for b in dets[i + 1:]]
    pos = []
    for k in range(len(frames) - 1):
        bag_a, idx_a, dets_a = frames[k]
        bag_b, idx_b, dets_b = frames[k + 1]
        if bag_a != bag_b or idx_b - idx_a > max_frame_gap:
            continue
        for p in dets_a:
            for q in dets_b:
                if p['id'] == q['id']:
                    pos.append(cos(p[key], q[key]))
    if not pos or not neg:
        return None
    pos, neg = np.array(pos), np.array(neg)
    wins = sum(1 for p, n in product(pos, neg) if p > n)
    ties = sum(1 for p, n in product(pos, neg) if p == n)
    auc = (wins + 0.5 * ties) / (len(pos) * len(neg))
    best = (0.0, 0.0)
    for th in np.linspace(0.0, 1.0, 401):
        balanced = ((pos >= th).mean() + (neg < th).mean()) / 2
        if balanced > best[0]:
            best = (balanced, th)
    return {'pos': pos, 'neg': neg, 'auc': auc, 'best_acc': best[0], 'best_th': best[1]}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bags', nargs='+', required=True)
    ap.add_argument('--topic', default='/robot5/oakd/rgb/image_raw/compressed')
    ap.add_argument('--pose-model', default='yolo11n-pose.pt')
    ap.add_argument('--reid-model', default='yolo26n-reid.onnx',
                    help="전용 ReID onnx 경로. 없으면 ultralytics가 자동 다운로드. ''면 생략")
    ap.add_argument('--conf', type=float, default=0.3)
    ap.add_argument('--stride', type=int, default=1, help='N프레임마다 1장만 평가(속도 조절)')
    ap.add_argument('--max-frame-gap', type=int, default=3,
                    help='양성쌍으로 인정할 최대 프레임 간격')
    args = ap.parse_args()

    frames = collect(args.bags, args.topic, args.pose_model, args.reid_model,
                     args.conf, args.stride)
    multi = sum(1 for f in frames if len(f[2]) >= 2)
    print(f'검출된 프레임 {len(frames)}개 (동시검출 2명 이상: {multi}개)')
    if multi == 0:
        print('경고: 동시검출 프레임이 없어 음성쌍을 만들 수 없습니다.', file=sys.stderr)
        return

    candidates = [('YOLO 백본 특징', 'backbone'), ('HSV 상하체 히스토그램', 'hsv')]
    if args.reid_model:
        candidates.append(('전용 ReID 모델', 'reid'))

    for name, key in candidates:
        r = evaluate(frames, key, args.max_frame_gap)
        if r is None:
            print(f'{name}: 표본 부족')
            continue
        pos, neg = r['pos'], r['neg']
        print(f'{name}:')
        print(f'   같은사람 평균={pos.mean():.4f}  5%={np.percentile(pos, 5):.4f} (n={len(pos)})')
        print(f'   다른사람 평균={neg.mean():.4f} 95%={np.percentile(neg, 95):.4f} (n={len(neg)})')
        print(f'   AUC={r["auc"]:.4f}  최적임계 {r["best_th"]:.3f}에서 '
              f'균형정확도 {r["best_acc"] * 100:.1f}%')


if __name__ == '__main__':
    main()
