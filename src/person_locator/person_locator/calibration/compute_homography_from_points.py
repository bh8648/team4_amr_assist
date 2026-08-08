# compute_homography_from_points.py
#
# pick_pixel_points.py(웹캠 PC)와 log_amr_positions.py(AMR PC)가 각자
# 저장한 pixel_points.yaml / world_points.yaml을 순서대로 짝지어서
# homography를 계산하고, calibrate_homography.py와 완전히 같은 형식으로
# config/person_homography.yaml에 저장함. 이 스크립트는 ROS 토픽이 필요
# 없으므로(그냥 두 yaml 파일을 읽는 것뿐) 두 PC 중 아무 데서나, 또는 두
# 파일을 한쪽으로 옮겨온 뒤 그쪽에서 돌리면 됨.
#
# 사용법:
#   ros2 run person_locator compute_homography_from_points \
#       --pixel-yaml pixel_points.yaml --world-yaml world_points.yaml \
#       --output src/person_locator/config/person_homography.yaml

import argparse
import datetime
import os

import cv2
import numpy as np
import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--pixel-yaml', required=True)
    parser.add_argument('--world-yaml', required=True)
    parser.add_argument('--output', default='config/person_homography.yaml')
    args = parser.parse_args()

    with open(args.pixel_yaml) as f:
        pixel_points = yaml.safe_load(f)['pixel_points']
    with open(args.world_yaml) as f:
        world_points = yaml.safe_load(f)['world_points']

    if len(pixel_points) != len(world_points):
        raise SystemExit(
            f'점 개수가 다릅니다: pixel={len(pixel_points)}개, world={len(world_points)}개. '
            f'두 PC에서 같은 순서로 같은 개수만큼 찍었는지 확인하세요.'
        )
    if len(pixel_points) < 4:
        raise SystemExit(f'점이 {len(pixel_points)}개뿐입니다. 최소 4개 필요합니다.')

    pixel_arr = np.array(pixel_points, dtype=np.float32)
    world_arr = np.array(world_points, dtype=np.float32)
    H, _ = cv2.findHomography(pixel_arr, world_arr)
    if H is None:
        raise SystemExit('homography 계산 실패 - 점들이 일직선상에 있는지 확인하세요.')

    out = {
        'homography_matrix': H.flatten().tolist(),
        'frame_id': 'map',
        'calibrated_at': datetime.datetime.now().isoformat(),
        'num_points': len(pixel_points),
        'pixel_points': [[float(u), float(v)] for u, v in pixel_points],
        'world_points': [[float(x), float(y)] for x, y in world_points],
        'note': ('이 homography는 이 카메라가 map 프레임에 대해 고정된 '
                 '위치/각도로 있을 때만 유효함. 카메라를 옮기거나 map '
                 '원점이 바뀌면 다시 캘리브레이션해야 함. '
                 '(pick_pixel_points.py + log_amr_positions.py로 분리 수집 후 계산됨)'),
    }
    os.makedirs(os.path.dirname(args.output) or '.', exist_ok=True)
    with open(args.output, 'w') as f:
        yaml.safe_dump(out, f, sort_keys=False)

    print(f'homography ({len(pixel_points)}개 점)를 {args.output}에 저장했습니다')
    print(H)


if __name__ == '__main__':
    main()
