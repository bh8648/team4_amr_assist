#!/usr/bin/env bash
# 화면 전체를 ffmpeg x11grab으로 녹화한다 - RViz 창을 최대화해두고 이 스크립트를 먼저
# 백그라운드로 띄운 뒤 ros2 bag play를 시작하면 된다.
#
# 사용법: record_evidence.sh <output_name_no_ext> <duration_seconds> [video_size]
#   예: tools/record_evidence.sh evidence_before_circularity_filter 28

set -euo pipefail

OUT="${1:?output name required}"
DUR="${2:?duration seconds required}"
VIDEO_SIZE="${3:-$(xdpyinfo 2>/dev/null | awk '/dimensions/{print $2}')}"
VIDEO_SIZE="${VIDEO_SIZE:-1920x1080}"

echo "녹화 시작: ${OUT}.mp4 (${DUR}s, ${VIDEO_SIZE}, 화면 전체 :0.0)"
ffmpeg -y -f x11grab -video_size "$VIDEO_SIZE" -framerate 15 -i :0.0 \
    -t "$DUR" -pix_fmt yuv420p "${OUT}.mp4"
echo "녹화 종료: ${OUT}.mp4"
