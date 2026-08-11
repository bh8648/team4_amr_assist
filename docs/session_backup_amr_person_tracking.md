# 세션 백업 — amr_person_tracking 디버깅/개선 전체 기록

이 문서는 `feature/amr_person_tracking` 브랜치에서 진행된 긴 대화 세션(컴팩션으로 요약된
초반부 포함) 전체를 시간 순으로 백업한 기록이다. 코드 자체의 최신 상태는 각 파일과
`docs/vision_pipeline.md`, `docs/amr_person_tracking_pipeline_flow.md`를 참고하고, 이
문서는 "무엇을, 왜, 어떻게 결정했는지"의 경위를 남기는 용도다.

## 0. 세션 시작 시점 배경

ROS2 Humble 워크스페이스에서 AMR(TurtleBot4 + OAK-D PRO) 기반 사람 추종 파이프라인
(`src/amr_person_tracking`)을 디버깅/개선하는 세션. 라이다 기반 다리검출, YOLO-pose+depth
근접 검출, 재식별/트래킹, 예측 회피 4개 노드로 구성.

## 1단계 — 치명적 버그 5건 수정 (Plan Mode로 진행, 승인 후 구현)

코드 리뷰로 발견된 버그를 고쳤다:
1. **header 참조 공유 버그**: `det.header = header` 처럼 호출자의 원본 header 객체를
   그대로 대입하면, 이후 `frame_id`를 바꾸는 코드가 원본까지 오염시켜 같은 콜백의 다음
   검출이 이미 바뀐 frame_id를 읽는 문제. `leg_detector_bridge_node.py`,
   `reid_tracking_node.py`의 `_make_detection3d`/`publish_*` 계열 함수들을
   `X.header.stamp = header.stamp; X.header.frame_id = ...` 패턴으로 수정 (tf2의 항등변환
   버그로 이어지던 문제).
2. **비배타적 트랙 매칭**: 한 콜백에 여러 검출이 들어올 때 각각 따로 `match_track`을
   부르면 같은 트랙을 두 검출이 중복으로 차지할 수 있었다. `tracking_utils.py`에
   `assign_tracks()`(그리디 배타적 배정)를 새로 추가하고 `match_track()`에
   `exclude_ids` 파라미터를 추가해 `leg_detector_bridge_node`/`reid_tracking_node`
   양쪽에서 사용하도록 교체.
3. **타임스탬프 역행 시 트랙 오염**: `Track.update()`가 과거 시각의 관측을 받으면 속도
   추정이 깨지는 문제. `_STALE_EPS` 허용치를 두고 그보다 과거면 갱신을 통째로 무시,
   `dt`를 0 이상으로 클램프, `last_stamp`는 `max()`로만 갱신하도록 수정.
4. **라이다 단독 락온 시 미발행**: 카메라가 안 보이는 초근접 구간에서
   `reid_tracking_node`가 라이다로만 갱신될 때 발행 경로가 없어 `tracked_detections`/
   `target_pose`가 조용히 멈춘 것처럼 보였다. `leg_detections_callback` 끝에 락온 상태일
   때만 발행하는 경로를 추가.
5. **`package.xml` 잔여 의존성**: 더 이상 쓰지 않는 `leg_detector_msgs` 의존과 설명
   주석을 제거.

단위테스트 `test_tracking_utils.py` 신규 작성(7건), 실제 bag으로 재검증.

## 2단계 — 도메인 격리 이슈 진단

`ROS_DOMAIN_ID`를 분리해도 로스백 재생 종료 후 RViz가 계속 갱신되는 문제 → 원인은
Fast-DDS Discovery Server가 `ROS_DOMAIN_ID`를 무시하는 것으로 진단. `ROS_DISCOVERY_SERVER=""`
+ `ROS_LOCALHOST_ONLY=1` + 매번 고유한 `ROS_DOMAIN_ID` 조합이 실제 로봇과 확실히 격리되는
유일한 방법임을 확인 — 이후 모든 실측/영상 작업에서 이 조합을 표준으로 사용.

## 3단계 — 다리검출 곡률(원형적합) 필터 도입 (Plan Mode)

사용자가 실제 로스백 재생 화면 녹화 영상을 보고: "다리가 하나도 검출 안 되다가 마지막
몇 초에 다리가 엄청 많다"는 문제 제기. 헝가리안/칼만필터 고도화, 논문기반(Arras et al.
ICRA2007) 부스팅 분류기 교체 등을 제안받았으나, **범위를 곡률/형상 필터로만 좁혀서
먼저 진행**하기로 함(사용자 선택).

- `leg_detection_utils.py`에 `fit_circle_kasa()`(Kåsa 대수적 원형적합, Cramer's rule로
  numpy 없이 구현), `circle_fit_rms_residual()` 추가.
- `filter_leg_clusters()`에 원형적합 게이팅 추가 — 벽 모서리(오목하게 꺾인 두 직선)를
  다리(볼록한 원호)로 오검출하던 문제 해결. 기본값: `radius_max=0.20m`, `rms_max=0.01m`
  (초기 0.02는 실측 모서리 케이스를 못 거름 → 0.01로 타이트하게 조정).
- 실측 결과: 오탐이 13% 정도만 감소 — 사용자가 "먼저 원인 더 파기"를 요청.
- 원인 추적: 가까운 거리(0.4~1.2m)의 몇몇 위치에 원형적합 통과 후보가 전체 스캔의
  56~97%라는 압도적 빈도로 반복 등장 — 실제 사람이라면 불가능한 지속성. 사용자 확인:
  "벽 너머에는 책상과 의자가 여러 개 있다" — 진짜 원통형 가구 다리라 형상만으로는
  구별 불가능한 근본적 한계.

## 4단계 — 정적배경 필터, 3세대에 걸친 재설계

**1세대 (고정 학습창)**: 노드 시작 후 N초(기본 5초)는 관측만 하고, 그 안에서
`min_occurrence_ratio` 이상 반복된 위치를 배경으로 확정. 사용자 질문: "사람이 미동없이
가만히 서있으면 제외되나?" → 실측 검증(로스백에서 일부러 자주 움직인 구간)으로 안전함을
확인. 그런데 "증거영상으로 확인해볼까?" 요청으로 영상을 만들어 자세히 보니 **12~14초,
24~25초 구간에 여전히 다리가 너무 많거나 사람 없이도 검출**되는 문제 발견. 원인: 학습창
5초 동안 사람이 그 물체 앞에 서 있어 가려졌던 물체는 영영 배경으로 못 배움.

**2세대 (person 트랙 단위 상시 확정)**: 짝지어진 사람 중심점이 `confirm_duration_sec`
이상 안 움직이면 확정하는 방식으로 변경. 재검증 결과 **더 나쁨** — `pair_legs()`의
그리디 짝짓기가 스캔마다 다른 후보와 묶여, 완전히 정지한 물체의 "짝지어진 중심점"도
5~10cm씩 튀는 것을 발견. 사람의 실제 정지 드리프트(초당 ~1.5cm)와 크기가 겹쳐 구분 불가.

**3세대 (그리드 셀 상시 연속관측)**: 짝짓기 이전의 개별 다리 위치를 그리드 셀 단위로
관측, 같은 셀이 `max_gap_sec` 이내 간격으로 `confirm_duration_sec` 이상 끊김 없이
관측되면 확정. 실측 재검증(같은 bag)에서 개별 다리 위치는 1cm 이내로 안정적임을 확인해
설계 방향은 맞았으나, **사람이 그 물체 앞을 반복해서 오가며 매번 1초 넘는 공백을 만들면
짧은 세션 안에 확정을 못 끝내는 문제**가 새로 발견됨 (여전히 3마커/유령마커 잔존).

## 5단계 — 증거영상 인프라 구축

`sudo apt install ffmpeg`로 설치 후, RViz 실제 화면 녹화 방식으로 전환:
- `depth_view_republisher_node.py` 신규 — 이 로봇의 compressedDepth가 비표준(양자화
  미적용) 포맷이라 RViz 표준 플러그인에 맡기지 않고, 이미 검증된 `vision_utils.decode_
  compressed_depth`로 디코딩 후 컬러맵 입혀 평범한 jpeg로 재발행.
- `rviz/evidence.rviz` 신규 — Grid/TF/LaserScan/MarkerArray/RGB Image/Depth Image 배치.
  TF의 "Show Names"가 근접 다리 마커와 겹쳐 안 보이는 문제를 나중에 발견해 껐다.
- `tools/record_evidence.sh` 신규 — `ffmpeg -f x11grab` 전체화면 녹화.
- 반복적으로 발견한 함정들: `cd && cmd &` 형태로 명령을 합치면 `&`의 낮은 우선순위 때문에
  `cd`까지 백그라운드 서브셸 안에서 실행돼 부모 쉘의 cwd가 안 바뀌는 문제(여러 번 재발) →
  `cd`는 항상 독립된 문장으로 실행해야 함. `rviz2 -r /tf:=...` 축약 remap 문법은
  rviz2에서 안 먹히고 `--ros-args --remap`이 필요함. 녹화 시작과 bag play 시작 사이
  텔레메트리 지연으로 타임라인이 어긋난 사례 → 고정 2초 딜레이로 정착.

## 6단계 — 칼만필터 기반 정적배경 필터로 재설계 (4세대, 최종)

3세대(그리드 셀)의 실패를 데이터로 재확인한 뒤, `predictive_avoidance_node`가 이미 쓰던
`ConstantVelocityKalman2D`(등속도 모델 칼만필터)를 재사용해 `predictive_utils.py`에
`LegKalmanTracker` 신규 추가:
- 개별 다리 후보마다 칼만필터를 붙여 배타적 매칭 + 속도 추정
- "처음 관측된 뒤 누적 나이 ≥ confirm_duration_sec(3초) 이고 추정 속도 ≤
  stationary_speed_threshold(0.01m/s)"를 정지 확정 기준으로 사용
- 칼만필터는 관측 공백이 있어도 상태가 리셋되지 않으므로(공백만큼 공정잡음이 커질 뿐),
  반복된 occlusion에도 결국 확정됨 — 3세대의 근본 문제를 구조적으로 해결
- `StaticBackgroundFilter`는 "확정 위치 저장 + 근접 판정"만 하는 얇은 클래스로 단순화
- `test_predictive_utils.py` 신규(6건: 정지 확정, occlusion 공백에도 확정 유지, 실측
  드리프트보다 빠른 이동은 미확정, 배타적 매칭, 오래된 다리 정리)

실측 재검증(my_new_bag2): 전체 평균 5.86→1.33건/scan(77.3%↓), 이전에 phantom 검출이
반복되던 bag 끝부분(occlusion 구간)은 4.94→0.36건/scan(**92.7%↓**), 실제 걷는 사람 트랙은
끊김없이 유지됨을 확인.

증거영상 재생성 과정에서 겪은 문제들: gnome-terminal로 로그를 화면에 띄우려던 시도가
렌더링되지 않아 실패 → 대신 실제 로그를 텍스트 파일로 별도 보관하는 방식으로 전환.

## 7단계 — 문서(`vision_pipeline.md`) 반영 + 락온 스트릭 실측 검증 (Plan Mode)

사용자 요청: 문서를 실제 구현에 맞게 갱신(새 섹션 추가 없이 기존 문단만), 칼만필터
영상에서 다리가 깜빡이는 게 `reid_tracking_node`의 원래 기대 역할(락온)을 방해하는지
검토, 지금까지 증거영상으로 검증된/안된 기능 정리 + 향후 검증 순서 제안.

- `docs/vision_pipeline.md`: "외부 ros2_leg_detector 패키지에 의존"이라는 옛 설명을
  실제 구현(LaserScan 직접 구독 + 곡률필터 + 칼만필터 기반 정적배경 필터)으로 교체,
  인터페이스 표에서 외부 패키지 행 삭제.
- 코드 추적으로 `_handle_unlocked()`가 "이번 스캔에 후보가 아예 없으면" 모든 락온 후보
  스트릭을 0으로 리셋하는 것을 발견 — 원시 검출기 깜빡임과 겹치면 락온 확정에 필요한
  연속 프레임을 채우기가 실질적으로 불가능할 수 있다는 이론적 리스크 제기.
- 시뮬레이션으로 검증하려 했으나 **첫 번째 시도가 자체 결함**이 있었음: "카메라 위치"를
  흉내낸 보조 트랙이 게이트 안에서만 최근접 매칭을 해서, 한 번 후보를 놓치면 그 자리에
  얼어붙어 실제 사람과 다시 못 만나는 문제. 게이트 없는 최근접 매칭으로 고쳐 재검증하니
  락온이 실제로 재획득됨을 확인 (수정 전: 재획득 5.66초, 수정 후: 1.17초 — 사용자가
  "성능이 좋아질 가능성 높다면 한번 수행" 요청).
- `reid_tracking_node.py`의 `_handle_unlocked()` 수정: `if best_rid is None: return`을
  스트릭 리셋 루프 앞으로 옮겨, 후보가 아예 없는 스캔은 스트릭을 안 건드리게 함 (다른
  후보로 실제로 바뀔 때만 리셋).
- **증거영상/평가 현황 표와 향후 검증 순서**를 대화창으로 정리해 전달 (문서에는 반영 안
  함 — 별도 새 섹션 금지 지침에 따름):
  ①락온 스트릭 시뮬레이션(완료) ②reid_tracking_node end-to-end 영상 ③oakd_detector_node
  단독 검증 ④predictive_avoidance_node 검증 ⑤실기 검증(마지막 단계).

## 8단계 — reid_tracking_node end-to-end 실측 + YOLO 기반 ID 안정성 확인

사용자 지적: "0.7m 이격거리를 두므로 라이다가 중요한가" → 조사 결과 0.7m는 "유지 거리"가
아니라 **스테레오 depth 센서의 물리적 최소 측정거리**(하드웨어 한계)였음을 확인, 실제
정지거리를 정하는 Nav2 goal 소비 로직은 이 저장소 범위 밖이라는 점, 사람이 스스로 로봇에
다가올 수도 있다는 점을 근거로 근접 커버리지의 필요성은 유지된다고 답변.

- `mock_webcam_publisher_node.py` 신규 — 이 워크스페이스엔 실제 웹캠 스트림이 없어서,
  라이다 다리검출 출력에 위치 노이즈(±0.1m)+지연(3메시지)을 더해 독립 출처처럼
  `/vision/webcam/detections_3d`에 재발행하는 테스트 전용 노드. `publish_mock_webcam`
  런치 인자(기본 false)로 평소엔 꺼짐.
- 실제 my_new_bag2를 이 구성으로 두 번 재생해, 수정한 락온 스트릭 로직이 실제 라이브
  실행에서도 "라이다 락온 확정" 로그를 여러 차례 정상 발생시키는 것을 확인 (예:
  `leg_5→locked→해제→leg_18→locked` 등 여러 번의 락온/해제 사이클).
- 증거영상 녹화 시도 중 RViz 화면은 잘 녹화됐지만, 로그 텍스트를 화면에 실시간으로
  띄우려던 시도(harness의 Bash 출력은 화면 터미널에 실시간 반영 안 됨, gnome-terminal도
  렌더링 안 됨)가 두 번 다 실패 → 실제 로그를 텍스트 파일로 별도 보관.
- 사용자 관찰: "다리 id는 5→15→18로 변경된다. 가급적 rgb에서 yolo나 다른 알고리즘 기반
  id 유지로 시도해보자" → 조사 결과 `oakd_detector_node.py`가 **이미**
  `pose_model.track(persist=True)`로 YOLO 자체 트래킹(ByteTrack류)을 쓰고 있었음을 확인.
- 사용자 질문: "id를 검출 좌표 위에 표시하는 게 나은가 고정 위치가 나은가?" → 검출 좌표를
  따라다니게 표시하는 게 맞다고 답변(고정 위치는 추적 중인지 프레임인지 구분 안 됨).
  기존 YOLO `result.plot()`의 라벨이 bbox 위쪽에 그려지는데, 이 로봇은 초근접 구간(머리가
  화면 밖으로 잘리는 바로 그 상황)에서 그 라벨이 안 보이는 경우가 흔함 → 발끝 좌표(항상
  화면에 보이는 지점) 위에 track_id를 추가로 그리도록 `oakd_detector_node.py`
  `_publish_debug_image` 수정.
- 실측 검증(`oakd_detector_node`의 `/vision/detections_3d`에서 `det.id`를 직접 캡처):
  **22.74초 전체 bag에서 track_id가 `oakd_2 → oakd_4 → oakd_5`로 단 2번만 바뀜** —
  같은 시간 동안 라이다 쪽이 5→15→18로 자주 바뀌던 것과 대조적으로 훨씬 안정적임을 확인.
  사용자 가설이 실측으로 뒷받침됨.

## 9단계 — 커밋/푸시 이력

- `c7db720`: 버그 수정 5건 + 곡률필터 + 1~3세대 정적배경 필터 + 증거영상 인프라
  (원래 `evidence/`도 함께 커밋했었는데, 사용자 요청으로 이후 amend해서 영상은 제외)
- `81e1520`: 칼만필터 기반 정적배경 필터 재설계(4세대, 최종) + 테스트
- `1f5feb2`: `docs/vision_pipeline.md` 갱신 + `reid_tracking_node` 락온 스트릭 리셋 조건
  수정
- `6583b4c`: `mock_webcam_publisher_node` 추가
- `5ff9b62`: `oakd_detector_node` 디버그 이미지에 YOLO track_id를 발끝 좌표 위에 표시

모두 `feature/amr_person_tracking` 브랜치, `origin`(bh8648/team4_amr_assist) 리모트로
푸시됨. `evidence/`(영상+로그)는 이번 백업 작업 전까지는 로컬에만 유지하기로 사용자가
명시적으로 요청했었음 (이번 백업 작업에서 `docs/`, `evidence/` 모두 커밋하도록 지침이
바뀜 — 아래 10단계 참고).

## 10단계 — 이번 백업 작업

사용자 요청 4가지:
1. 전체 대화 내역(컴팩션된 이전 내역 포함)을 마크다운으로 백업 → 이 문서
   (`docs/session_backup_amr_person_tracking.md`) + 별도 흐름 정리 문서
   (`docs/amr_person_tracking_pipeline_flow.md`)로 작성.
2. `docs/`, `evidence/`까지 커밋하고 푸시 (이전의 "영상 제외" 지침을 이번엔 명시적으로
   뒤집음).
3. 로스백은 git이 아니라 홈 디렉토리 밑에 별도 백업.
4. `amr_person_tracking`이 아직 미완성이지만 지금 어떤 작업이 어떤 흐름(반복/분기/통신
   포함)으로 진행되는지, 함수명/인터페이스 수준이 아니라 기능/작업흐름 단위로 상세하게
   정리 → `docs/amr_person_tracking_pipeline_flow.md`.

## 알아둘 것 — 이번 세션에서 발견했지만 아직 손대지 않은 이슈

- `oakd_detector_node.py`의 `publish_detections`/`_make_detection3d`에 `det.header =
  header` 참조공유 패턴이 남아있음 (다른 두 노드에서는 이미 고침). 아직 수정 안 함.
- `reid_tracking_node`의 라이다 락온이 실제로 웹캠 없이(mock 대체물로만) 검증됐다는
  한계 — 진짜 호모그래피 기반 웹캠 스트림 연동 검증은 아직.
- `predictive_avoidance_node`, `oakd_detector_node` 자체는 이번 세션에서 코드 변경도
  전용 실측 검증도 없었음 (Part C 제안 순서의 ③④ 단계가 남아있음).
