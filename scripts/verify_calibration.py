"""기준점 터치 검증 — 선생님 calibration/verify.py 를 수치 출력 버전으로.
사용: 테이블 위 마커 N개를 클릭 → 비전 좌표(base) 출력 → 로봇 TCP 를 그 위에 수동(직접 교시)으로 두고 get_current_posx → 차이(mm) 기록.
담당: 박성현 (로봇 자세는 정정인)
TODO: rally_vision.calibration.verify 를 import 해 클릭 → 좌표 변환까지 재사용, 이동 대신 좌표만 출력
"""
