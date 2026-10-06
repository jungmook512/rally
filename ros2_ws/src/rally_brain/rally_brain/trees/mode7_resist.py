"""rally_brain/trees/mode7_resist.py — 모드 7 저항 밀기 서브트리
담당: 조정묵
ResistPush(stiffness, target_force 는 환자 지난 기록 peak_force 의 60 % 에서 시작) → 성공 3회 연속 → target +10 % / 실패 2회 → -10 %
→ TrialResult(metrics: peak_force_n, hold_time, smoothness) → TTS 피드백(\"좋아요, 2초 버텼어요\")
"""
