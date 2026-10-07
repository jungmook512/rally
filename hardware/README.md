# hardware — 3D 출력물

출력물마다 폴더 하나. 원본 CAD(STEP 또는 Fusion 등)와 출력용 STL 을 같이 둔다.
파일이 수십 MB 를 넘으면 Git LFS 로.

| 폴더 | 무엇 | 담당 | 마감 |
|---|---|---|---|
| `pad_handle/` | 벨크로 패드(지름 15 cm) 뒤에 붙여 RG2 가 잡는 손잡이. RealSense 시야를 가리지 않게, 충격 시 회전 방지 | 정동휘 | Day 4 (10/9) |
| `pad_holder/` | 로봇이 받은 패드를 걸어두는 거치대. 그 위에서 `DetachBall`(공 떼기) | 정동휘 | Day 4 (10/9) |

각 폴더 README 에 적을 것: 주요 치수, 재료·출력 설정, 조립 방법, 바꾼 이유(버전마다 한 줄).

## 지금 들어 있는 파일 (10/7)
| 폴더 | 파일 | 형식 |
|---|---|---|
| `pad_handle/` | `패드손잡이.stp` | STEP (다른 CAD 에서 열기용) |
| `pad_handle/` | `패드 그리퍼 연계.ipt` | Inventor 원본 |
| `pad_holder/` | `패드 거치대.ipt` | Inventor 원본 |
| `pad_holder/` | `패드 거치대.stp` | STEP |

출력용 STL 은 아직 없음 (출력할 때 추가).
