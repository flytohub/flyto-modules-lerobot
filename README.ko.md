<div align="center">

# Flyto2 for LeRobot Arms

**집어 올릴 것을 말하기만 하세요. 240달러짜리 로봇팔이 일하고, 다른 카메라가 정말 해냈는지 확인합니다.**

![Flyto2 device pack](https://img.shields.io/badge/Flyto2-device%20pack-6d5dfc) ![LeRobot](https://img.shields.io/badge/LeRobot-arms-ffcc4d) ![status](https://img.shields.io/badge/status-early%20preview-lightgrey)

[English](README.md) · [日本語](README.ja.md) · **한국어** · [简体中文](README.zh-CN.md) · [Español](README.es.md) · [Português (BR)](README.pt-BR.md)

</div>

---

## 한 문장이 실제 작업으로

*"지그 위의 부품을 집어서 트레이에 넣어 줘."* Flyto2는 이를 단계로 나누고, 필요한 단계를 로봇팔에 보내며, 실제로 완료된 것이 확인될 때만 작업을 완료로 처리합니다.

## 로봇팔이 할 수 있는 일

- **자기 위치 파악.** 모든 관절과 그리퍼의 현재 상태.
- **가르친 곳으로 이동.** *ready*, *tray* 같은 이름 붙은 자세로, 설정한 한계는 절대 넘지 않습니다.
- **집기.** 그리퍼를 닫고 붙잡습니다.
- **놓기.** 그리퍼를 열고 내려놓습니다.

## 다른 로봇과 함께

로봇팔은 혼자 일하지 않습니다. 이동 로봇이 부품을 가져오고, 로봇팔이 집고, 카메라가 확인합니다. 브랜드가 달라도 하나의 권한 규칙 아래에서.

## 처음부터 안전하게

- **정지는 그 자리 유지.** 힘이 빠져 들고 있던 것을 떨어뜨리지 않습니다.
- **스스로의 말을 믿지 않습니다.** 결과는 다른 카메라가 확인합니다.
- **정해진 범위 안에서만.** 모든 동작을 설정한 한계와 대조합니다.
- **추측하지 않습니다.** 동작 중 연결이 끊기면 *알 수 없음*으로 보고하고 동작을 반복하지 않습니다.
- **결정권은 당신에게.** 로봇팔을 움직이려면 승인이 필요하고, 다시 학습한 로봇팔은 다시 승인을 받습니다.

## 함께 성장합니다

손으로 가르친 자세부터 시작하세요. 나중에 LeRobot으로 더 똑똑한 집기를 학습시키면 같은 능력 뒤에 그대로 들어가고, Flyto2는 새 버전을 쓰기 전에 승인을 요청합니다. LeRobot으로 움직이는 로봇팔이라면 누구나 참여할 수 있으며, 첫 번째는 AM-ARM200입니다.

## 4단계로 시작

1. AM-ARM200(팔로워 약 240달러)과 손이 닿는 곳에 12V 전원 스위치를 준비합니다.
2. 로봇팔을 연결할 컴퓨터에 이 팩과 LeRobot을 설치합니다.
3. `examples/am-arm200.json`을 복사하고 포트를 설정한 뒤 자세를 가르칩니다.
4. 로봇팔을 Flyto2 Space에 추가하고 원하는 일을 말합니다.

## 현재 상태

얼리 프리뷰입니다. 모든 능력은 시뮬레이션 로봇팔과 Flyto2 Cloud로 끝까지 테스트했습니다. 다음은 실제 AM-ARM200에서의 첫 실행입니다.

---

## 개발자용

설치, 설정, 능력, 테스트 내용은 영어 README에 있습니다. [README.md → For developers](README.md#for-developers)

License: Flyto2 Source Available License 1.1 (`LICENSE`).
