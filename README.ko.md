<div align="center">

# Flyto2 × LeRobot Arms

**LeRobot 로봇팔에게 평소 말투로 일을 맡기세요. 로봇팔이 해내고, Flyto2가 끝났는지 확인해요.**

![Flyto2 device pack](https://img.shields.io/badge/Flyto2-device%20pack-6d5dfc) ![LeRobot](https://img.shields.io/badge/LeRobot-compatible-ffcc4d) ![Python](https://img.shields.io/badge/python-3.11%2B-3776ab)

[English](README.md) · [日本語](README.ja.md) · **한국어** · [简体中文](README.zh-CN.md) · [Español](README.es.md) · [Português (BR)](README.pt-BR.md)

[빠른 시작](#빠른-시작) · [하이라이트](#하이라이트) · [작동 방식](#작동-방식) · [개발자용](#개발자용)

</div>

## 빠른 시작

```bash
# in the Python environment that runs Flyto2
git clone https://github.com/flytohub/flyto-modules-lerobot && cd flyto-modules-lerobot
pip install "lerobot[feetech] @ git+https://github.com/liyiteng/lerobot_alohamini"
pip install -e .
cp examples/am-arm200.json my-arm.json        # set your port, teach your poses
export FLYTO_LEROBOT_ARM_CONFIG=$PWD/my-arm.json
```

로봇팔을 Flyto2 Space에 추가하고 이렇게 말하면 돼요:

> 지그 위 부품 집어서 트레이에 넣어 줘.

## 하이라이트

- **말로 하면, 진짜로 움직여요.** 코드도, 경유점도, 옆에서 지켜볼 필요도 없어요.
- **바로 쓰는 네 가지 기술.** 자세 파악, 가르친 자세로 이동, 집기, 놓기.
- **로봇팔 말만 믿지 않아요.** 로봇팔은 한 일을 보고할 뿐이에요. Flyto2는 그것만으로 증명이라고 보지 않아요.
- **멈추면 딱 서요.** 자세를 그대로 유지하고, 들고 있는 걸 떨어뜨리지 않아요.
- **정한 범위를 안 넘어요.** 모든 동작을 설정한 범위로 확인해요.
- **브랜드를 넘어 협업.** 다른 로봇들과 하나의 권한 규칙으로 함께 움직여요.

## 작동 방식

1. 원하는 걸 말해요.
2. Flyto2가 계획하고 권한을 확인해요.
3. 이 팩이 LeRobot으로 로봇팔을 움직여요.
4. Flyto2가 로봇팔의 보고를 기록하지만, 그것만으로 증명으로 치지 않아요.

---

## 개발자용

설치, 설정, 능력, 테스트 내용은 영어 버전에 있어요. [README.md → For developers](README.md#for-developers)

License: Flyto2 Source Available License 1.1 (`LICENSE`).
