<div align="center">

# Flyto2 × LeRobot Arms

**用自然语言给 LeRobot 机械臂下达任务。机械臂执行，Flyto2 确认任务真正完成。**

![Flyto2 device pack](https://img.shields.io/badge/Flyto2-device%20pack-6d5dfc) ![LeRobot](https://img.shields.io/badge/LeRobot-compatible-ffcc4d) ![Python](https://img.shields.io/badge/python-3.11%2B-3776ab)

[English](README.md) · [日本語](README.ja.md) · [한국어](README.ko.md) · **简体中文** · [Español](README.es.md) · [Português (BR)](README.pt-BR.md)

</div>

## 快速开始

```bash
# in the Python environment that runs Flyto2
git clone https://github.com/flytohub/flyto-modules-lerobot && cd flyto-modules-lerobot
pip install "lerobot @ git+https://github.com/liyiteng/lerobot_alohamini"
pip install -e .
cp examples/am-arm200.json my-arm.json        # set your port, teach your poses
export FLYTO_LEROBOT_ARM_CONFIG=$PWD/my-arm.json
```

把机械臂加入 Flyto2 Space，然后说：

> 把治具上的零件拿起来，放进托盘。

## 特性

- **自然语言下达任务**：直接说“把零件放进托盘”，无需写代码或设路径点。
- **四项操作能力**：读取姿态、移动到示教姿势、抓取、释放。
- **独立结果验证**：是否完成由另一台摄像头判定，而不是机械臂自己报告。
- **停止即保持**：停止时原地保持姿态，不会松手掉落物品。
- **动作范围限制**：每次移动都在你设定的关节范围内执行。
- **跨品牌协作**：与其他品牌的机器人在同一套权限规则下协同工作。

## 工作原理

1. 你用一句话说出要做的事。
2. Flyto2 规划步骤并检查权限。
3. 本包通过 LeRobot 驱动机械臂执行。
4. 另一台摄像头确认结果。

---

## 开发者

安装、配置、能力与测试的细节在英文版里。 [README.md → For developers](README.md#for-developers)

License: Flyto2 Source Available License 1.1 (`LICENSE`).
