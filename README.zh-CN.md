<div align="center">

# Flyto2 × LeRobot Arms

**用大白话给你的 LeRobot 机械臂派活。它去干，Flyto2 确认真的干完了。**

![Flyto2 device pack](https://img.shields.io/badge/Flyto2-device%20pack-6d5dfc) ![LeRobot](https://img.shields.io/badge/LeRobot-compatible-ffcc4d) ![Python](https://img.shields.io/badge/python-3.11%2B-3776ab)

[English](README.md) · [日本語](README.ja.md) · [한국어](README.ko.md) · **简体中文** · [Español](README.es.md) · [Português (BR)](README.pt-BR.md)

</div>

## 快速开始

```bash
git clone https://github.com/flytohub/flyto-modules-lerobot && cd flyto-modules-lerobot
pip install "lerobot @ git+https://github.com/liyiteng/lerobot_alohamini"
pip install -e ".[core]"
cp examples/am-arm200.json my-arm.json        # set your port, teach your poses
export FLYTO_LEROBOT_ARM_CONFIG=$PWD/my-arm.json
```

把机械臂加入 Flyto2 Space，然后说：

> 把治具上的零件拿起来，放进托盘。

## 亮点

- **说人话，真干活。** 不写代码，不设路径点，不用盯着。
- **四项技能，开箱即用。** 知道自己的姿态、去教过的姿势、抓住、放下。
- **要证据，不要嘴上说。** “做完了”要另一台摄像头看到才算数。
- **一停就定住。** 原地保持姿态，绝不把手里的东西摔下去。
- **绝不越界。** 每个动作都对照你设的范围检查。
- **跨品牌协作。** 和你的其他机器人在同一套权限规则下一起干活。

## 怎么运作

1. 你说出想做的事。
2. Flyto2 规划步骤并检查权限。
3. 本包通过 LeRobot 驱动机械臂。
4. 另一台摄像头确认结果。

---

## 开发者

安装、配置、能力与测试的细节在英文版里。 [README.md → For developers](README.md#for-developers)

License: Flyto2 Source Available License 1.1 (`LICENSE`).
