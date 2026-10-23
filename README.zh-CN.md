<div align="center">

# Flyto2 for LeRobot Arms

**说出要拿什么，一支约 240 美元的机械臂就去做，再由另一台摄像头确认真的完成了。**

![Flyto2 device pack](https://img.shields.io/badge/Flyto2-device%20pack-6d5dfc) ![LeRobot](https://img.shields.io/badge/LeRobot-arms-ffcc4d) ![status](https://img.shields.io/badge/status-early%20preview-lightgrey)

[English](README.md) · [日本語](README.ja.md) · [한국어](README.ko.md) · **简体中文** · [Español](README.es.md) · [Português (BR)](README.pt-BR.md)

</div>

---

## 一句话，真干活

*“把治具上的零件拿起来，放进托盘。”* Flyto2 把这句话拆成步骤，把该给机械臂的步骤交给它，并且只有亲眼看到完成，才算任务完成。

## 机械臂能做什么

- **知道自己在哪。** 每个关节和夹爪此刻的状态。
- **去你教过的地方。** 比如 *ready*、*tray* 这样的命名姿势，绝不超出你设定的范围。
- **抓取。** 合上夹爪并握住。
- **放下。** 张开夹爪，把东西放好。

## 和你的其他机器人一起干活

机械臂不是单打独斗。移动机器人把零件送来，机械臂抓起，摄像头确认，跨品牌协作，遵守同一套权限规则。

## 安全是设计出来的

- **停止就是原地保持。** 不会突然失力，把手里的东西掉下去。
- **不相信自己的一面之词。** 结果由另一台摄像头确认。
- **不越界。** 每个动作都对照你设定的范围检查。
- **不瞎猜。** 动作中途断线就回报“未知”，绝不重复同一个动作。
- **主动权在你。** 机械臂动之前需要你批准，重新训练过的机械臂会再问一次。

## 越用越强

先从你手把手教的姿势开始。之后用 LeRobot 训练出更聪明的抓法，它会直接接在同一个能力后面，Flyto2 在启用新版本前先请你批准。任何 LeRobot 能驱动的机械臂都能加入，第一支是 AM-ARM200。

## 四步开始

1. 准备一支 AM-ARM200（从臂约 240 美元），以及一个伸手就能按到的 12V 电源开关。
2. 在连接机械臂的电脑上安装本包和 LeRobot。
3. 复制 `examples/am-arm200.json`，设置端口，教好姿势。
4. 把机械臂加入 Flyto2 Space，说出你想做的事。

## 目前状态

抢先预览版。所有能力都已用模拟机械臂和 Flyto2 Cloud 端到端测试。下一步是在真实的 AM-ARM200 上运行。

---

## 开发者

安装、配置、能力与测试的说明在英文 README 中。 [README.md → For developers](README.md#for-developers)

License: Flyto2 Source Available License 1.1 (`LICENSE`).
