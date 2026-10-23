<div align="center">

# Flyto2 × LeRobot Arms

**LeRobot アームに、ふつうの言葉で仕事を頼もう。アームが動き、Flyto2 が完了を確かめる。**

![Flyto2 device pack](https://img.shields.io/badge/Flyto2-device%20pack-6d5dfc) ![LeRobot](https://img.shields.io/badge/LeRobot-compatible-ffcc4d) ![Python](https://img.shields.io/badge/python-3.11%2B-3776ab)

[English](README.md) · **日本語** · [한국어](README.ko.md) · [简体中文](README.zh-CN.md) · [Español](README.es.md) · [Português (BR)](README.pt-BR.md)

[クイックスタート](#クイックスタート) · [ハイライト](#ハイライト) · [しくみ](#しくみ) · [開発者向け](#開発者向け)

</div>

## クイックスタート

```bash
# in the Python environment that runs Flyto2
git clone https://github.com/flytohub/flyto-modules-lerobot && cd flyto-modules-lerobot
pip install "lerobot @ git+https://github.com/liyiteng/lerobot_alohamini"
pip install -e .
cp examples/am-arm200.json my-arm.json        # set your port, teach your poses
export FLYTO_LEROBOT_ARM_CONFIG=$PWD/my-arm.json
```

アームを Flyto2 の Space に追加して、こう言うだけ:

> 治具の上の部品を取って、トレイに入れて。

## ハイライト

- **言葉で頼めば、本当に動く。** コードも経由点も、付きっきりの監視もいらない。
- **4つのスキルをすぐ使える。** 姿勢の把握、教えた姿勢への移動、つかむ、離す。
- **アームの自己申告は信じない。** アームは何をしたかを報告するだけ。Flyto2 はそれだけでは証明とみなさない。
- **止めたら、ピタッと止まる。** その場で姿勢を保ち、持っている物を落とさない。
- **決めた範囲から出ない。** すべての動きを、設定した可動範囲でチェック。
- **メーカーを越えて連携。** ほかのロボットと、ひとつの権限ルールで動く。

## しくみ

1. やってほしいことを言う。
2. Flyto2 が段取りを組み、権限を確認する。
3. このパックが LeRobot 経由でアームを動かす。
4. Flyto2 はアームの報告を記録するが、それだけでは証明として扱わない。

---

## 開発者向け

インストール、設定、能力、テストの詳細は英語版にあります。 [README.md → For developers](README.md#for-developers)

License: Flyto2 Source Available License 1.1 (`LICENSE`).
