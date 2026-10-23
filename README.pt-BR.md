<div align="center">

# Flyto2 × LeRobot Arms

**Dê trabalho ao seu braço LeRobot com palavras do dia a dia. Ele faz, e o Flyto2 confere se ficou pronto.**

![Flyto2 device pack](https://img.shields.io/badge/Flyto2-device%20pack-6d5dfc) ![LeRobot](https://img.shields.io/badge/LeRobot-compatible-ffcc4d) ![Python](https://img.shields.io/badge/python-3.11%2B-3776ab)

[English](README.md) · [日本語](README.ja.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md) · [Español](README.es.md) · **Português (BR)**

</div>

## Início rápido

```bash
# in the Python environment that runs Flyto2
git clone https://github.com/flytohub/flyto-modules-lerobot && cd flyto-modules-lerobot
pip install "lerobot @ git+https://github.com/liyiteng/lerobot_alohamini"
pip install -e .
cp examples/am-arm200.json my-arm.json        # set your port, teach your poses
export FLYTO_LEROBOT_ARM_CONFIG=$PWD/my-arm.json
```

Adicione o braço a um Space do Flyto2 e diga:

> Pega a peça do suporte e coloca na bandeja.

## Destaques

- **Palavras simples, movimento de verdade.** Sem código, sem pontos de passagem, sem ficar de babá.
- **Quatro habilidades, prontas para usar.** Saber a posição, ir às posições ensinadas, pegar, soltar.
- **Não acredita só na palavra do braço.** O braço conta o que fez. O Flyto2 nunca trata isso como prova sozinho.
- **Parar é congelar.** Segura a posição e nunca derruba o que carrega.
- **Dentro dos seus limites.** Cada movimento é conferido com as faixas que você definiu.
- **Time entre marcas.** Trabalha com seus outros robôs sob uma mesma regra de permissões.

## Como funciona

1. Você diz o que quer.
2. O Flyto2 planeja e confere as permissões.
3. Este pacote move o braço pelo LeRobot.
4. O Flyto2 registra o que o braço relata, mas nunca trata isso como prova sozinho.

---

## Para desenvolvedores

Instalação, configuração, capacidades e testes estão na versão em inglês. [README.md → For developers](README.md#for-developers)

License: Flyto2 Source Available License 1.1 (`LICENSE`).
