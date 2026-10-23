<div align="center">

# Flyto2 × LeRobot Arms

**Dale trabajo a tu brazo LeRobot con palabras normales. Lo hace, y Flyto2 comprueba que quedó hecho.**

![Flyto2 device pack](https://img.shields.io/badge/Flyto2-device%20pack-6d5dfc) ![LeRobot](https://img.shields.io/badge/LeRobot-compatible-ffcc4d) ![Python](https://img.shields.io/badge/python-3.11%2B-3776ab)

[English](README.md) · [日本語](README.ja.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md) · **Español** · [Português (BR)](README.pt-BR.md)

</div>

## Inicio rápido

```bash
# in the Python environment that runs Flyto2
git clone https://github.com/flytohub/flyto-modules-lerobot && cd flyto-modules-lerobot
pip install "lerobot @ git+https://github.com/liyiteng/lerobot_alohamini"
pip install -e .
cp examples/am-arm200.json my-arm.json        # set your port, teach your poses
export FLYTO_LEROBOT_ARM_CONFIG=$PWD/my-arm.json
```

Añade el brazo a un Space de Flyto2 y di:

> Coge la pieza del soporte y ponla en la bandeja.

## Lo mejor

- **Palabras normales, movimiento real.** Sin código, sin puntos de paso, sin vigilarlo.
- **Cuatro habilidades, listas para usar.** Saber su postura, ir a posturas enseñadas, agarrar, soltar.
- **No se fía de la palabra del brazo.** El brazo informa de lo que hizo. Flyto2 nunca lo toma como prueba por sí solo.
- **Parar es congelarse.** Mantiene la posición y nunca suelta lo que lleva.
- **Dentro de tus límites.** Cada movimiento se comprueba con los rangos que fijaste.
- **Equipo entre marcas.** Trabaja con tus otros robots bajo un mismo conjunto de permisos.

## Cómo funciona

1. Dices lo que quieres.
2. Flyto2 lo planifica y revisa los permisos.
3. Este paquete mueve el brazo a través de LeRobot.
4. Flyto2 registra lo que informa el brazo, pero nunca lo toma como prueba por sí solo.

---

## Para desarrolladores

La instalación, la configuración, las capacidades y las pruebas están en la versión en inglés. [README.md → For developers](README.md#for-developers)

License: Flyto2 Source Available License 1.1 (`LICENSE`).
