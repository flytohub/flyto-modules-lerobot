<div align="center">

# Flyto2 for LeRobot Arms

**Dile qué recoger. Un brazo de 240 dólares hace el trabajo, y otra cámara comprueba que de verdad se hizo.**

![Flyto2 device pack](https://img.shields.io/badge/Flyto2-device%20pack-6d5dfc) ![LeRobot](https://img.shields.io/badge/LeRobot-arms-ffcc4d) ![status](https://img.shields.io/badge/status-early%20preview-lightgrey)

[English](README.md) · [日本語](README.ja.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md) · **Español** · [Português (BR)](README.pt-BR.md)

</div>

---

## Una frase, trabajo real

*"Toma la pieza del soporte y ponla en la bandeja."* Flyto2 la convierte en pasos, envía a tu brazo los que le tocan y solo da el trabajo por terminado cuando se ha visto terminado.

## Lo que tu brazo puede hacer

- **Saber dónde está.** Cada articulación y la pinza, ahora mismo.
- **Ir a donde le enseñaste.** Posturas con nombre como *ready* o *tray*, nunca más allá de los límites que fijes.
- **Recoger.** Cerrar la pinza y sujetar.
- **Soltar.** Abrir la pinza y dejarlo.

## Trabaja con tus otros robots

Tu brazo no trabaja solo. Un robot móvil trae la pieza, el brazo la recoge y una cámara lo confirma, entre marcas distintas y bajo un mismo conjunto de permisos.

## Seguro por diseño

- **Parar significa quedarse quieto.** Nunca se afloja ni deja caer lo que sostiene.
- **Nunca se cree su propia palabra.** Otra cámara confirma el resultado.
- **No se sale de su carril.** Cada movimiento se comprueba contra tus límites.
- **Nada de adivinar.** Si la conexión se corta a mitad de un movimiento, dice *desconocido* y nunca repite el movimiento.
- **Tú mandas.** Mover el brazo necesita tu aprobación, y un brazo reentrenado vuelve a pedirla.

## Crece contigo

Empieza con posturas que enseñas a mano. Más adelante entrena un agarre más inteligente con LeRobot: encaja detrás de la misma capacidad, y Flyto2 te pide aprobar la nueva versión antes de usarla. Cualquier brazo que LeRobot controle puede unirse, empezando por el AM-ARM200.

## Empieza en cuatro pasos

1. Consigue un AM-ARM200 (unos 240 dólares el seguidor) y un interruptor de 12 V al alcance de la mano.
2. Instala este paquete y LeRobot en el ordenador al que se conecta el brazo.
3. Copia `examples/am-arm200.json`, configura tu puerto y enséñale las posturas.
4. Añade el brazo a un Space de Flyto2 y di lo que quieres hacer.

## Estado

Vista previa. Cada capacidad está probada de principio a fin con un brazo simulado y Flyto2 Cloud. Lo siguiente: las primeras pruebas con un AM-ARM200 real.

---

## Para desarrolladores

La instalación, la configuración, las capacidades y las pruebas están en el README en inglés. [README.md → For developers](README.md#for-developers)

License: Flyto2 Source Available License 1.1 (`LICENSE`).
