<div align="center">

# Flyto2 for LeRobot Arms

**Diga o que pegar. Um braço de 240 dólares faz o trabalho, e outra câmera confere se foi feito de verdade.**

![Flyto2 device pack](https://img.shields.io/badge/Flyto2-device%20pack-6d5dfc) ![LeRobot](https://img.shields.io/badge/LeRobot-arms-ffcc4d) ![status](https://img.shields.io/badge/status-early%20preview-lightgrey)

[English](README.md) · [日本語](README.ja.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md) · [Español](README.es.md) · **Português (BR)**

</div>

---

## Uma frase, trabalho de verdade

*"Pegue a peça no suporte e coloque na bandeja."* O Flyto2 transforma isso em etapas, envia ao seu braço as que são dele e só considera o trabalho concluído quando ele foi visto concluído.

## O que o seu braço pode fazer

- **Saber onde está.** Cada junta e a garra, agora mesmo.
- **Ir aonde você ensinou.** Posições com nome como *ready* ou *tray*, nunca além dos limites que você definiu.
- **Pegar.** Fechar a garra e segurar.
- **Soltar.** Abrir a garra e deixar no lugar.

## Trabalha com seus outros robôs

Seu braço não trabalha sozinho. Um robô móvel traz a peça, o braço pega e uma câmera confirma, entre marcas diferentes e sob um mesmo conjunto de permissões.

## Seguro desde o projeto

- **Parar é ficar parado.** Ele nunca amolece nem deixa cair o que segura.
- **Nunca acredita na própria palavra.** Outra câmera confirma o resultado.
- **Fica na sua faixa.** Cada movimento é conferido com os seus limites.
- **Sem chute.** Se a conexão cair no meio de um movimento, ele diz *desconhecido* e nunca repete o movimento.
- **Quem manda é você.** Mover o braço precisa da sua aprovação, e um braço retreinado pede de novo.

## Cresce com você

Comece com posições que você ensina à mão. Depois treine uma pegada mais inteligente com o LeRobot: ela entra por trás da mesma capacidade, e o Flyto2 pede sua aprovação antes de usar a nova versão. Qualquer braço que o LeRobot controla pode participar, começando pelo AM-ARM200.

## Comece em quatro passos

1. Tenha um AM-ARM200 (cerca de 240 dólares o seguidor) e um interruptor de 12 V ao alcance da mão.
2. Instale este pacote e o LeRobot no computador onde o braço é conectado.
3. Copie `examples/am-arm200.json`, configure sua porta e ensine as posições.
4. Adicione o braço a um Space do Flyto2 e diga o que você quer fazer.

## Status

Prévia. Cada capacidade foi testada de ponta a ponta com um braço simulado e o Flyto2 Cloud. Próximo passo: os primeiros testes com um AM-ARM200 real.

---

## Para desenvolvedores

Instalação, configuração, capacidades e testes estão no README em inglês. [README.md → For developers](README.md#for-developers)

License: Flyto2 Source Available License 1.1 (`LICENSE`).
