# 🤖 SSL Tactical Engine - LabIND / Roboteam

![Python](https://img.shields.io/badge/Python-3.10-blue)
![Docker](https://img.shields.io/badge/Docker-Ready-2496ED)
![Status](https://img.shields.io/badge/Status-Ativo-success)

Este repositório contém o **Motor Tático e de Inteligência Artificial** desenvolvido para robôs da categoria **RoboCup Small Size League (SSL)**. O projeto foi estruturado como parte de uma Iniciação Científica (IC) no laboratório LabIND da Universidade do Estado de Santa Catarina (UDESC).

O sistema substitui abordagens monolíticas tradicionais por uma arquitetura moderna baseada em **Árvores de Comportamento (Behavior Trees)** para a tomada de decisão e **Campos Potenciais Artificiais (APF)** para navegação e desvio de obstáculos.

---

## 🌟 Principais Funcionalidades

* **🧠 Árvore de Comportamento (Behavior Tree):** Arquitetura modular (`Selector`, `Sequence`, `Action`, `Condition`) que permite a criação rápida de lógicas complexas sem aninhamento de `ifs`.
* **📡 Sistema Multi-Agente (O "Maestro"):** Distribuição dinâmica de papéis (Atacante, Goleiro, Zagueiros) em tempo real, baseada na distância da bola e do gol.
* **🧲 Navegação por APF Avançado:** Uso de Campos Potenciais Artificiais com correção **GNRON** para evitar mínimos locais e garantir uma navegação fluida em altas velocidades.
* **🛡️ Goleiro Inteligente:** Posicionamento matemático (bisseção de ângulo) para fechar o gol e lógica dinâmica de interceptação e cruzamento (Chip Kick) para limpar a área.
* **🐳 100% Dockerizado:** O ambiente roda de forma isolada, eliminando problemas de dependências e ambientes virtuais.

---

## 🏗️ Arquitetura do Projeto

O código está dividido de forma modular para facilitar a expansão tática:

```text
estrategia_labind/
├── main.py                # Loop principal (60Hz), leitura de rede e Maestro (Multi-Agente)
├── behavior_tree/         # O Cérebro da Inteligência Artificial
│   ├── __init__.py
│   ├── core.py            # Classes base da árvore (Node, Selector, Sequence)
│   ├── conditions.py      # Sensores lógicos (ex: IsNearBall, IsBallSafeToClear)
│   └── actions.py         # Músculos (ex: GoToBall, ClearBall, DefendGoal)
├── proto_msg/             # Arquivos compilados do Google Protobuf
├── Dockerfile             # Receita de infraestrutura
└── requirements.txt       # Dependências do Python