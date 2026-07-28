# 🤖 SSL Tactical Engine - LabIND / Roboteam

![Python](https://img.shields.io/badge/Python-3.10+-blue?style=for-the-badge&logo=python)
![Docker](https://img.shields.io/badge/Docker-Ready-2496ED?style=for-the-badge&logo=docker)
![Status](https://img.shields.io/badge/Status-Ativo-success?style=for-the-badge)

Este repositório contém o **Motor Tático e de Inteligência Artificial** desenvolvido para a equipe de robótica da **RoboCup Small Size League (SSL)**. O projeto foi estruturado como parte de uma Iniciação Científica (IC) no laboratório LabIND da Universidade do Estado de Santa Catarina (UDESC).

O sistema substitui abordagens monolíticas tradicionais por uma arquitetura moderna baseada em **Árvores de Comportamento (Behavior Trees)** para a tomada de decisão e **Campos Potenciais Artificiais (APF)** para navegação e desvio de obstáculos.

---

## 🌟 Principais Funcionalidades

*   **🧠 Arquitetura com Behavior Tree:** Motor de decisão modular (`Selector`, `Sequence`, `Action`, `Condition`) construído do zero, permitindo a criação de táticas complexas de forma clara e expansível.
*   **📡 Sistema Multi-Agente (O "Maestro"):** Algoritmo que distribui papéis dinamicamente (Atacante, Goleiro, Zagueiros, etc.) em tempo real, usando a posição da bola e lógica de histerese para garantir estabilidade tática.
*   **🧲 Navegação por APF Avançado:** Uso de Campos Potenciais Artificiais com "bolha de segurança" dinâmica e repulsão diferenciada para robôs e paredes, garantindo desvios suaves e eficientes.
*   **🛡️ Goleiro Inteligente:** Posicionamento preditivo para interceptar chutes e lógica de "fechar o ângulo" baseada na bisseção do ângulo entre a bola e as traves.
*   **👁️ Visualização com Groot:** Geração automática de arquivos XML compatíveis com o **Groot**, o editor visual padrão da indústria para Behavior Trees, permitindo depuração e design de táticas de forma gráfica.
*   **🐳 100% Dockerizado:** O ambiente de desenvolvimento e execução é totalmente encapsulado em um contêiner Docker, garantindo consistência e eliminando problemas de dependências.

---

## 🚀 Como Executar

O projeto foi desenhado para rodar em conjunto com o simulador **grSim** e o sistema de visão **SSL-Vision**. Certifique-se de que ambos estejam em execução na sua máquina.

1.  **Pré-requisitos:**
    *   Docker instalado e em execução.

2.  **Build da Imagem Docker:**
    No terminal, na raiz do projeto, execute o comando para construir a imagem:
    ```bash
    docker build -t estrategia_labind .
    ```

3.  **Executando a Estratégia:**
    Para rodar a estratégia para o time amarelo, utilize o comando abaixo. Ele conecta o contêiner à rede do seu computador (`--network=host`) para que ele possa se comunicar com o simulador.

    ```bash
    docker run --rm --network=host estrategia_labind --team yellow
    ```

    Para executar o time azul, basta alterar a flag `--team`:
    ```bash
    docker run --rm --network=host estrategia_labind --team blue
    ```

---

## 🎨 Visualizando a Estratégia com Groot

Uma das grandes vantagens deste projeto é a capacidade de visualizar e depurar a árvore de comportamento de forma gráfica.

1.  **Instale o Groot:** Baixe a versão mais recente para o seu sistema operacional no repositório oficial do Groot.

2.  **Gere a Paleta de Nós:** Execute o script `generate_palette.py` uma única vez para catalogar todas as `Actions` e `Conditions` do projeto.
    ```bash
    python3 generate_palette.py
    ```
    Isso criará o arquivo `diagramas/nodes_palette.xml`.

3.  **Gere o XML da Árvore:** Execute o `main.py`. Ele irá gerar automaticamente o arquivo `diagramas/arvore_mestra.xml` que representa a estrutura da sua estratégia.

4.  **Abra no Groot:**
    *   Inicie o Groot.
    *   Carregue a paleta de nós: **Load palette** -> **Load file** e selecione `diagramas/nodes_palette.xml`.
    *   Abra a sua árvore: **File** -> **Open** e selecione `diagramas/arvore_mestra.xml`.

---

## 🏗️ Arquitetura do Projeto

```text
estrategia_labind/
├── main.py                # Loop principal (60Hz): Percepção, Maestro e Execução da BT.
├── behavior_tree/         # Implementação do motor da Behavior Tree.
│   ├── core.py            # Classes base: Node, Selector, Sequence, NodeState.
│   ├── conditions.py      # "Sensores" da árvore (ex: IsNearBall, IsPathClear).
│   └── actions.py         # "Músculos" da árvore (ex: GoToBall, AimAndShoot).
├── proto_msg/             # Módulos Python compilados a partir dos arquivos .proto do Google Protobuf.
├── diagramas/             # Diagramas e arquivos de visualização (DOT, XML).
├── generate_palette.py    # Script para gerar a paleta de nós para o Groot.
├── Dockerfile             # Define o ambiente de execução conteinerizado.
└── requirements.txt       # Dependências Python (instaladas automaticamente pelo Docker).
```

## Arquitetura Tática (Behavior Tree)
A imagem abaixo mostra uma visualização estática da árvore de comportamento gerada a partir do código. Para uma visualização interativa, utilize o **Groot** conforme as instruções acima.

!Visualização da Árvore de Comportamento