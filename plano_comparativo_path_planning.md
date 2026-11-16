# Plano de Implementação: Análise Comparativa de Algoritmos de Planejamento de Trajetória na RoboCup SSL

> **Nota para o Agente / Desenvolvedor do Projeto:**  
> Este documento é a **especificação técnica autossuficiente e modular** para execução da pesquisa de iniciação científica (SIC / LabIND / UDESC) de análise comparativa de algoritmos de planejamento de trajetória na RoboCup Small Size League (SSL).
>
> Ele detalha a implementação das famílias de algoritmos (Grafos determinísticos, Amostragem estocástica e Campos Potenciais), do gerador de perfil trapezoidal temporal, das proteções de limites de campo e da bateria de testes automatizados no simulador **grSim** (visando rigor estatístico para publicação de artigos).
>
> **Repositório:** `https://github.com/Danielgmichels/labind-ssl-ai`  
> **Branch Base Oficial:** `main` (versão 3.3 estável da arquitetura tática)  
> *(Nota: A extensão avançada de longo prazo com CBF-QP, Splines C2 e Coordenação Multi-Robô encontra-se documentada separadamente no arquivo `plano_mestre_completo_ssl.md`.)*

---

## 1. Contexto da Liga, Premissas Físicas e Algoritmos Avaliados

### 1.1 Premissas da RoboCup SSL no grSim
* **Robôs:** Plataformas holonômicas omnidirecionais com raio físico $r_{bot} \approx 0,09\text{ m}$ ($90\text{ mm}$), operando em velocidades de $2,5\text{ a }4,0\text{ m/s}$ e acelerações de $4,0\text{ a }6,0\text{ m/s}^2$.
* **Ciclo de Controle:** A visão do simulador (`SSL-Vision` / `grSim`) opera a **60 Hz ($\Delta t \approx 16,67\text{ ms}$)**. O tempo de cálculo por ciclo ($T_{calc}$) deve permanecer muito abaixo de $16,67\text{ ms}$ para não gerar atraso temporal (*delay latency*).
* **Dimensões e Limites do Campo (Divisão A / Expandido):**
  * Campo útil: $X \in [-6,0; +6,0]\text{ m}$ e $Y \in [-4,5; +4,5]\text{ m}$.
  * Limites de segurança navegáveis (*Field Bounding Box*): $X \in [-5,80; +5,80]\text{ m}$ e $Y \in [-4,30; +4,30]\text{ m}$, garantindo que nenhum planejador gere rotas que tirem o robô ou a bola para fora das linhas demarcadas.
* **Comunicação com o grSim:** Os comandos enviados aos robôs utilizam o protocolo oficial `ssl_simulation_robot_control_pb2.RobotControl` (portas UDP 10302 para Amarelo e 10301 para Azul), com velocidades no **referencial local do robô**:
  * `move_command.local_velocity.forward` ($v_{forward}$): velocidade linear para frente/trás (m/s).
  * `move_command.local_velocity.left` ($v_{left}$): velocidade linear lateral (m/s).
  * `move_command.local_velocity.angular` ($v_w$): velocidade angular de rotação (rad/s).
  
  A conversão entre a velocidade linear no plano global $[v_x, v_y]^T$ e o referencial do robô com orientação $\theta_{robô}$ dá-se pela matriz de rotação:
  $$\begin{bmatrix} v_{forward} \\ v_{left} \end{bmatrix} = \begin{bmatrix} \cos(\theta_{robô}) & \sin(\theta_{robô}) \\ -\sin(\theta_{robô}) & \cos(\theta_{robô}) \end{bmatrix} \begin{bmatrix} v_x \\ v_y \end{bmatrix}$$

### 1.2 Algoritmos Selecionados para Comparação
Para a análise comparativa experimental, selecionamos representantes adaptados de cada família:

1. **Família de Busca em Grafo Determinístico:**
   * **VG Poligonal + A\*** *(Baseline clássica)*: Obstáculos inflados poligonizados; busca A* euclidiana.
   * **VG Poligonal + D\* Lite** *(Koenig & Likhachev; Barbosa et al., 2026)*: Replanejamento incremental que reutiliza a árvore de busca anterior quando obstáculos se movem.
   * **C-VG Circular + A\* Lazy** *(Maximo et al., ITAndroids / SBR 2025)*: Modela obstáculos como círculos perfeitos, calculando tangentes e arcos analíticos sem quinas agudas em $< 0,5\text{ ms}$.
2. **Família de Amostragem Aleatória (Estocástica):**
   * **RRT\* com Viés de Alvo e Passo Adaptativo** *(inspirado em ERRT e DBVS-APF-RRT\*, Feng et al., 2024)*: Amostragem contínua com atração para a meta, passo dependente da densidade local e simplificação de caminho (*shortcut*).
3. **Família Local / Reativa (Campos Potenciais Artificiais — APF):**
   * **APF Híbrido** *(alinhado a Gao et al., 2024)*: Forças atrativas e repulsivas combinadas com guiamento global por waypoints e força tangencial de escape para contornar mínimos locais.
4. **Camada Comum de Execução Temporal (Para Suavidade e Chegada sem Derrapagem):**
   * **Gerador de Trajetória Trapezoidal Sincronizado com $\vec{v}_0$** *(Rodrigues et al., RobôCIn 2024)*: Converte waypoints geométricos em perfis contínuos de posição e velocidade no tempo, sincronizando o início com a velocidade real estimada atual do robô ($\vec{v}(0) = \vec{v}_{atual}$) e respeitando restrição centrípeta de curvas ($v_{curva} \le \sqrt{a_{lat} \cdot R}$).

---

## 2. Estrutura Real do Repositório `labind-ssl-ai`

```text
labind-ssl-ai/
├── main.py                          # Loop principal e injeção de dependências
├── behavior_tree/                   # Motor de Árvores de Comportamento
│   ├── actions.py                   # Ações dos robôs (consomem o Blackboard)
│   ├── conditions.py                # Condições lógicas
│   ├── core.py                      # Framework da Behavior Tree
│   └── trees/                       # Árvores de atacante, goleiro, defesa, etc.
├── communication/                   # Clientes UDP e Protobuf
│   ├── ActionClient.py              # Envio de comandos aos robôs (RobotControl na porta 10302/10301)
│   ├── RefereeClient.py             # Leitura do Game Controller
│   └── VisionClient.py              # Leitura da visão SSL-Vision via ZMQ (porta 5558)
├── navigation/                      # PASTA EXISTENTE DE NAVEGAÇÃO
│   ├── APF.py                       # APF atual do projeto (preservado para compatibilidade)
│   ├── Obstacles.py                 # VirtualObstacle e paredes das grandes áreas
│   ├── interfaces.py                # [NOVO] IPathPlanner e ITrajectoryGenerator
│   ├── manager.py                   # [NOVO] NavigationManager (multi-robô, limites e proteções)
│   ├── factory.py                   # [NOVO] get_path_planner(nome)
│   ├── planners/                    # [NOVO] Algoritmos intercambiáveis
│   │   ├── vg_astar.py              # Algoritmo 1A: VG Poligonal + A*
│   │   ├── d_star_lite.py           # Algoritmo 1B: VG Poligonal + D* Lite
│   │   ├── c_vg.py                  # Algoritmo 1C: C-VG Circular + A* Lazy
│   │   ├── rrt_star.py              # Algoritmo 2: RRT* Adaptado
│   │   └── apf_hybrid.py            # Algoritmo 3: APF Híbrido com escape
│   └── trajectory/                  # [NOVO] Camada de temporização
│       └── trapezoidal.py           # Gerador Trapezoidal sincronizado com v0 e curvatura R
├── benchmarks/                      # [NOVO] Automação experimental e visualização
│   ├── visualize_path.py            # [NOVO] Visualizador 2D rápido no campo SSL (Debug visual)
│   ├── runner.py                    # Orquestrador com grSim_Replacement (porta UDP 20011)
│   ├── scenarios.py                 # Gerador de cenários estáticos e dinâmicos
│   ├── logger.py                    # Logger de telemetria a 60 Hz em CSV
│   └── plot_results.py              # Boxplots, curvas temporais e testes estatísticos (SciPy/Seaborn)
├── proto_msg/                       # Mensagens compiladas do Protocol Buffers (grSim_Packet_pb2, etc.)
├── strategy/                        # Análise tática e Maestro
├── tests/                           # Testes unitários com unittest
│   ├── test_shielding.py            # Teste existente
│   ├── test_tactical_scoring.py     # Teste existente
│   └── test_planners.py             # [NOVO] Testes unitários dos novos planejadores (MockRobotPos)
└── world/
    ├── Blackboard.py                # Quadro negro compartilhado (armazena controller/planner)
    └── world_model.py               # Modelo de mundo unificado
```

---

## 3. Integração Transparente e Proteções Robustas (`navigation/manager.py`)

### 3.1 Contrato Atual da Behavior Tree (`actions.py`)
No código atual de `behavior_tree/actions.py`, 19 nós de ação chamam a navegação da seguinte forma:
```python
vf, vl, vw = blackboard.controller.calculate_velocity(
    blackboard.my_pos.pos.x, 
    blackboard.my_pos.pos.y, 
    blackboard.my_pos.yaw, 
    alvo_x, 
    alvo_y, 
    blackboard.obstacles,
    target_yaw=None
)
```

### 3.2 O Adaptador Universal (`navigation/manager.py`)
O `NavigationManager` incorpora 5 salvaguardas essenciais:

1. **Estado Isolado Multi-Robô:** Gerenciamento de contexto de histórico e velocidade indexado por `robot_id`.
2. **Separação de Obstáculos vs. Zonas Proibidas:** Converte as ~86 paredes pontuais em duas caixas retangulares (`forbidden_zones`), evitando colapso combinatório de tangentes no C-VG e A*.
3. **Limites de Campo Rígidos (*Field Bounding Box*):** Alvos fora do campo são saturados para $X \in [-5,8; +5,8]\text{ m}$ e $Y \in [-4,3; +4,3]\text{ m}$.
4. **Desatolamento e Projeção de Alvo (*Unstuck & Goal Projection*):**
   * Se o robô começar dentro da margem de colisão de um adversário (tranco ou erro de visão), $start$ é projetado radialmente para fora antes de planejar.
   * Se o alvo (bola disputada) estiver dentro da margem de um adversário, projeta $goal$ para a borda externa.
5. **Estimação Dinâmica de $\vec{v}_0$ com Suavização:** Filtro exponencial sobre $\frac{\Delta p}{\Delta t}$ para alimentar o perfil trapezoidal.
6. **Predição Linear Opcional de Obstáculos Móveis ($\Delta t_{pred}$):** Projeta $c_j + \vec{v}_{obs, j} \cdot \Delta t_{pred}$ para antecipar adversários velozes.

---

## 4. Organização de Branches e Fluxo Git

A árvore de branches parte formalmente de `main`:

```text
main  (Branch oficial da equipe — NUNCA comitar diretamente nela)
  │
  └── feat/path-planning  (Branch integradora oficial da pesquisa de navegação da IC)
        │
        ├── feat/path-planner-interfaces     (Interfaces, Factory, NavigationManager, Visualizador 2D e Runner)
        │     └── [merge após validação] ──► feat/path-planning
        │
        ├── feat/trapezoidal-trajectory      (Perfilador Trapezoidal Sincronizado com v0 e Curvatura R)
        │     └── [merge após validação] ──► feat/path-planning
        │
        ├── feat/planner-circular-vg         (C-VG Circular + A* Lazy)
        │     └── [merge após validação] ──► feat/path-planning
        │
        ├── feat/planner-astar-vg            (Grafo de Visibilidade Poligonal + A*)
        │     └── [merge após validação] ──► feat/path-planning
        │
        ├── feat/planner-d-star-lite         (D* Lite Incremental Dinâmico)
        │     └── [merge após validação] ──► feat/path-planning
        │
        ├── feat/planner-rrt-star            (RRT* Adaptado com Viés de Alvo e Passo Variável)
        │     └── [merge após validação] ──► feat/path-planning
        │
        ├── feat/planner-apf-hybrid          (APF Híbrido com escape de mínimos locais)
        │     └── [merge após validação] ──► feat/path-planning
        │
        └── feat/benchmark-experiments       (Bateria automatizada de testes no grSim e gráficos estatísticos)
              └── [merge após validação] ──► feat/path-planning
```

---

## 5. Especificação Detalhada das Etapas de Implementação

---

### ETAPA 0: Interfaces, Fábrica de Algoritmos, Visualizador 2D e Infraestrutura
* **Branch:** `feat/path-planner-interfaces` (ramificada de `feat/path-planning`)

**Objetivo:** Criar os contratos abstratos, o gerenciador universal `NavigationManager` com proteção de limites de campo e desatolamento, o script de visualização visual 2D e o orquestrador `grSim_Replacement`.

#### Arquivos a Criar/Modificar:
1. `navigation/interfaces.py`:
   * Interfaces `IPathPlanner(ABC)` e `ITrajectoryGenerator(ABC)`.
2. `navigation/factory.py`:
   * Fábrica para chaveamento dinâmico: `"C_VG"`, `"VG_ASTAR"`, `"D_STAR_LITE"`, `"RRT_STAR"`, `"APF_HYBRID"`.
3. `navigation/manager.py`:
   * Gerenciador com suporte multi-robô, saturação de limites de campo, projeção de desatolamento e extração de `forbidden_zones`.
4. `benchmarks/visualize_path.py`:
   * Script independente usando `matplotlib` que desenha o campo da SSL em 2D com linhas, áreas, obstáculos circulares e traça os waypoints gerados pelo algoritmo para inspeção visual imediata.
5. `benchmarks/logger.py`:
   * Logger CSV a 60 Hz para gravação das métricas de navegação.
6. `benchmarks/runner.py`:
   * Utilitário de teletransporte e reset de cenário via `grSim_Replacement`.
7. `tests/test_planners.py`:
   * Testes unitários com `MockRobotPos`.

#### Critérios de Aceite:
* [ ] Executar `python3 -m unittest discover tests` com 100% de sucesso.
* [ ] `python3 benchmarks/visualize_path.py` plota o campo e gera um mapa de rota visual confirmando que a interface funciona.

---

### ETAPA 1: Gerador de Trajetória Trapezoidal Sincronizado com Curvatura
* **Branch:** `feat/trapezoidal-trajectory` (ramificada de `feat/path-planning`)

**Objetivo:** Converter caminhos geométricos em perfis contínuos no tempo com respeito aos limites de aceleração e frenagem, calculando o raio de curvatura $R$ nos waypoints para evitar paradas em cantos.

#### Especificações Técnicas (`navigation/trajectory/trapezoidal.py`):
1. **Sincronização com $\vec{v}_0$:** Partida a partir da velocidade estimada atual do robô.
2. **Cálculo de Curvatura $R$ nos Waypoints (Circuncentro de 3 Pontos):**
   Para cada quina intermediária formada por $(w_{i-1}, w_i, w_{i+1})$, o raio de curvatura local $R$ é calculado pelo círculo circunscrito:
   $$R = \frac{\|w_i - w_{i-1}\| \cdot \|w_{i+1} - w_i\| \cdot \|w_{i+1} - w_{i-1}\|}{2 \cdot |(x_i - x_{i-1})(y_{i+1} - y_{i-1}) - (y_i - y_{i-1})(x_{i+1} - x_{i-1})|}$$
   Velocidade limite na curva: $v_{lim, i} = \min(v_{max}, \sqrt{a_{lat, max} \cdot R})$.
3. **Perfil Unidimensional de 3 Fases (Aceleração, Cruzeiro e Frenagem):**
   Cálculo contínuo da distância de frenagem $d_{brake} = \frac{v^2 - v_{goal}^2}{2 a_{max}}$.
4. **Conversão para Referencial Local:** Geração de `v_forward`, `v_left` e controle angular para `target_yaw`.

#### Critérios de Aceite:
* [ ] O robô percorre sequências de waypoints em curvas sem parar a zero a cada nó.
* [ ] Parada suave e precisa sobre o alvo sem derrapagem excessiva.

---

### ETAPA 2: Grafo de Visibilidade Circular (C-VG) + A\* Lazy
* **Branch:** `feat/planner-circular-vg` (ramificada de `feat/path-planning`)

**Objetivo:** Implementar o Grafo de Visibilidade Circular (Maximo et al., 2025), padrão estado da arte em determinismo e suavidade na SSL.

#### Especificações Técnicas (`navigation/planners/c_vg.py`):
1. Modelagem analítica de tangentes Ponto-Círculo e Círculo-Círculo.
2. Interseção analítica Reta-Círculo via discriminante quadrático.
3. Arcos circulares de contorno e expansão preguiçosa (*Lazy A\**).
4. Tratamento de exceções (start/goal dentro da margem inflada).

#### Critérios de Aceite:
* [ ] Tempo médio de cálculo $< 1,0\text{ ms}$ no campo com 11 robôs.
* [ ] `visualize_path.py --planner C_VG` exibe tangentes e arcos contornando os círculos com folga de segurança.

---

### ETAPA 3: Grafo de Visibilidade Poligonal + A\* e D\* Lite
* **Branches:**
  * `feat/planner-astar-vg` (ramificada de `feat/path-planning`)
  * `feat/planner-d-star-lite` (ramificada de `feat/path-planning`)

**Objetivo:** Implementar o baseline determinístico poligonal com A\* e o algoritmo de replanejamento incremental D\* Lite (Koenig & Likhachev).

#### Critérios de Aceite:
* [ ] D\* Lite atinge caminhos equivalentes ao A\* nos mesmos grafos.
* [ ] D\* Lite reutiliza vértices consistentes com redução expressiva no número de expansões de nós durante movimentação contínua de obstáculos.

---

### ETAPA 4: RRT\* Adaptado para SSL
* **Branch:** `feat/planner-rrt-star` (ramificada de `feat/path-planning`)

**Objetivo:** Implementar o representante estocástico com viés de meta ($p_{goal} = 0,20$), passo adaptativo por densidade, teto temporal estrito (*anytime* $\le 3,0\text{ ms}$) e pós-processamento de atalhos (*shortcut pruning*).

#### Critérios de Aceite:
* [ ] O planejador nunca estoura o orçamento de tempo de $5,0\text{ ms}$.
* [ ] Caminho simplificado por atalhos elimina zig-zags da amostragem aleatória.

---

### ETAPA 5: Campos Potenciais Artificiais (APF Híbrido)
* **Branch:** `feat/planner-apf-hybrid` (ramificada de `feat/path-planning`)

**Objetivo:** Implementar o representante reativo com forças atrativas, repulsão hiperbólica com distância de corte $D_0 \approx 0,8\text{ m}$, guiamento global e força tangencial de escape para mínimos locais simétricos.

#### Critérios de Aceite:
* [ ] Robô contorna obstáculos frontais sem travar em equilíbrio repulsivo estático.

---

### ETAPA 6: Bateria Automatizada de Benchmarks e Análise Estatística (SIC / Publicação)
* **Branch:** `feat/benchmark-experiments` (ramificada de `feat/path-planning`)

**Objetivo:** Coleta experimental automatizada no grSim e análise estatística rigorosa para o relatório do SIC e submissão de artigo científico.

#### Dependências Científicas:
* `scipy` (para testes de normalidade e hipótese: `shapiro`, `mannwhitneyu`, `wilcoxon`).
* `seaborn` (para boxplots estatísticos com pontos sobrepostos e curvas KDE).

#### Especificações Técnicas:
1. **Cenários Padronizados (50 a 100 rodadas por algoritmo/cenário):**
   * Cenário A (Densidades Estáticas: 3, 6 e 11 robôs).
   * Cenário B (Pontos Aleatórios com Interceptadores Móveis a $2,5\text{ m/s}$).
   * Cenário C (Colisão Frontal Direta / *Head-on Encounter*).
2. **Métricas Avaliadas:**
   * Distância percorrida ($L$), tempo de cálculo ($T_{calc}$ em ms), tempo de chegada ($T_{total}$), suavidade (*Jerk* médio $J$) e taxa de sucesso sem colisões (%).
3. **Tratamento Estatístico e Gráficos:**
   * Teste de Shapiro-Wilk para normalidade.
   * Teste de hipótese não-paramétrico pareado (Mann-Whitney U / Wilcoxon) com significância $p < 0,05$.
   * Gráficos Box Plot comparativos e mapas de trajetórias sobrepostas exportados em 300 DPI em `benchmarks/results/`.

#### Critérios de Aceite:
* [ ] Relatório consolidado com tabelas (Média $\pm$ Desvio Padrão) e gráficos gerados automaticamente.
