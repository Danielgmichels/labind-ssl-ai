# Especificação Técnica e Arquitetural
## SSL Tactical Engine — Estratégia Multiagente para RoboCup Small Size League

**Projeto:** Motor de Estratégia e Inteligência Artificial para RoboCup Small Size League  
**Laboratório:** LabIND / UDESC  
**Equipe:** Roboteam  
**Versão da especificação:** 3.1 (Atualizada com Tactical Scoring — Shot & Pass Engines e Sincronia Multiagente)  
**Data:** Setembro de 2026

---

# 1. Objetivo do Sistema

O SSL Tactical Engine é um sistema de estratégia e controle multiagente desenvolvido para permitir que uma equipe de robôs autônomos participe de partidas da **RoboCup Small Size League (SSL)**.

O sistema tem como objetivo transformar informações provenientes da visão computacional e do árbitro em ações coordenadas para todos os robôs da equipe.

A arquitetura deve permitir que o sistema:

- interprete o estado atual da partida;
- identifique a posição e velocidade da bola;
- identifique robôs aliados e adversários;
- determine dinamicamente a função de cada robô;
- selecione comportamentos ofensivos e defensivos;
- navegue pelo campo evitando obstáculos;
- execute passes, chutes, cruzamentos e conduções;
- mantenha comportamento seguro durante interrupções da partida;
- reaja às mudanças do ambiente em tempo real;
- coordenar múltiplos robôs sem depender de uma máquina de estados monolítica.

A arquitetura adotada combina três níveis principais:

1. **Percepção e estado do mundo**;
2. **Decisão tática através de Behavior Trees**;
3. **Controle reativo e navegação através de Artificial Potential Fields (APF)**.

A decisão estratégica é realizada aproximadamente a **60 Hz**, permitindo que mudanças na posição da bola, dos robôs e do estado do árbitro sejam incorporadas rapidamente ao comportamento.

---

# 2. Arquitetura Geral

O sistema deve ser dividido conceitualmente nas seguintes camadas:

```text
SSL-Vision
    │
    │ ZMQ / Protobuf
    ▼
VisionClient
    │ dados brutos
    ▼
WorldModel
    │ estado limpo
    ▼
Blackboard
    │
    ├──────────────► Maestro
    │                   │
    │                   ▼
    │             Behavior Tree
    │                   │
    │                   ▼
    │                 Action
    │                   │
    │                   ▼
    │                  APF
    │                   │
    │                   ▼
    │              Controlador
    │                   │
    │                   ▼
    │                  Robô
    │
    └──────────────► Conditions / Actions
```

O `RefereeClient` fornece o estado regulamentar da partida ao fluxo de decisão.

A principal alteração arquitetural desta versão é a materialização do **Estado do Mundo** através do `WorldModel`. O `VisionClient` recebe os dados da SSL-Vision e os encaminha para o modelo de mundo. O `WorldModel` purifica e organiza os dados perceptivos antes que eles cheguem à estratégia.

O Blackboard deixou de ser a representação primária do estado perceptivo. Ele funciona como uma **camada de compatibilidade transitória**, mantendo a estrutura esperada pelas Conditions e Actions existentes.

O fluxo arquitetural principal passa a ser:

```text
Percepção
    ↓
Estado do Mundo
    ↓
Maestro
    ↓
Behavior Tree
    ↓
Action
    ↓
APF
    ↓
Controlador
    ↓
Robô
```

Essa separação reduz a responsabilidade do `main.py` e impede que dados brutos de visão, cálculos cinemáticos e decisões táticas sejam misturados no loop principal.

# 3. Modelo de Execução

O sistema deve operar continuamente em ciclos de aproximadamente:

```text
T_ciclo = 1 / 60 s
```

Cada ciclo deve executar as seguintes etapas:

### Etapa 1 — Atualização do árbitro

O sistema recebe a mensagem mais recente do árbitro.

São atualizados:

- comando atual;
- estágio da partida;
- posição designada da bola, quando existente;
- estado de interrupção;
- estado de bola em jogo.

O `RefereeClient` utiliza UDP multicast e socket não bloqueante para evitar que uma mensagem antiga impeça o processamento da informação mais recente.

### Etapa 2 — Atualização da percepção

O sistema recebe o estado mais recente da SSL-Vision através do `VisionClient`.

O socket utiliza `CONFLATE` para priorizar o pacote mais recente e evitar que o algoritmo processe estados atrasados.

O `VisionClient` é responsável pela aquisição e comunicação da percepção. Ele não deve assumir responsabilidades de decisão estratégica.

### Etapa 3 — Atualização do WorldModel

O `WorldModel` recebe os dados perceptivos mais recentes e atualiza a representação interna do campo.

Essa etapa é responsável por:

- atualizar a posição da bola;
- atualizar posição e orientação dos robôs aliados;
- atualizar posição e orientação dos robôs adversários;
- determinar explicitamente a visibilidade dos elementos;
- estimar a velocidade da bola;
- aplicar o filtro EMA à velocidade estimada;
- manter informações temporais necessárias para perda temporária de visão;
- disponibilizar um estado consistente para as demais camadas.

A estimativa e filtragem da velocidade da bola não fazem mais parte do loop principal. Essa responsabilidade é encapsulada no estado da bola dentro do `WorldModel`.

A implementação atual utiliza um filtro EMA com `alpha = 0.3` para `ball_vel_x` e `ball_vel_y`.

### Etapa 4 — Atualização do Blackboard

O `Blackboard` recebe os dados já processados pelo `WorldModel` e os organiza na estrutura esperada pela estratégia.

Sua função é compatibilizar o novo modelo de mundo com as Conditions e Actions existentes.

O Blackboard não deve:

- interpretar pacotes brutos da SSL-Vision;
- estimar a velocidade da bola;
- aplicar o filtro EMA;
- inferir visibilidade através de coordenadas zeradas;
- decidir papéis;
- executar navegação ou controle.

### Etapa 5 — Distribuição de papéis

O Maestro recebe:

- robôs aliados;
- estado limpo da bola;
- posição do gol adversário;
- papéis anteriores.

Com essas informações, determina o papel atual de cada robô.

### Etapa 6 — Execução multiagente

Para cada robô:

1. seleciona-se sua posição;
2. seleciona-se seu papel;
3. atualiza-se a lista de inimigos;
4. constrói-se sua lista de obstáculos;
5. configura-se a perspectiva do Blackboard;
6. executa-se a Behavior Tree correspondente;
7. o comportamento gera um comando.

A implementação atual reutiliza uma única instância do Blackboard, alterando sua perspectiva para cada robô antes da execução da árvore.

### Etapa 7 — Controle de frequência

O ciclo deve manter aproximadamente 60 Hz, respeitando o tempo gasto pelas etapas anteriores.

O `main.py` deve funcionar como **orquestrador do ciclo**, concentrando inicialização, dependências e ordenação das etapas, e não como local da lógica de percepção, estado do mundo, Maestro, construção das árvores, APF ou comunicação.

# 4. Comunicação e Infraestrutura

## 4.1 SSL-Vision

A visão deve fornecer:

- posição da bola;
- posição dos robôs aliados;
- posição dos robôs adversários;
- orientação dos robôs;
- visibilidade dos elementos.

A comunicação atual utiliza:

```text
ZeroMQ
Porta: 5558
Formato: Google Protobuf
Mensagem: State_pb2.State
```

O mecanismo `zmq.CONFLATE` deve ser mantido para priorizar o estado mais recente.

---

# 5. Comunicação com o Árbitro

O sistema recebe as mensagens do Game Controller através de multicast.

Configuração atual:

```text
IP multicast: 224.5.23.1
Porta: 10003
Protocolo: UDP
```

As mensagens devem permitir identificar pelo menos:

- HALT;
- STOP;
- NORMAL_START;
- FORCE_START;
- PREPARE_KICKOFF;
- DIRECT_FREE;
- INDIRECT_FREE;
- PREPARE_PENALTY;
- estágio da partida;
- posição designada da bola.

Quando disponível, a posição designada deve ser convertida de milímetros para metros antes de ser utilizada pelo algoritmo.

---

# 6. Comunicação com os Robôs

O sistema deve enviar comandos através de UDP.

A implementação utiliza:

```text
Time Azul: 10301
Time Amarelo: 10302
```

Cada comando deve permitir controlar:

```text
v_forward
v_left
angular_velocity
kick_speed
kick_angle
dribbler_speed
```

O robô recebe velocidades no sistema de coordenadas local.

O módulo de ação é responsável exclusivamente pela transmissão dos comandos e não deve conter decisões estratégicas.

---

# 7. Blackboard

O Blackboard funciona atualmente como uma **camada de compatibilidade** entre o `WorldModel` e a infraestrutura existente de Conditions e Actions.

Anteriormente, o Blackboard funcionava como o estado compartilhado que mesclava estado do árbitro, estado do robô, estado da bola, estado multiagente e informações temporárias de decisão. Após a refatoração, a representação e atualização do estado perceptivo pertencem ao `WorldModel`.

O Blackboard deve:

- receber dados limpos do `WorldModel`;
- disponibilizar esses dados no formato esperado pelas Behavior Trees;
- combinar, quando necessário, dados do árbitro e da estratégia;
- fornecer uma perspectiva específica para o robô em execução;
- preservar a compatibilidade com a estrutura Protobuf legada durante a migração.

O Blackboard não deve:

- receber diretamente pacotes brutos da SSL-Vision;
- calcular velocidade da bola;
- aplicar o EMA;
- determinar `visible` a partir de coordenadas;
- escolher papéis;
- executar APF;
- enviar comandos.

## 7.1 Estado do árbitro

```text
referee_command
referee_stage
bola_em_jogo_forcada
posicao_bola_no_apito
designated_position
```

## 7.2 Estado do robô

```text
my_id
my_role
my_pos
is_yellow
our_goal_x
enemy_goal_x
```

A perspectiva `my_*` é configurada para o robô cuja Behavior Tree está sendo executada.

## 7.3 Estado da bola

```text
ball_pos
ball_vel_x
ball_vel_y
last_ball_pos
last_ball_time
```

Esses dados são derivados do `WorldModel`, que é responsável pela atualização e filtragem da velocidade.

## 7.4 Estado multiagente

```text
team
enemies
obstacles
papeis
```

## 7.5 Informações temporárias de decisão e sincronia tática

```text
shot_decision               # Instância completa de ShotDecision com todos os alvos avaliados
best_shot_y                 # Coordenada Y da trave selecionada pelo Shot Scoring
pass_decision               # Instância completa de PassDecision com scores dos companheiros
pass_target_point           # Coordenada (x, y) do ponto de recepção escolhido
pass_target_robot           # ID do companheiro receptor escolhido pelo Pass Scoring
last_pass_time              # Timestamp (time.time()) do último disparo de passe
last_passer_id              # ID do robô que efetuou o passe recente (para Pass Lockout)
pass_in_progress_target     # ID do receptor em trânsito (para promoção antecipada no Maestro)
```

## 7.6 Compatibilidade e evolução

O Blackboard deve ser considerado uma **camada de transição arquitetural**.

Enquanto Conditions e Actions ainda dependem da estrutura Protobuf antiga, o Blackboard deve fornecer os dados no formato esperado sem reintroduzir responsabilidades de processamento.

A evolução desejada é:

```text
WorldModel
    ↓
Blackboard de compatibilidade
    ↓
Conditions / Actions
```

Posteriormente, Conditions e Actions podem consumir diretamente objetos de domínio do `WorldModel`, permitindo a remoção gradual da camada de compatibilidade.

# 8. WorldModel — Modelo de Mundo

O `WorldModel` é um componente **[IMPLEMENTADO]** e representa a materialização do conceito de **Estado do Mundo** na arquitetura.

Sua finalidade é separar a percepção bruta da tomada de decisão. Ele recebe os dados do `VisionClient`, normaliza e processa essas informações e disponibiliza uma representação consistente do ambiente para o Blackboard e para as camadas superiores.

## 8.1 Responsabilidades

O `WorldModel` deve ser responsável por:

- manter o estado atual da bola;
- manter o estado dos robôs aliados;
- manter o estado dos robôs adversários;
- manter posição e orientação quando disponíveis;
- determinar a visibilidade de cada elemento;
- manter timestamps de observação;
- estimar e filtrar a velocidade da bola;
- preservar o último estado válido quando apropriado;
- fornecer dados purificados para a estratégia.

O `WorldModel` não deve:

- escolher papéis;
- selecionar Behavior Trees;
- decidir chute, passe ou drible;
- enviar comandos;
- executar APF;
- substituir o Maestro.

## 8.2 Estado da bola

A bola possui um estado próprio, conceitualmente equivalente a um `BallState`.

Esse estado deve encapsular pelo menos:

```text
position
velocity
last_position
last_time
visible
```

Quando uma nova observação válida estiver disponível:

1. a posição atual é atualizada;
2. a velocidade instantânea é estimada;
3. a velocidade é filtrada;
4. o timestamp da observação é atualizado;
5. `visible` é atualizado para verdadeiro.

A filtragem utilizada atualmente é um EMA:

```text
v_filtrada =
    alpha × v_atual
    +
    (1 - alpha) × v_anterior
```

com:

```text
alpha = 0.3
```

O cálculo pertence ao `BallState`/`WorldModel`, e não ao loop principal.

## 8.3 Estado dos robôs

Robôs aliados e adversários devem possuir estados explícitos contendo, no mínimo:

```text
id
position
orientation
visible
```

Quando necessário, o estado pode manter:

```text
last_seen
```

## 8.4 Visibilidade

A visibilidade deve ser tratada como propriedade explícita do estado:

```text
ball.visible
robot.visible
```

Assim, Conditions e Actions não precisam utilizar verificações como:

```text
x == 0
y == 0
```

para inferir se um elemento foi detectado.

## 8.5 Purificação da percepção

O fluxo de dados deve ser:

```text
SSL-Vision
    ↓
VisionClient
    ↓
dados brutos
    ↓
WorldModel
    ↓
estado normalizado
    ↓
Blackboard
    ↓
estratégia
```

O objetivo é impedir que detalhes de comunicação, ausência de observação, filtragem matemática e representação bruta do Protobuf sejam espalhados pelas Conditions, Actions e pelo `main.py`.

## 8.6 Perda temporária de visão

O `WorldModel` deve fornecer a infraestrutura necessária para distinguir:

```text
elemento não observado neste ciclo
```

de:

```text
elemento fisicamente ausente
```

Para isso, deve manter pelo menos:

- timestamp da última observação;
- estado `visible`;
- último estado válido;
- possibilidade de aplicar timeout de validade.

A política de fallback permanece responsabilidade da estratégia.

## 8.7 Relação com o Blackboard

O `WorldModel` é a fonte primária do estado perceptivo limpo:

```text
WorldModel
    │
    ├── BallState
    ├── RobotState aliados
    └── RobotState adversários
             │
             ▼
        Blackboard
             │
             ▼
       Conditions / Actions
```

Essa separação deve ser preservada para impedir que o Blackboard volte a acumular responsabilidades do modelo de mundo.

## 8.8 Evolução planejada

A implementação atual pode evoluir para incorporar:

- confiança da percepção;
- estimativa de posição durante perda temporária;
- previsão de trajetória;
- velocidade dos robôs;
- detecção de eventos do mundo;
- histórico temporal mais rico;
- fusão de múltiplas fontes de percepção.

Essas extensões devem permanecer no modelo de mundo, sem transferir responsabilidades para o `main.py` ou para o Blackboard.

# 9. Maestro — Gerenciamento Multiagente

O Maestro é responsável por transformar a situação global do campo em uma distribuição de funções.

Ele não deve controlar diretamente a velocidade dos robôs.

Sua responsabilidade é responder:

> "Qual função cada robô deve desempenhar neste instante?"

---

# 10. Distribuição de Papéis

Os papéis atualmente suportados são:

```text
GOLEIRO

ATACANTE

ATACANTE_APOIO_ESQ
ATACANTE_APOIO_DIR
MEIA_ARMADOR

MEIO_CAMPO
VOLANTE

LATERAL_ESQUERDO
LATERAL_DIREITO

ZAGUEIRO_MARCACAO
ZAGUEIRO_BLOQUEIO

ESPERA
```

Esses papéis devem ser tratados como funções táticas e não como IDs fixos de robôs.

---

# 11. Goleiro

O goleiro possui prioridade absoluta sobre os demais papéis.

Por padrão, o robô definido como goleiro é o ID 0.

O goleiro deve:

1. proteger o gol;
2. fechar o ângulo;
3. acompanhar a trajetória da bola;
4. interceptar bolas que estejam indo em direção ao gol;
5. realizar clearance quando houver condições seguras.

A implementação atual determina um ponto de atuação próximo à linha do gol e calcula uma previsão da interseção entre a trajetória da bola e essa linha.

---

# 12. Seleção do Atacante

O atacante deve ser, inicialmente, o robô elegível mais próximo da bola.

Para evitar alternâncias rápidas entre dois robôs próximos da bola, deve existir histerese.

Na implementação atual, um robô que já é atacante recebe um bônus equivalente a 0,5 m na comparação de distância.

Formalmente:

```text
d_efetiva =
    d_bola - H, se robô já é atacante
    d_bola,     caso contrário
```

onde:

```text
H = 0,5 m
```

Esse valor deve ser tratado como parâmetro configurável.

### 12.1 Transferência Dinâmica durante Passe em Trânsito

Para evitar que o passador dispute a bola com o receptor após disparar um passe, o Maestro incorpora consciência temporal de eventos de passe:

Se `(tempo_atual - last_pass_time) < 1.2 s` e existir um `pass_target_robot`:
1. O passador anterior **perde** o bônus de atacante ($H = 0.0\text{ m}$);
2. O receptor intencional (`pass_target_robot`) recebe uma bonificação virtual de proximidade:
   $$d_{efetiva} = d_{bola} - 2.0\text{ m}$$
3. O receptor é promovido imediatamente a **ATACANTE** antes mesmo da bola atingir sua posição física, garantindo transição suave e permitindo que o passador desacelere com segurança.

---

# 13. Distribuição Geométrica dos Demais Papéis

Após selecionar o goleiro e atacante, os demais robôs devem ser distribuídos considerando principalmente sua posição relativa ao gol adversário.

A estratégia atual separa os robôs em:

### Linha ofensiva

- atacante de apoio esquerdo;
- meia armador;
- atacante de apoio direito.

### Linha defensiva

- zagueiro bloqueio;
- zagueiro marcação;
- volante.

### Região intermediária

- lateral esquerdo;
- meio-campo;
- lateral direito.

A ordenação pelo eixo Y também é utilizada para evitar que robôs troquem excessivamente de lado.

---

# 14. Histerese de Papéis

A redistribuição dos papéis não deve ocorrer somente com base na posição instantânea.

Deve existir memória do papel anterior.

O objetivo é evitar:

```text
Robô A → atacante
Robô B → atacante
Robô A → atacante
Robô B → atacante
...
```

Essa alternância é prejudicial porque provoca:

- cruzamento de trajetórias;
- perda de posicionamento;
- atraso na tomada de decisão;
- disputa entre robôs pela bola.

Portanto, cada papel deve possuir um custo de troca ou bônus de permanência.

---

# 15. Behavior Tree

A Behavior Tree é responsável pela decisão local de cada robô.

O motor possui os seguintes estados:

```text
SUCCESS
FAILURE
RUNNING
```

E os principais tipos de nós são:

```text
Selector
Sequence
Condition
Action
```

O `main.py` constrói uma árvore mestra que seleciona o comportamento correspondente ao papel recebido do Maestro.

---

# 16. Prioridade Global da Behavior Tree

Independentemente do papel, as decisões devem obedecer à seguinte hierarquia:

```text
1. Segurança / HALT
2. Regras impostas pelo árbitro
3. Situações de kickoff
4. Bola parada
5. Defesa emergencial
6. Comportamento tático
7. Comportamento ofensivo
8. Posicionamento padrão
9. Stop / fallback
```

Essa prioridade é importante porque uma ação ofensiva nunca deve sobrepor um comando HALT.

---

# 17. Behavior Tree do Atacante

O atacante é o robô com a árvore de decisão mais rica da equipe.

A prioridade ofensiva na versão 3.1 foi reestruturada para garantir agressividade ofensiva e evitar recuos prematuros ou estagnação contra defesas em barreira:

```text
1. Receber passe (ConditionIsPassArriving → ActionInterceptPass)
         ↓
2. Finalizar a gol (ConditionEvaluateShot → ActionAimAndShoot)
         ↓
3. Passe avançado para frente (ConditionEvaluatePass(only_forward=True) → ActionPassBall)
         ↓
4. Encontrar ângulo / Strafe lateral (ConditionIsInShootingZone → ActionFindShootingAngle)
         ↓
5. Conduzir com drible evasivo (ActionSmartDribble)
         ↓
6. Passe de recuo / Manutenção de posse (ConditionEvaluatePass(only_forward=False) → ActionPassBall)
         ↓
7. Buscar bola com Pass Lockout (ActionGoToBall)
```

Essa estrutura assegura que o atacante primeiro explore todas as possibilidades de finalização ou aceleração da jogada à frente, utilize o drible lateral para tirar barreiras estáticas da frente do gol e, apenas se essas tentativas falharem ou expirarem, opte pelo recuo controlado para girar a posse de bola.

---

# 18. Recepção de Passe (Universal)

A interceptação ativa de passes deixou de ser exclusiva dos atacantes de apoio, passando a ser **universal** para todas as funções de apoio e transição (`ATACANTE_APOIO_ESQ`, `ATACANTE_APOIO_DIR`, `MEIA_ARMADOR`, `MEIO_CAMPO`, `LATERAL_ESQUERDO`, `LATERAL_DIREITO` e `VOLANTE`).

Um robô ativa a interceptação quando a bola:

- estiver suficientemente rápida;
- estiver dentro do raio operacional;
- estiver se deslocando na direção do robô.

A condição `ConditionIsPassArriving` utiliza:

```text
velocidade mínima > 0,6 m/s
distância máxima ≤ 3,0 m
dot product > 0,7 (cone de ~45º em direção ao robô)
```

Ao detectar o passe, o robô executa a `ActionInterceptPass`, projetando a trajetória vetorial da bola e movimentando-se para recebê-la de frente com o dispositivo de drible acionado (`dribbler_speed = 1500 RPM`), eliminando recuos passivos ou fuga da bola.

---

# 19. Finalização

A finalização deve ocorrer quando:

1. o robô possui a posse de bola (`dist_bola < 0.12 m`);
2. está na zona ofensiva de finalização (`dist_gol < 4.0 m`);
3. o motor de avaliação de chute aprova um alvo com score aceitável (`score >= 0.58`).

---

# 20. Seleção do Alvo de Chute — Shot Scoring Engine [IMPLEMENTADO]

A seleção do alvo de chute foi completamente reformulada da heurística linear anterior para um **motor de pontuação multi-critério baseado em geometria e risco** (`strategy/shot_scoring.py`).

O algoritmo discretiza a meta adversária em **11 alvos candidatos** ao longo da linha do gol:

```text
targets_y = [-0.45, -0.38, -0.30, -0.20, -0.10, 0.00, 0.10, 0.20, 0.30, 0.38, 0.45]
```

Para cada alvo candidato, é calculada uma pontuação normalizada entre 0.0 e 1.0 composta por 6 fatores ponderados:

```text
Score_chute =
    0.25 * opening_score
  + 0.20 * angle_score
  + 0.10 * distance_score
  + 0.15 * progression_score
  + 0.25 * blocking_score
  + 0.05 * risk_score
```

### Componentes de Avaliação:

1. **Abertura da Trave (`opening_score`, peso 0.25):** Distância do adversário/goleiro mais próximo ao ponto específico da trave nos últimos 1.5m de campo.
2. **Alinhamento Angular (`angle_score`, peso 0.20):** Diferença angular entre a orientação atual do robô (`yaw`) e o vetor de mira até o alvo.
3. **Distância (`distance_score`, peso 0.10):** Proximidade à meta adversária (normalizada até 6.0m).
4. **Progressão (`progression_score`, peso 0.15):** Avanço longitudinal normalizado `bx / goal_x`, aumentando de 0.0 no meio-campo para 1.0 na linha de gol.
5. **Bloqueio de Trajetória (`blocking_score`, peso 0.25) — Regra Rígida:**
   - Calcula a menor distância ortogonal de todos os robôs (aliados e adversários) ao segmento de reta bola $\rightarrow$ alvo.
   - **Hard Block:** se qualquer robô estiver dentro do raio de colisão física (`collision_radius = 0.16 m`), o `blocking_score = 0.0` e o candidato é **imediatamente invalidado** (`score = 0.0`), impedindo chutes em barreiras ou companheiros.
   - Entre 0.16m e a margem de segurança (`safe_margin = 0.35 m`), a pontuação cresce linearmente até 1.0.
6. **Pressão / Risco (`risk_score`, peso 0.05):** Proximidade do adversário mais próximo da bola (pressão de marcação).

O alvo com maior pontuação $\ge 0.58$ é salvo em `blackboard.best_shot_y` e transmitido para a `ActionAimAndShoot`.

---

# 21. Execução do Chute (Sniper)

A `ActionAimAndShoot` alinha o robô com a coordenada `blackboard.best_shot_y`.

A execução requer tolerância angular rigorosa:

```text
|erro_angular| < 0.03 rad (~1.7 graus)
```

Ao atingir a precisão, o robô freia a translação (`vf = 0.0, vl = 0.0`), liga o chutador na potência máxima regulamentar (`kick_speed = 6.0 m/s`) e aciona o driblador para garantir contato firme e limpo na liberação da bola.

---

# 22. Passe — Pass Scoring Engine [IMPLEMENTADO]

A seleção de receptores de passe foi modernizada para um **motor de pontuação multi-critério** (`strategy/pass_scoring.py`), substituindo a seleção linear do primeiro companheiro com linha desimpedida.

O motor avalia simultaneamente todos os companheiros visíveis e elegíveis (excluindo o próprio passador, o goleiro e robôs fora da faixa de 0.70m a 6.00m).

Cada companheiro é avaliado segundo 7 fatores ponderados:

```text
Score_passe =
    0.25 * progression_score
  + 0.15 * space_score
  + 0.15 * reception_angle_score
  + 0.10 * distance_score
  + 0.15 * support_score
  + 0.15 * interception_score
  + 0.05 * density_score
```

### Componentes de Avaliação:

1. **Progressão Longitudinal (`progression_score`, peso 0.25):**
   - Passes para frente ($\Delta_{prog} \ge 0$): nota de 0.50 a 1.00 baseada no ganho de metros em direção ao gol inimigo.
   - Passes para trás ($\Delta_{prog} < 0$): penalizados proporcionalmente à distância recuada (nota entre 0.00 e 0.30 no campo ofensivo), preservando sua viabilidade como recurso de giro de jogo, mas impedindo que superem jogadas de ataque.
2. **Espaço Livre (`space_score`, peso 0.15):** Distância do receptor ao adversário mais próximo (raio confortável $\ge 1.5\text{ m}$).
3. **Ângulo de Recepção (`reception_angle_score`, peso 0.15):** Facilidade angular para o receptor dominar a bola e dar continuidade à jogada em direção ao gol.
4. **Distância Operacional (`distance_score`, peso 0.10):** Penalização de passes curtos demais (< 1.0m) ou excessivamente longos (> 5.0m), com pico ideal entre 2.0m e 3.5m.
5. **Função Tática do Receptor (`support_score`, peso 0.15):** Priorização baseada na tabela de papéis:
   - `ATACANTE_APOIO_ESQ` / `DIR`: 1.00
   - `MEIA_ARMADOR`: 0.90
   - `LATERAL_ESQUERDO` / `DIREITO`: 0.75
   - `MEIO_CAMPO`: 0.65
   - `VOLANTE`: 0.40
   - `ZAGUEIROS`: 0.15 ~ 0.20
6. **Linha de Intercepção (`interception_score`, peso 0.15) — Regra Rígida:**
   - Projeção ortogonal de todos os adversários sobre o segmento passador $\rightarrow$ receptor.
   - Se houver adversário a menos de `collision_radius = 0.18 m`, `interception_score = 0.0` e o candidato é **anulado**.
7. **Densidade de Oponentes (`density_score`, peso 0.05):** Contagem de oponentes em um raio de 1.20m em torno do receptor.

O melhor candidato com score $\ge 0.50$ é selecionado. A condição `ConditionEvaluatePass` suporta o modo `only_forward=True` (exige $\text{progression\_score} \ge 0.50$) e `only_forward=False` (permite recuos).

---

# 23. Potência e Mecânica do Passe

A execução do passe pela `ActionPassBall` segue três princípios mecânicos e de controle fundamentais:

1. **Força Mínima de Chute:**
   A potência do chute é calculada por:
   $$kick\_speed = \max(2.5, \min(distancia \times 2.2, 6.0))$$
   garantindo que mesmo passes curtos (1.0m) sejam disparados a no mínimo $2.5\text{ m/s}$, impedindo que a bola morra prematuramente pelo atrito com o piso/carpete.
2. **Desligamento do Driblador na Soltura:**
   Ao atingir o alinhamento angular ($< 0.02\text{ rad}$), o comando zera a velocidade do motor de drible (`dribbler_speed = 0.0`), permitindo que a bola se desprenda limpa da frente do robô sem atrito reverso ou desaceleração.
3. **Pass Lockout e Sincronia Temporal:**
   No milissegundo do disparo, o passador registra:
   ```python
   blackboard.last_pass_time = time.time()
   blackboard.last_passer_id = blackboard.my_id
   blackboard.pass_in_progress_target = pass_target_robot
   ```
   - O passador ativa um *lockout* de 1.0 segundo na `ActionGoToBall`, impedindo-o de perseguir seu próprio passe.
   - O Maestro utiliza esse registro para transferir imediatamente o papel de `ATACANTE` ao receptor antes da chegada física da bola.

---

# 24. Condução, Drible Evasivo e Busca de Ângulo

Quando não há oportunidade limpa de finalização nem passe para frente, o atacante dispõe de dois recursos ativos antes de recorrer ao recuo de bola:

### 24.1 Busca de Ângulo Lateral (`ActionFindShootingAngle`)
- O robô mantém o driblador a 1500 RPM segurando a bola e executa passadas laterais (*strafing* com $v_l = \pm 1.0\text{ m/s}, v_f \approx 0.1\text{ m/s}$) mantendo a orientação apontada para o gol adversário.
- Objetivo: deslocar-se lateralmente para sair de trás da barreira estática e abrir um corredor de finalização para os cantos da trave.
- Possui salvaguardas de estabilidade: encerra após 1.5s ou se aproximar das linhas laterais do campo ($|y| > 2.8\text{m}$), retornando `FAILURE` para autorizar outras ações (drible ou passe de recuo).

### 24.2 Drible Evasivo Frontal (`ActionSmartDribble`)
- Conduz a bola em direção ao gol adversário.
- Ao detectar um adversário a menos de 0.8m no seu cone frontal de visão, reduz a velocidade para frente e aplica velocidade lateral evasiva na direção oposta ao obstáculo para contornar a marcação mantendo o controle da posse.

---

# 25. Apoios Ofensivos

Os robôs de apoio devem:

- manter largura no ataque;
- evitar concentração no centro;
- fornecer linhas de passe;
- receber passes;
- participar de cruzamentos;
- manter posições estratégicas quando a bola estiver em outra região.

Os apoios esquerdo e direito possuem posições laterais distintas e agora operam com **posicionamento dinâmico por espaço livre** (`strategy/space_scoring.py`).

### 25.1 Free-Space Positioning Engine [IMPLEMENTADO]

Em vez de se fixarem rigidamente em coordenadas estáticas ($y = \pm 2.5\text{ m}, x = \text{gol} - 2.5\text{ m}$), os robôs de apoio avaliam uma grade espacial de candidatos na zona ofensiva e intermediária através do motor multi-critério:

```text
Score_posicao =
    0.25 * space_score
  + 0.25 * pass_line_score
  + 0.15 * support_score
  + 0.15 * progression_score
  + 0.10 * defensive_cover_score
  + 0.10 * safety_score
```

#### Componentes de Avaliação:
1. **Espaço Livre (`space_score`, peso 0.25):** Distância ao adversário mais próximo, recompensando regiões desmarcadas e isoladas da defesa.
2. **Linha de Passe (`pass_line_score`, peso 0.25):** Raycast da bola até o candidato. Se a reta contiver adversários a menos de $0.18\text{m}$, o score cai para 0.0 (linha bloqueada).
3. **Distância de Suporte (`support_score`, peso 0.15):** Distância ideal em relação ao portador da bola (faixa ideal de 1.8m a 3.5m).
4. **Progressão (`progression_score`, peso 0.15):** Proximidade longitudinal à meta adversária.
5. **Cobertura Defensiva (`defensive_cover_score`, peso 0.10):** Posicionamento que assegura recomposição rápida em caso de perda de posse.
6. **Segurança (`safety_score`, peso 0.10):** Distância segura das linhas laterais do campo e áreas de penalidade.

#### Histerese e Estabilidade:
Para evitar oscilações a 60 Hz na escolha de posição:
- A coordenada candidata correspondente ao alvo atual recebe um bônus de estabilidade (`stability_bonus = 0.15`).
- Uma nova posição só substitui o alvo atual se superar sua pontuação por uma margem mínima (`switch_margin = 0.05`).

O ponto vencedor é salvo em `blackboard.free_space_target` e consumido diretamente pela `ActionPositionForPass`, que utiliza o APF para navegar até o ponto enquanto mantém o robô orientado para a bola com o driblador a 1500 RPM. Caso a condição falhe, o sistema utiliza o posicionamento lateral clássico como fallback seguro.

---

# 26. Meio-Campo

O meia armador deve funcionar como elo entre:

```text
defesa ↔ meio ↔ ataque
```

Suas principais funções são:

- oferecer linha de passe;
- aproximar-se da jogada;
- ocupar espaços livres;
- auxiliar na progressão da bola;
- participar da criação ofensiva;
- recompor defensivamente quando necessário.

---

# 27. Volante

O volante deve priorizar:

- proteção da defesa;
- cobertura;
- bloqueio de linhas de passe;
- formação defensiva em bola parada;
- apoio à saída de bola.

Na implementação atual, o volante possui um comportamento específico para formação de barreira durante faltas adversárias.

---

# 28. Laterais

Os laterais devem:

- manter largura;
- evitar congestionamento central;
- acompanhar adversários em bolas paradas;
- avançar pelas laterais;
- participar de cruzamentos.

Os comportamentos atuais utilizam posições laterais fixadas no eixo Y durante determinadas situações.

---

# 29. Zagueiros

Os zagueiros devem possuir duas funções complementares:

### Zagueiro de marcação

Responsável por acompanhar o adversário mais perigoso.

### Zagueiro de bloqueio

Responsável por ocupar a linha defensiva e bloquear trajetórias de chute ou passe.

A seleção dos adversários para marcação atualmente utiliza a distância do adversário até o próprio gol como indicador de perigo.

---

# 30. Defesa em Bola Parada

Quando ocorre uma falta adversária, a equipe deve formar uma barreira.

O posicionamento deve ser calculado geometricamente a partir de:

```text
bola
   ↓
linha em direção ao gol
   ↓
posição do defensor
```

A posição deve respeitar os limites estabelecidos pelas regras oficiais da competição.

A implementação atual posiciona o centro da barreira ao longo da direção bola → gol e aplica um deslocamento perpendicular para distribuir os robôs.

Os valores de distância devem ser parametrizados e validados de acordo com a versão das regras da SSL utilizada pela equipe.

---

# 31. Goleiro

O goleiro deve possuir três comportamentos prioritários:

```text
HALT
 ↓
CLEAR BALL
 ↓
DEFEND GOAL
```

A árvore atual utiliza exatamente essa estrutura conceitual.

---

# 32. Defesa Preditiva

O goleiro deve utilizar a velocidade da bola para prever sua trajetória.

Se:

```text
velocidade da bola > limiar
```

e a componente X indicar que a bola está se aproximando do gol, o sistema calcula a interseção da trajetória com a linha de defesa.

Caso contrário, o goleiro utiliza a direção bola → centro do gol para fechar o ângulo.

O alvo final é limitado ao intervalo permitido de atuação do goleiro.

---

# 33. Clear da Área

O goleiro pode tentar retirar a bola da área quando:

```text
bola está dentro da área
E
bola possui baixa velocidade
```

A implementação atual utiliza:

```text
velocidade < 1,2 m/s
```

como condição de segurança.

A finalidade dessa condição é evitar que o goleiro abandone sua função defensiva para tentar dominar uma bola que esteja chegando rapidamente.

---

# 34. Artificial Potential Field

O APF é responsável pela navegação local.

O objetivo do APF é transformar:

```text
posição atual
+
objetivo
+
obstáculos
```

em:

```text
velocidade linear
+
velocidade lateral
+
velocidade angular
```

A velocidade de atração é determinada pelo vetor entre o robô e o alvo:

```text
V_atr = K_atr × (P_alvo - P_robo)
```

A magnitude da atração deve ser limitada por:

```text
V_max
```

---

# 35. Repulsão

Para cada obstáculo:

```text
V_rep = K_rep / d²
```

aplicado na direção oposta ao obstáculo.

A implementação atual diferencia dois tipos:

```text
robô físico
parede virtual
```

As paredes recebem uma força de repulsão significativamente maior e uma distância de atuação maior.

---

# 36. Bolha de Segurança Dinâmica

O raio de segurança não deve ser constante.

A implementação atual aumenta o raio de segurança de acordo com a velocidade do robô.

Modelo atual:

```text
dist_segura_dinamica =
    dist_segura + fator_velocidade × 0,20
```

A força repulsiva também aumenta com a velocidade.

Essa abordagem permite:

- maior distância de reação em alta velocidade;
- maior aproveitamento de espaços em baixa velocidade;
- redução de colisões;
- navegação mais suave.

---

# 37. Filtro Direcional do APF

Um problema conhecido do APF tradicional é a repulsão causada por obstáculos localizados atrás do robô.

Para evitar esse comportamento, o sistema deve calcular:

```text
V_mov · V_obs
```

onde:

- `V_mov` é o vetor desejado de movimento;
- `V_obs` é o vetor do robô para o obstáculo.

Se:

```text
V_mov · V_obs < 0
```

o obstáculo está atrás do vetor de movimento e sua repulsão pode ser ignorada.

A implementação atual possui exatamente esse mecanismo.

---

# 38. Obstáculos Virtuais

As áreas próximas aos gols devem ser representadas no APF através de obstáculos virtuais.

Esses obstáculos têm como objetivo impedir que robôs de campo atravessem regiões que não devem ser utilizadas durante determinadas ações.

O goleiro é uma exceção e pode utilizar uma lista de obstáculos diferente para permitir movimentação dentro da área defensiva.

A implementação atual adiciona as paredes virtuais a todos os robôs que não sejam goleiros.

---

# 39. Separação entre Decisão e Controle

Um princípio fundamental do sistema deve ser:

```text
Behavior Tree:
"O que fazer?"

APF:
"Como chegar ao objetivo?"

ActionClient:
"Como enviar o comando?"
```

Por exemplo:

```text
BT:
"Posicionar-se para receber passe"

        ↓

Action:
"Alvo = posição de apoio"

        ↓

APF:
"Desviar dos obstáculos até o alvo"

        ↓

ActionClient:
"Enviar vf, vl e ω"
```

Isso evita que regras estratégicas sejam misturadas com controle cinemático.

---

# 40. Condições Principais

As Conditions devem ser classificadas por função.

## Árbitro

```text
ConditionIsGameRunning
ConditionIsHalted
ConditionIsOurPrepareKickoff
ConditionIsEnemyPrepareKickoff
ConditionIsOurFreeKick
ConditionIsEnemyFreeKick
```

## Bola

```text
ConditionIsNearBall
ConditionIsPassArriving
ConditionIsBallSafeToClear
```

## Ataque

```text
ConditionIsInShootingZone
ConditionIsPathClear
ConditionIsPassClear
```

## Papel

```text
ConditionCheckRole
```

As Conditions devem ser responsáveis somente por avaliar o estado e retornar:

```text
SUCCESS
FAILURE
```

Não devem enviar comandos aos robôs.

---

# 41. Actions Principais

As Actions devem ser responsáveis pela execução.

Entre as ações existentes estão:

```text
ActionStopMotors
ActionPrepareKickoff
ActionGoToBall
ActionSmartDribble
ActionAimAndShoot
ActionFindShootingAngle
ActionPositionForPass
ActionPassBall
ActionInterceptPass
ActionDefendGoal
ActionClearBall
ActionFormDefensiveWall
ActionMarkEnemy
ActionPositionForShortPass
ActionPositionForCross
ActionCrossToBox
ActionPositionForKickoff
```

Cada Action deve representar uma intenção comportamental específica.

---

# 42. Convenção de Estados

Toda Action deve respeitar:

### SUCCESS

A ação terminou e atingiu seu objetivo.

### FAILURE

A ação não pode ser executada ou não possui dados suficientes.

### RUNNING

A ação ainda está sendo executada e deve continuar no próximo ciclo.

Exemplo:

```text
ActionGoToBall
        │
        ├── SUCCESS → chegou à bola
        ├── FAILURE → bola/robô indisponível
        └── RUNNING → ainda navegando
```

---

# 43. Tratamento de Perda de Visão

O sistema deve ser tolerante a perda temporária de dados.

Caso:

```text
bola não esteja visível
```

o comportamento não deve assumir imediatamente que a bola desapareceu fisicamente.

Deve existir:

- timestamp da última observação;
- timeout de validade;
- estado de confiança;
- comportamento de fallback.

O mesmo princípio deve ser aplicado aos robôs.

---

# 44. Confiança da Percepção

Uma evolução recomendada é adicionar ao Blackboard:

```text
ball_confidence
robot_confidence
vision_timestamp
```

permitindo decisões como:

```text
confiança alta:
    utilizar posição atual

confiança média:
    utilizar posição estimada

confiança baixa:
    executar comportamento seguro
```

Isso é especialmente importante para uma estratégia que pretende sair do simulador e operar com visão real.

---

# 45. Segurança

O sistema deve possuir uma camada de segurança superior à estratégia.

A ordem deve ser:

```text
HALT
 ↓
STOP
 ↓
restrições regulamentares
 ↓
estratégia
 ↓
controle
```

Nenhuma Action ofensiva deve conseguir ignorar um comando HALT.

O `ActionStopMotors` já implementa a parada completa das velocidades, chute e driblador.

---

# 46. Configuração

Valores atualmente espalhados no código devem progressivamente ser centralizados.

Exemplo:

```text
config/
    field.yaml
    apf.yaml
    strategy.yaml
    robots.yaml
    referee.yaml
```

Parâmetros como:

```text
kp_linear
kp_angular
max_vel
max_angular_vel
dist_segura
kr_repulsao
histerese_atacante
shooting_zone
pass_speed_factor
dribble_distance
ball_speed_threshold
```

não devem ficar fixos dentro das Actions e Conditions.

---

# 47. Máquina de Estados Estratégica de Alto Nível

Além da Behavior Tree, a estratégia deve ser documentada através de estados macro:

```text
                 ┌──────────────┐
                 │     HALT     │
                 └──────┬───────┘
                        │
                        ▼
              ┌──────────────────┐
              │  PREPARAÇÃO      │
              │ Kickoff/Falta    │
              └────────┬─────────┘
                       │
                       ▼
              ┌──────────────────┐
              │     JOGO         │
              └────────┬─────────┘
                       │
           ┌───────────┼────────────┐
           ▼           ▼            ▼
       Ataque       Defesa       Transição
           │           │            │
           └───────────┼────────────┘
                       ▼
                    JOGO
```

A Behavior Tree implementa as decisões detalhadas dentro desses estados.

---

# 48. Transições Ofensivas

O atacante deve seguir uma lógica aproximada:

```text
Sem posse
    ↓
Buscar bola
    ↓
Possession
    ├── chute possível → finalizar
    ├── passe possível → passar
    ├── ângulo ruim → abrir ângulo
    └── adversário bloqueando → driblar
```

---

# 49. Transições Defensivas

A equipe deve reagir à perda da posse:

```text
Perdeu posse
     ↓
Identificar bola
     ↓
Identificar ameaça
     ↓
Redistribuir papéis
     ↓
Bloquear linha de ataque
     ↓
Recompor formação
```

O Maestro deve ser responsável pela reorganização dos papéis.

---

# 50. Bola Parada

A estratégia deve diferenciar:

```text
Kickoff nosso
Kickoff adversário
Falta nossa
Falta adversária
Pênalti
```

Cada situação deve possuir:

- formação;
- posicionamento;
- distância mínima;
- robô responsável;
- ação após início da jogada;
- fallback caso a bola seja movimentada.

A implementação já possui condições distintas para kickoff e faltas. 
---

# 51. Detecção de Bola em Jogo

Em determinadas situações de bola parada, o sistema pode precisar detectar que a bola começou a se mover.

A implementação atual compara:

```text
velocidade da bola
```

e

```text
distância percorrida desde o apito
```

para determinar se deve abandonar a formação de bola parada.

Essa lógica deve ser documentada como mecanismo de transição e não como "hack", pois conceitualmente ela representa um **detector de transição entre estado de bola parada e bola em jogo**.

---

# 52. Arquitetura de Software

A estrutura recomendada é:

```text
ssl_tactical_engine/
│
├── main.py
│
├── behavior_tree/
│   ├── core.py
│   ├── conditions.py
│   ├── actions.py
│   └── trees.py
│
├── strategy/
│   ├── maestro.py
│   ├── roles.py
│   ├── formations.py
│   └── tactical_state.py
│
├── navigation/
│   ├── apf.py
│   ├── obstacles.py
│   └── controller.py
│
├── communication/
│   ├── vision.py
│   ├── referee.py
│   └── action.py
│
├── world/
│   ├── blackboard.py
│   ├── world_model.py
│   └── states.py
│
├── config/
│
├── tests/
│
├── experiments/
│
├── diagrams/
│
└── proto_msg/
```

O `WorldModel` já foi incorporado e reduziu a sobrecarga do `main.py`. O `main.py` deve permanecer como orquestrador do runtime, enquanto comunicação, estado do mundo, Blackboard, Maestro, Behavior Trees, APF e controle permanecem encapsulados em seus respectivos módulos.

---

# 53. Groot e Visualização

Toda Behavior Tree deve poder ser exportada para XML.

A visualização deve permitir identificar:

- sequência de decisão;
- Conditions;
- Actions;
- prioridade;
- estados;
- papel do robô.

O repositório atual já possui geração de XML e paleta de nós para Groot.

---

# 54. Testes

O sistema deve possuir testes separados em quatro níveis.

## Testes unitários

Validar:

- APF;
- cálculo de distância;
- cálculo angular;
- seleção de papel;
- histerese;
- raycast;
- interceptação;
- seleção de chute.

## Testes de integração

Validar:

```text
VisionClient → WorldModel
RefereeClient → estado do árbitro
WorldModel → Blackboard
Blackboard → Maestro
Maestro → BT
BT → ActionClient
```

## Testes comportamentais

Cenários:

- bola parada;
- bola livre;
- bola atrás do robô;
- dois robôs próximos;
- adversário bloqueando chute;
- adversário bloqueando passe;
- bola rápida;
- bola entrando na área;
- perda da visão.

## Testes estatísticos

Executar o mesmo cenário diversas vezes com pequenas perturbações.

---

# 55. Motor Experimental

A versão futura do sistema deve possuir um módulo separado para experimentos científicos.

Esse módulo deve permitir:

```text
executar N partidas
variar parâmetros
registrar resultados
calcular métricas
gerar gráficos
comparar versões
```

Importante: o documento anterior especificava um cenário de 20 execuções, jitter aleatório e análise estatística, mas essa funcionalidade não está representada no `main.py` enviado. Portanto, ela deve ser classificada como **módulo planejado**, e não como funcionalidade atualmente implementada.

---

# 56. Métricas

A estratégia deve ser avaliada utilizando métricas objetivas.

## Ofensivas

```text
taxa de conversão de chutes
taxa de passes completos
taxa de passes interceptados
taxa de perda de posse
tempo médio até finalização
tempo médio até progressão
```

## Defensivas

```text
gols sofridos
chutes adversários bloqueados
interceptações
recuperações de posse
tempo de recomposição
```

## Navegação

```text
distância percorrida
tempo de deslocamento
número de colisões
distância mínima entre robôs
número de travamentos
tempo em deadlock
```

## Multiagente

```text
número de trocas de papel
tempo médio de permanência em papel
número de conflitos pela bola
ocupação espacial
```

---

# 57. Experimentos com APF

A avaliação do APF deve comparar pelo menos:

```text
APF tradicional
vs.
APF com bolha dinâmica
vs.
APF com filtro direcional
vs.
APF completo
```

As variáveis experimentais devem incluir:

```text
densidade de obstáculos
velocidade do robô
posição inicial
posição do alvo
número de robôs
```

A hipótese principal é que o filtro direcional e a bolha dinâmica reduzam travamentos e colisões sem prejudicar significativamente o tempo de navegação.

---

# 58. Experimentos com Maestro

O Maestro deve ser avaliado em situações nas quais dois ou mais robôs possuem distâncias semelhantes à bola.

Comparar:

```text
sem histerese
vs.
com histerese
```

Métricas:

```text
trocas de atacante
conflitos pela bola
tempo até posse
distância total percorrida
```

---

# 59. Experimentos com Chute

Comparar:

```text
chute sempre no centro
vs.
seleção de trajetória
vs.
seleção de trajetória ponderada
```

Registrar:

```text
taxa de gol
taxa de bloqueio
distância do chute
ângulo do chute
posição dos defensores
```

---

# 60. Registro de Dados

Cada experimento deve gerar um log contendo:

```text
timestamp
frame
posição da bola
velocidade da bola

ID do robô
posição
orientação
papel

ação selecionada
condições satisfeitas
objetivo atual

velocidade enviada
chute
driblador

distância aos obstáculos
```

Isso permitirá reconstruir posteriormente a tomada de decisão.

---

# 61. Requisitos de Desempenho

O sistema deve:

- operar aproximadamente a 60 Hz;
- não bloquear o ciclo principal esperando mensagens;
- utilizar sempre o estado mais recente disponível;
- evitar processamento desnecessário de frames antigos;
- limitar velocidades máximas;
- evitar comandos indefinidos;
- possuir fallback seguro.

O loop atual já foi estruturado para operar a 60 Hz e utilizar sockets não bloqueantes.

---

# 62. Requisitos de Robustez

O sistema deve continuar funcional diante de:

- perda temporária da bola;
- perda temporária de robô;
- atraso de comunicação;
- ausência de adversários detectados;
- ausência de companheiros;
- bola fora da região esperada;
- mudança de comando do árbitro;
- redistribuição de papéis;
- obstáculos muito próximos.

Nenhuma dessas condições deve resultar em exceção não tratada capaz de interromper o motor estratégico.

---

# 63. Requisitos de Segurança

O sistema deve impedir:

- envio de velocidades acima dos limites configurados;
- entrada não autorizada em áreas restritas;
- chute durante estados de parada;
- movimentação durante HALT;
- conflito entre ações simultâneas;
- comandos com dados inválidos.

As restrições regulamentares devem ser isoladas em uma camada de regras configurável.

---

# 64. Parâmetros Atuais de Referência

Os seguintes valores representam a configuração atualmente observada na implementação e devem ser tratados como **parâmetros experimentais**, não como constantes definitivas:

```text
Loop:
    60 Hz

Controle:
    kp_linear = 2.0
    kp_angular = 2.0
    max_vel = 2.5 m/s
    max_angular_vel = 5.0 rad/s

APF:
    dist_segura = 0.20 m
    kr_repulsao = 0.017

Parede virtual:
    raio de atuação ≈ 0.85 m
    kr ≈ 3.0

Atacante:
    histerese ≈ 0.50 m

Posse:
    distância < 0.12 m

Zona de chute:
    distância ao gol < 4.0 m

Recepção:
    velocidade > 0.6 m/s
    distância < 3.0 m
    dot product > 0.7

Drible:
    adversário < 0.8 m
    cone frontal ≈ 0.6 rad

Clear:
    velocidade da bola < 1.2 m/s
```

Esses valores devem ser movidos para configuração e posteriormente obtidos através de experimentação.

---

# 65. Pontos que Devem Ser Melhorados na Implementação

A refatoração alterou o status de alguns itens que anteriormente estavam registrados como trabalho futuro. O `WorldModel`, em particular, deixou de ser uma melhoria planejada e passou a ser parte efetiva da arquitetura.

### 65.1 Separação do `main.py`

**Status: [PARCIALMENTE IMPLEMENTADO]**

A sobrecarga arquitetural do `main.py` foi reduzida pela introdução do `WorldModel` e pela separação dos módulos de comunicação, estratégia, navegação e Behavior Trees.

O `main.py` deve permanecer como orquestrador do runtime, concentrando essencialmente:

- inicialização dos componentes;
- criação das dependências;
- atualização do ciclo;
- coordenação da ordem de execução.

Responsabilidades como comunicação, processamento do estado do mundo, construção das árvores, Maestro e APF não devem ser reintroduzidas no arquivo.

### 65.2 Centralização de parâmetros

**Status: [PLANEJADO]**

Evitar valores mágicos espalhados pelo código.

### 65.3 Modelo de mundo

**Status: [IMPLEMENTADO]**

O `WorldModel` foi incorporado ao sistema e passou a ocupar a camada entre percepção e Blackboard.

A implementação deve ser mantida como a fonte de estado perceptivo limpo, incluindo:

- estado da bola;
- estado dos robôs;
- visibilidade;
- timestamps;
- velocidade estimada e filtrada da bola.

### 65.4 Sistema de confiança

**Status: [PLANEJADO]**

Adicionar confiança às informações perceptivas. Timestamps já fazem parte da infraestrutura necessária do modelo de mundo; a política de confiança permanece como evolução futura.

### 65.5 Seleção de ações por score

**Status: [PLANEJADO]**

Evoluir de uma lógica puramente sequencial para uma avaliação quantitativa de alternativas.

### 65.6 Sistema experimental

**Status: [PLANEJADO]**

Separar claramente:

```text
runtime
```

de:

```text
experimentos científicos
```

### 65.7 Registro de decisões

**Status: [PLANEJADO]**

Salvar qual Condition determinou cada Action.

# 66. Inconsistências que Devem Ser Corrigidas na Documentação

A especificação deve distinguir três categorias:

```text
[IMPLEMENTADO]
[PARCIALMENTE IMPLEMENTADO]
[PLANEJADO]
```

Isso é particularmente importante para evitar que a documentação científica afirme que uma funcionalidade existe quando ela ainda está apenas planejada.

Por exemplo:

### Implementado

- Behavior Tree;
- Maestro;
- distribuição dinâmica de papéis;
- histerese;
- APF;
- bolha dinâmica;
- filtro direcional;
- comunicação com visão;
- comunicação com árbitro;
- `WorldModel`;
- estado explícito de visibilidade da bola e dos robôs;
- estimativa e filtragem EMA da velocidade da bola encapsuladas no estado da bola;
- controle UDP;
- chute;
- passe;
- drible;
- goleiro preditivo;
- formação defensiva;
- execução a 60 Hz;
- exportação para Groot.

Esses elementos podem ser diretamente observados na implementação atual. 
### Planejado / a validar

- motor experimental completo;
- execução automatizada de 20 partidas;
- avaliação estatística;
- renderização científica das trajetórias;
- comparação sistemática entre versões do APF;
- otimização automática dos parâmetros.

---

# 67. Critérios de Aceitação

A estratégia será considerada funcional quando:

### Comunicação

- receber estados da visão;
- receber comandos do árbitro;
- enviar comandos aos robôs.

### WorldModel

- receber dados do `VisionClient`;
- atualizar o estado da bola;
- atualizar o estado dos robôs;
- expor `visible` explicitamente;
- estimar e filtrar a velocidade da bola;
- fornecer estado limpo ao Blackboard;
- manter informações temporais para perda de visão.

### Segurança

- parar imediatamente em HALT;
- respeitar limites de velocidade;
- evitar obstáculos.

### Maestro

- atribuir corretamente os papéis;
- evitar troca excessiva de atacante;
- adaptar-se ao número de robôs disponíveis.

### Behavior Tree

- selecionar comportamento de acordo com o papel;
- tratar kickoff;
- tratar faltas;
- tratar ataque;
- tratar defesa.

### Ataque

- buscar bola;
- conduzir;
- passar;
- receber;
- chutar;
- procurar ângulo.

### Defesa

- goleiro acompanhar bola;
- zagueiros marcar;
- volante proteger;
- laterais recompor;
- formação de barreira funcionar.

### Navegação

- alcançar objetivos;
- evitar obstáculos;
- evitar paredes virtuais;
- reduzir deadlocks;
- funcionar em alta densidade de robôs.

---

# 68. Objetivo Científico

O projeto não deve ser apresentado apenas como um sistema de controle de robôs.

O objetivo científico deve ser investigar uma arquitetura híbrida para tomada de decisão multiagente em futebol robótico, combinando:

```text
Percepção
     +
Coordenação Multiagente
     +
Behavior Trees
     +
Navegação por APF
     +
Controle em tempo real
```

A principal hipótese arquitetural é que a separação entre:

```text
decisão estratégica
```

e:

```text
controle reativo
```

permita aumentar a modularidade, interpretabilidade e capacidade de evolução da estratégia.

---

# 69. Evolução Futura

A arquitetura deve permitir posteriormente incorporar:

- previsão da trajetória da bola;
- previsão da movimentação adversária;
- formação dinâmica e Threat Model;
- planejamento de trajetórias contínuas;
- aprendizado de parâmetros e otimização dos pesos de scoring;
- avaliação automática de estratégias por métricas quantitativas.

> [!NOTE]
> Os três motores táticos centrais — **seleção de chute por score** (Fase 1), **seleção de passe por score** (Fase 2) e **posicionamento dinâmico por espaço livre** (Fase 3) —, anteriormente listados como evolução futura, foram plenamente **[IMPLEMENTADOS]** na versão 3.1 como módulos stateless independentes em `strategy/shot_scoring.py`, `strategy/pass_scoring.py` e `strategy/space_scoring.py`.

---

# 70. Estado da Refatoração e Evolução Tática

Esta versão consolida os seguintes pilares fundamentais no projeto:

1. **Materialização do Modelo de Mundo (`WorldModel`):**
   - Elimina a dependência de dados brutos de visão no loop de decisão.
   - Encapsula a cinemática e filtragem EMA da velocidade da bola.
   - Atribui visibilidade explícita a todos os agentes e bola.
   - Transforma o Blackboard em camada de compatibilidade transitória e desacoplada.

2. **Motores de Decisão Tática Explicável (Shot, Pass & Free-Space Scoring):**
   - Substituição de regras rígidas e seleções do primeiro caminho livre por funções de avaliação geométrica multi-critério normalizadas (0.0 a 1.0).
   - Bloqueio físico absoluto (*hard block*) contra adversários e barreiras.
   - Resolução dos conflitos mecânicos de passe através de velocidade mínima (2.5 m/s), desligamento do driblador no disparo, *Pass Lockout* de 1 segundo e transferência imediata de papel no Maestro.
   - Recepção de passe universal para todas as posições da equipe.
   - Posicionamento ofensivo e de apoio baseado em zonas de espaço livre com desobstrução de linha de passe e histerese temporal.

---

# 71. Testes Automatizados e Validação

A robustez da tomada de decisão e da coordenação multiagente é assegurada por uma suíte de testes unitários automatizados localizada em `tests/test_tactical_scoring.py`.

A suíte cobre **19 cenários determinísticos**, incluindo:

1. **Shot Scoring (5 testes):**
   - Gol aberto: centro do gol altamente pontuado.
   - Centro bloqueado: seleção precisa e desimpedida dos cantos da trave ($y = \pm 0.45\text{ m}$).
   - Barreira completa: rejeição segura de finalizações inviáveis (retorna `None` e mantém posse).
   - Aliado bloqueando: aliados tratados estritamente como obstáculos físicos.
   - Progressão longitudinal: recompensa proporcional ao avanço da bola.
2. **Pass Scoring (9 testes):**
   - Seleção de receptor desmarcado em relação a companheiro bloqueado.
   - Rejeição absoluta de trajetórias com oponente interceptando a reta.
   - Comparação entre receptor avançado e recuado (preferência pelo ganho territorial).
   - Validação da viabilidade de passes de recuo como alternativa segura de reciclagem de posse.
   - Funcionamento do filtro de avanço rápido `only_forward=True`.
   - Proteção de auto-perseguição (*Pass Lockout*) na `ActionGoToBall`.
   - Transferência de papel no `Maestro` promovendo o receptor antes da chegada física da bola.
   - Integração com as Conditions da Behavior Tree (`ConditionEvaluateShot` e `ConditionEvaluatePass`).
3. **Free-Space Positioning (5 testes):**
   - Identificação precisa de regiões de espaço livre e desmarcadas na ala ofensiva.
   - Contorno de linhas de passe obstruídas por adversários.
   - Histerese temporal com `stability_bonus` e `switch_margin`, impedindo oscilações indesejadas a 60 Hz.
   - Integração com o nó de condição `ConditionEvaluateFreeSpace`.
   - Execução de navegação APF com `ActionPositionForPass` consumindo o `free_space_target`.

---

# 72. Princípio Arquitetural Final

O princípio fundamental do SSL Tactical Engine deve ser:

```text
                 PERCEPÇÃO
                     │
                     ▼
              ESTADO DO MUNDO
                     │
        ┌────────────┴────────────┐
        ▼                         ▼
TACTICAL SCORING               MAESTRO
(Shot & Pass Engines)    "Quem faz o quê?"
        │                         │
        └────────────┬────────────┘
                     ▼
               BEHAVIOR TREE
           "Qual intenção tática?"
                     │
                     ▼
                  ACTION
               "Qual alvo?"
                     │
                     ▼
                   APF
             "Como chegar lá?"
                     │
                     ▼
               CONTROLADOR
            "Qual velocidade?"
                     │
                     ▼
                  ROBÔ
```

A arquitetura preserva com rigor a separação de responsabilidades:

O **WorldModel mantém e purifica o estado perceptivo do mundo**.

O **Tactical Scoring avalia e ranqueia geometricamente as melhores oportunidades de finalização e passe**.

O **Maestro coordena a distribuição dinâmica e estável dos papéis da equipe**.

A **Behavior Tree decide quando disparar cada intenção tática**.

As **Actions executam as intenções e comandam os atuadores**.

O **APF navega de forma reativa desviando de colisões**.

O **controlador converte vetores de navegação em comandos cinemáticos de motor**.

O **ActionClient transmite os comandos em tempo real ao simulador/robôs físicos**.

Essa separação modular constitui a base arquitetural do SSL Tactical Engine.
