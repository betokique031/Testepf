# AF 24b — Explicação dos códigos (avaliacao_af)

Documento de consulta: o que cada arquivo faz, como se conectam e onde ajustar.

---

## Visão geral da arquitetura

Tudo segue o padrão **`base_control.py`** do Cap 3: cada nó tem um dicionário `state_machine` que mapeia o nome do estado para uma função, um atributo `robot_state` com o estado atual, e uma função `control()` rodando num timer (0.1 s) que executa o estado atual e é **a única que publica em `/cmd_vel`**.

Existem dois tipos de nó:

- **Nós de ação** (`seguelinha.py`, `girar.py`): fazem uma tarefa motora simples e contínua (seguir linha, girar). Têm o próprio `control()` e publicam `cmd_vel` enquanto ativos.
- **Nós cliente** (`q1.py`): orquestram a lógica da questão. Eles *compõem* os nós de ação (criam instâncias deles), decidem quando cada ação roda, e ficam de olho nos sensores (aruco, yolo, handler). Quando o cliente está num "estado cliente", quem publica `cmd_vel` é o nó de ação; nos outros estados, o cliente publica (geralmente `Twist()` zerado pra parar).

A `q2.py` é um nó único (não usa cliente + ação separados) porque a tarefa é mais direta.

---

## q1.py — Exercício 1 (ExplorandoOrdenado)

**Objetivo:** seguir a linha, em cada bifurcação ler o Aruco e perguntar ao Handler se vai pra direita ou esquerda, achar o animal (gato/cachorro/cavalo), avisar o Handler e voltar ao ponto de partida.

**Herança:** `Node, Odom, Laser`. Usa odometria pra medir distâncias (cooldown, viés, retorno) e laser pra não bater na placa na volta.

**Compõe:** `SegueLinha` (segue a linha) e `Girar` (vira 180° na volta).

### Tópicos
- Assina `/handler` (`Conversation`), `/tag_list` (`TaginfoArray`, os arucos), `/yolo_info` (`YoloArray`, os objetos).
- Publica `/cmd_vel`, `/handler` (manda mensagens pro Handler) e `/poweron_yolo` (liga o YOLO).

### Fluxo dos estados
1. **`seguir`** — roda a ação `SegueLinha`. A cada ciclo verifica: (a) se vê um animal perto → vai pra `reportar`; (b) se vê um aruco de bifurcação perto e já passou o cooldown → vai pra `perguntar`.
2. **`perguntar`** — manda uma vez `Robo: Bifurcação: <id>` pro Handler e vai esperar.
3. **`aguardar_resposta`** — parado, espera a resposta. Ao receber `direita`/`esquerda`, define `seguelinha.lado` (que enviesa a máscara pra aquele ramo), marca o ponto da decisão e volta a `seguir`.
4. **`reportar`** — manda uma vez `Robo: Objeto: <nome>` pro Handler.
5. **`aguardar_retorno`** — espera o `Retorne ao ponto de partida!`.
6. **`girar_180`** — usa a ação `Girar` pra dar meia-volta.
7. **`retornar`** — segue a linha de volta; para se chegar perto do ponto de partida **ou** se o laser detectar a placa na frente (não bate).
8. **`stop` / `done`** — para de vez (estado eterno).

### Funções-chave
- `aruco_bifurcacao()` — devolve o id do aruco de bifurcação (100/150/200/250) que estiver mais perto que `dist_aruco`, senão `None`. É isso que decide "cheguei numa bifurcação".
- `animal_perto()` — entre os animais detectados pelo YOLO com box grande (perto) e confiança ≥ `score_min`, devolve o de **maior score** (resolve confusão cachorro/cavalo). Mapeia `cat→gato, dog→cachorro, horse→cavalo`.
- `liga_yolo()` — publica `True` em `/poweron_yolo` algumas vezes no início (garante que o YOLO liga mesmo se subiu depois).
- `fala(texto)` — publica `Robo: <texto>` no `/handler` mantendo o histórico.

### Ajustes rápidos (topo do `__init__`)
- `dist_aruco` (1.2) — distância pra considerar que está na bifurcação.
- `cooldown_dist` (0.8) — quanto anda após uma decisão antes de aceitar outro aruco (evita re-perguntar o mesmo).
- `dist_bias` (0.7) — por quanto tempo mantém o viés de lado depois da bifurcação.
- `area_objeto` (90) — largura mínima do box pra considerar o animal "perto". Aumente se ele errar o animal (decide só mais perto).
- `score_min` (0.40) — confiança mínima do YOLO.
- `dist_obstaculo` (0.35) — distância pra parar na volta sem bater na placa.

---

## seguelinha.py — ação de seguir a linha (usada pela q1)

**Objetivo:** detectar a linha do mapa e gerar o `cmd_vel` pra segui-la.

**Detalhe importante:** a linha do mapa `divisoes` **não é amarela** — é um degradê azul/vermelho sobre chão de madeira. Como o chão tem cor "fraca" e a linha tem cor "forte", a máscara pega **qualquer pixel com saturação e brilho altos** (`inRange` com matiz inteiro e `S_min`/`V_min` calibrados). Isso isola a linha independente da cor.

### Como funciona
- Assina a câmera (`/camera/image_raw/compressed`).
- Recorta a **metade de baixo** da imagem (o chão à frente das rodas), converte pra HSV e aplica a máscara.
- Calcula o centro da linha (`cx`) pelo momento da máscara, vira em erro normalizado e gera o giro com controle P: `w = kp * erro`.
- **Viés de lado** (`self.lado`): quando é `'direita'`, apaga a metade esquerda da máscara (segue só o ramo da direita); `'esquerda'` faz o contrário. É assim que o robô "escolhe" o caminho na bifurcação.

### Ajustes rápidos
- `s_min` (113), `v_min` (138) — limiares da máscara (valores calibrados com o tuner).
- `kp` (1.0) — quanto maior, mais ele corrige (alto demais = zigue-zague).
- `v_linear` (0.08) — velocidade pra frente (menor = mais estável).

---

## girar.py — ação de girar (usada pela q1 na volta)

**Objetivo:** girar uma quantidade em radianos usando odometria.

Herda `Node, Odom`. Recebe a rotação em `reset(rotacao=...)` (ex.: `np.pi` = 180°), calcula o yaw-alvo e gira com controle P no yaw até ficar a menos de 5° do alvo, então para. A q1 usa pra dar meia-volta antes de retornar.

---

## q2.py — Exercício 2 (MedirCaixa)

**Objetivo:** achar a caixa, contorná-la sem bater, medir largura e comprimento, e voltar ao ponto de partida.

**Herança:** `Node, Odom`. Assina o laser direto (`/scan`) pra ler `angle_min`/`angle_increment` e projetar os pontos corretamente.

### A ideia da medição
O robô **orbita** a caixa mantendo-a à esquerda a uma distância fixa e, enquanto anda, **registra no referencial da odometria os pontos onde o laser bate** (no mundo vazio, tudo que o laser vê é a caixa). A medida é a extensão desses pontos:
- largura = extensão em X, comprimento = extensão em Y.

**Por que não usa coordenadas do mundo:** a odometria começa em zero **onde o robô nasce**, não no centro do mundo. Por isso a medição é feita como uma *extensão relativa* dos pontos (não importa onde está a origem da odometria) — e por isso o robô orbita a caixa de fato, achando ela pelo laser, em vez de dar voltas numa coordenada fixa.

**Precisão (percentis):** em vez de `max − min` (que um único feixe perdido estraga), a largura/comprimento usam os **percentis 1% e 99%** dos pontos. Assim outliers do laser não contam, e a medida fica grudada no valor real.

### Fluxo dos estados
1. **`aproximar`** — gira até apontar pro ponto mais próximo (a caixa), sempre andando pra frente, até ficar a `D` dela. Se não vê nada, gira procurando e loga um aviso de diagnóstico.
2. **`orbitar`** — controle que mantém a caixa a ~90° (esquerda) e à distância `D`: `angular = kb*(beta − 90°) + kd*(rho − D)`, onde `beta` é o ângulo do ponto mais próximo e `rho` a distância. Sempre anda pra frente (não gira parado). Acumula os pontos do laser. Termina quando dá a volta (afasta do início e volta).
3. **`retornar`** — volta pro ponto de partida por odometria.
4. **`stop` / `done`** — para de vez.

### Funções-chave
- `feixes()` — gera `(ângulo_no_robô, distância)` dos feixes válidos (finitos e abaixo de 3.4 m).
- `ponto_mais_proximo()` — ângulo e distância do feixe mais perto = a caixa.
- `coleta()` — projeta cada feixe no referencial da odometria e guarda os pontos.
- `reporta()` — calcula largura/comprimento pelos percentis e loga.

### Ajustes rápidos
- `D` (0.7) — distância que mantém da caixa. Aumente se bater nela.
- `v` (0.12) — velocidade ao orbitar.
- `kb` (0.8) — ganho pra manter a caixa a 90°.
- `kd` (1.5) — ganho pra manter a distância (aumente se "descolar" da caixa).

---

## tuner_mascara.py — ferramenta de calibração (opcional)

**Objetivo:** achar os valores `S_min`/`V_min` da máscara da linha sem mover o robô.

Abre janelas com sliders e mostra a máscara ao vivo da câmera. Você mexe nos sliders até a linha aparecer branca e limpa, anota os valores que o terminal imprime, e copia pro `seguelinha.py`. Não controla o robô — é só pra calibrar. Útil se a iluminação/linha mudar no dia da prova.

---

## setup.py — registro dos executáveis

Define os `console_scripts` que viram os comandos `ros2 run avaliacao_af <nome>`:
- `q1` → `avaliacao_af.q1:main`
- `q2` → `avaliacao_af.q2:main`
- `tuner` → `avaliacao_af.tuner_mascara:main`

Toda vez que adiciona/renomeia um arquivo executável, atualiza aqui e recompila (`colcon build --packages-select avaliacao_af`).

---

## Como cada peça se encaixa

```
q1 (cliente)
 ├── compõe SegueLinha  → segue a linha / viés de lado na bifurcação
 ├── compõe Girar       → meia-volta no retorno
 ├── lê /tag_list       → decide bifurcação (aruco)
 ├── lê /yolo_info      → identifica o animal
 ├── fala /handler      → pergunta direção / reporta objeto
 └── usa Laser/Odom     → cooldown, viés, retorno sem bater

q2 (nó único)
 ├── lê /scan           → acha e orbita a caixa
 ├── usa Odom           → projeta os pontos e volta ao início
 └── percentis          → mede largura/comprimento robusto a outliers

tuner → ferramenta à parte pra calibrar a máscara do seguelinha
setup.py → registra q1, q2, tuner como comandos do ros2 run
```
