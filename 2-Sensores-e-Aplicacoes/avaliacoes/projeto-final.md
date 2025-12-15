# Projeto Final: Sensores e Aplicações

## 1.
A paletização é um processo amplamente utilizado na logística para armazenar, organizar e proteger mercadorias. Consiste em agrupar volumes menores de produtos sobre um palete, otimizando o uso do espaço, facilitando a movimentação, reduzindo custos, agilizando processos e protegendo as cargas contra danos.

Na paletização automática, esse processo é realizado por sistemas que utilizam tecnologia avançada, como robôs industriais e máquinas paletizadoras, capazes de empilhar e organizar produtos em paletes de forma autônoma, sem intervenção manual direta. O principal objetivo é substituir o trabalho humano em tarefas repetitivas e fisicamente extenuantes, aumentando a eficiência, a precisão e a segurança das operações logísticas.

Como exemplo, pode ser citada a Célula Robotizada de Paletização FlexPall R180, da empresa DALCA Brasil

https://youtu.be/igJ5B2DYfk4

Uma empresa cerealista está avaliando a possibilidade de implementar uma paletizadora automática e, de acordo com um fornecedor, o seu equipamento utiliza um sensor de força acoplado à extremidade do braço robótico, com as especificações a seguir.

Especificação
| Característica | Unidade | Valor (ou Faixa) |
|:---:|:---:|:---:|
| Faixa nominal | kgf | 0,0 --- 300 |
| Sensibilidade | % | 0,1 |
| Acurácia | % | 0,2 |

 


Sabe-se que essa linha de produtos opera com:

- paletes contendo 80 unidades de pó de café de 250 g, e
- paletes contendo 20 unidades de arroz de 5 kg.
Com base nas informações fornecidas e nos dados do “datasheet” apresentados na tabela, responda:

### a. 
O sensor proposto pode ser utilizado nessa aplicação, considerando a faixa de valores de força a serem medidos? Justifique sua resposta.

Sim, pode ser usado.
Os paletes exigem medições entre 20 e 100 kgf, que estão confortavelmente dentro da faixa 0–300 kgf. Não há risco de saturação nem de operação no limite do sensor.

Palete de pó de café
80 unidades × 250 g = 20 kg
≈ 20 kgf

Palete de arroz
20 unidades × 5 kg = 100 kg
≈ 100 kgf

Faixa do sensor: 0 a 300 kgf

---

### b.
Sabendo que a principal causa de não conformidade do palete é a falta de uma única unidade de produto, seria possível, analisando o valor medido pelo sensor, detectar se os paletes de pó de café e de arroz estão em conformidade (com todas as unidades) ou não? Explique.

— Palete de café: não dá para detectar falta de 1 unidade. A variação (0,25 kgf) é menor que a sensibilidade do sensor (0,3 kgf) e menor que o erro (0,6 kgf). A medição fica totalmente encoberta pelo ruído do instrumento.

— Palete de arroz: dá para detectar.


- Sensibilidade: 0,1% da faixa
0,1% de 300 kgf = 0,3 kgf (resolução mínima útil)
- Acurácia (erro máximo): 0,2% da faixa
0,2% de 300 kgf = 0,6 kgf (erro total)

Variação causada por faltar UMA unidade:

Pó de café (250 g → 0,25 kgf)
A ausência altera o total em 0,25 kgf

Arroz (5 kg)
A ausência altera o total em 5 kgf

| Produto | Variação real se faltar 1 unidade | Sensibilidade (0,3 kgf) | Erro máx (0,6 kgf) | Detectável? |
|:---:|:---:|:---:|:---:|:---:|
| Café | 0,25 kgf | Maior | Maior | Não |
| Arroz | 5 kgf | Maior | Maior | Sim |



---

### c.
O sensor atende ao critério de conformidade metrológica, isto é, o sistema de medição não afeta significativamente o processo e o experimento não afeta o sensor (não o sobrecarrega, não o satura e não altera suas características de medição)? Justifique.

Sim, atende.

1.	Não há sobrecarga:
A carga máxima esperada (100 kgf) é apenas um terço da faixa nominal (300 kgf). O sensor trabalha longe da saturação.

2.	O processo não afeta o sensor:
Carga constantemente abaixo de 300 kgf → não há risco de deformação permanente.

3.	O sensor não afeta o processo:
A força medida é apenas o peso da carga; o sensor está na extremidade do braço e não interfere no empilhamento ou na estabilidade da carga.

4.	Não opera em zona de “ruído de fundo”:
As cargas estáticas (20–100 kgf) são muito superiores à sensibilidade (0,3 kgf), garantindo boa repetibilidade no contexto industrial.

Conclusão:
Metrologicamente adequado, desde que não seja usado para detectar a falta de unidades pequenas (como o café).


---
---

## 2.
Outra possível não conformidade no processo de paletização é a formação inadequada do palete, o que pode comprometer a estabilidade, a segurança e a qualidade das operações. Para evitar esse problema, é necessário utilizar um sensor capaz de verificar o volume, o formato e o posicionamento das caixas durante a montagem do palete.

Uma solução viável consiste em acoplar um sensor de varredura 3D na extremidade da paletizadora, como ilustrado na figura abaixo, permitindo que o robô inspecione cada camada ou o palete completo.

paletizadora



Com base nos conceitos estudados na disciplina, responda:

### a.
Qual é o tipo de sensor mais indicado para essa aplicação? Justifique sua resposta considerando a necessidade de medir volume, forma e possíveis desalinhamentos no palete.

Um sensor 3D por triangulação/varredura (laser line profiler ou structured-light) ou uma câmera 3D ToF / RGB-D de alta resolução montada no fim de braço (End-Of-Arm) é a melhor opção.

Justificativa:
	•	Dados 3D por pixel (nuvem de pontos) permitem medir volume, estimar poses e detectar desalinhamentos camada a camada.
	•	Triangulação (laser line / structured light) oferece alta resolução e acurácia em curtas/medianas distâncias — ideal para inspeção de camadas e caixas.
	•	Câmeras ToF / RGB-D (mais simples) são rápidas e robustas em ambientes industriais, mas têm menor resolução/precisão que triangulação.
	•	Lidar 3D costuma ser overkill (custo, resolução angular) para paletes onde você precisa de detalhe de caixas e cantos.
	•	Importante: escolha um sensor com campo de visão (FOV) e alcance compatíveis com a geometria da paletizadora e com taxa suficiente para inspecionar durante o ciclo do robô.

    Especificações práticas a considerar (valores indicativos — [Inferência]):
	•	resolução Z de ~0.5–5 mm para detectar desalinhamentos e empilhamentos incorretos;
	•	taxa de aquisição ≥ 10–30 Hz para manter cadência de produção;
	•	campo de visão cobrindo a área de palete a uma distância de trabalho típica de 0.5–1.5 m;
	•	imunidade razoável à poeira/variações de iluminação ou proteção/encaixe para ambiente industrial.

---

### b.
Qual o nível de desenvolvimento e processamento necessário para que o sistema seja capaz de identificar automaticamente a má formação do palete? Explique de forma clara como o sensor, o processamento de dados e os algoritmos estariam envolvidos na detecção de não conformidades.

é preciso pipeline de visão 3D em tempo real com etapas clássicas: captura → pré-processamento → segmentação/registro → análise geométrica → decisão (regras/ML). Requer software (PCL/ROS/OpenCV), cumprimento de sazonalidade industrial e, dependendo do débito, GPU/CPU robusta.

Fluxo técnico recomendado (passo a passo):
	1.	Calibração e sincronização
	•	Calibração extrínseca entre sensor e flange do robô; transformar nuvem para referencial do palete.
	•	Compensar distorções e retirar ruído sistemático.
	2.	Aquisição e pré-processamento
	•	Filtragem de ruído (statistical outlier), downsampling (voxel grid) e remoção de plano do chão/pallet se necessário.
	•	Remoção de reflexos/artefatos e máscara por ROI (região de interesse).
	3.	Registro e montagem de camadas
	•	Se fizer varredura camada a camada: registrar (coletar) as varreduras com odometria do robô; usar transformações conhecidas (não necessariamente ICP se tiver pose precisa).
	•	Para palete inteiro, compor várias vistas com ICP ou técnicas robustas.
	4.	Segmentação de caixas e extração de features
	•	Detecção de caixas via clustering (Euclidean), ajuste de bounding boxes 3D, detecção de planos e arestas, e medição de volumes (convex hull / voxel occupancy).
	•	Medidas extraídas: posição (x,y,z) de cada caixa, orientação (roll/pitch/yaw), ocupação volumétrica, sobreposição entre caixas, folgas entre caixas.
	5.	Regras de conformidade (detecção de não conformidade)
	•	Regras simples inicialmente:
	•	contagem de caixas por camada deve ser N ± tolerance;
	•	desvio de posição de cada caixa > limite (mm) → não conformidade;
	•	rotação/tilt da caixa > limite → não conformidade;
	•	volume total discrepante → falta/extra de unidade.
	•	Regras podem ser combinadas com thresholding estatístico (por ex. desvio > 3σ).
	6.	(Opcional) ML/Anomaly detection
	•	Treinar modelos (CNN 3D / PointNet / Random Forest sobre features) para classificar “bom” vs “mal formado” quando padrões forem complexos (ex.: caixas deformadas, embalagens amassadas).
	•	Útil onde regras determinísticas produzem muitos falsos positivos.
	7.	Feedback ao robô / ação
	•	Se não conformidade: sinal de reprovação → rejeitar palete, reposicionar caixas, ou acionar operador.
	•	Registrar logs e imagens/nuvens para auditoria.

Plataforma e requisitos computacionais (prático):
	•	PC industrial com CPU multicore + GPU (se usar redes neurais) ou só CPU se regras forem leves.
	•	Software: PCL, Open3D, ROS para integração, e biblioteca de visão 3D.
	•	Latência alvo: inspeção por camada em <1 s é razoável para linhas médias; para linhas rápidas buscar ≪1 s.
	•	Testes e tuning são essenciais: definir tolerâncias aceitáveis e população de dados para calibração.

---

### c.
Seria possível utilizar sensores instalados em uma estrutura fixa, em vez de acoplados ao robô? Explique como essa alternativa funcionaria e quais seriam suas vantagens e limitações.

Sim. Ambas as soluções (fixa e móvel) são viáveis; escolha depende de custo, espaço, taxa e risco de oclusão.

Sensor no fim de braço (End-of-Arm) — vantagens
	•	Visão adaptável: o robô leva o sensor para vantage points ideais; reduz oclusões.
	•	Inspeção por camada: consegue varrer cada camada de forma controlada e próxima (alta resolução).
	•	Menos sensores físicos (um único sensor pode cobrir várias vistas).

Desvantagens
	•	Integração mecânica e calibração exigem cuidado (vibração, cabos, peso no braço).
	•	Impacto na dinâmica do braço (inércia, ciclo de movimento).
	•	Sensor exposto a colisões/impactos.

Sensor(es) fixos — vantagens
	•	Maior robustez mecânica e menos impacto no movimento do robô.
	•	Possibilidade de usar múltiplos sensores fixos para cobertura multi-ângulo e reduzir oclusões sem mover o robô.
	•	Instalação e manutenção potencialmente mais simples (menor risco de choque).

Desvantagens
	•	Oclusões persistentes: áreas podem ficar escondidas se houver só uma ou duas vistas fixas; exige mais sensores posicionados estrategicamente (aumenta custo).
	•	Menos flexível para diferentes tamanhos de paletes/produtos — pode exigir ajuste de posição/ângulo ou múltiplos pontos fixos.
	•	Maior necessidade de calibração extrínseca entre sensores e área de trabalho.

Soluções híbridas (muito comuns):
	•	Um sensor fixo principal cobrindo vista geral + sensor end-of-arm para inspeção detalhada quando necessário.
	•	Isso reduz custo (menos varreduras com o robô) e mantém capacidade de resolver oclusões e pontos críticos com o sensor móvel.




	-------
	---
	a) 
- paletes de pó de café: 80 unidades x 0,25 kg = 20 kg

- paletes de arroz: 20 unidades x 5 kg = 100 kg

Considerando a faixa de valores de força a serem medidos (20 a 100 kgf), o sensor proposto pode ser utilizado nessa aplicação, uma vez que os valores estão dentro da faixa nominal: 0-300 kgf.





b)

Especificações do sensor:

- Sensibilidade: 0,1% * 300 kgf = 0,3 kgf (o sensor consegue detectar variações a partir de 300 g)

- Acurácia: 0,2% * 300 kgf = 0,6kgf (a leitura pode ter um erro de 600 g)



Paletes de pó de café:
O peso unitário (0,25 kgf) é menor do que a sensibilidade do sensor e significativamente menor do que a margem de erro da acurácia (0,6 kgf). 

Portanto não seria possível detectar se os paletes de pó de café estão em conformidade.



Paletes de arroz:

O peso do pacote de arroz (5 kgf) é maior que a sensibilidade do sensor (0,3 kgf) e maior que a margem de erro da acurácia (0,6 kgf).

Portanto, seria possível detectar se os paletes de arroz estão em conformidade com confiabilidade.





c)

Sim, o sensor atende ao critério de conformidade metrológica.

O experimento não afeta o sensor, pois a carga máxima aplicada ao sensor será 100 kgf e este valor está bem abaixo do fundo de escala (300 kgf), portanto não há riscos de saturação e de sobrecarga.

O sistema de medição não afeta o processo, pois não altera a massa dos produtos a serem pesados e não interfere na integridade da carga.




