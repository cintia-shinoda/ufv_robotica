import time
import os
import math
import numpy as np
import matplotlib.pyplot as plt
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from matplotlib.animation import FuncAnimation

# -------------------------------------------------------------------------------------------------------------------------------------
matricula = input("Digite sua Matrícula: ")
print()

# Conecta ao CoppeliaSim
for attempt in range(20):
    try:
        client = RemoteAPIClient()
        sim = client.getObject('sim')
        print(f"[SUCCESS] Conectado ao CoppeliaSim")
        break
    except Exception:
        print(f"[WAITING] Tentando conectar... ({attempt + 1}/20)")
        time.sleep(1)
else:
    raise TimeoutError("Não foi possível conectar ao CoppeliaSim.")

# Encontra o Manipulador referente a matrícula
if int(matricula) % 3 == 0:
    scene = "Scara.ttt"
    manipulador = "SCARA"
elif int(matricula) % 3 == 1:
    scene = "Cilindrico.ttt"
    manipulador = "CILÍNDRICO"
else:
    scene = "Esferico.ttt"
    manipulador = "ESFÉRICO"

# Carrega a cena com o manipulador
scene_path = os.path.abspath(scene)
result = sim.loadScene(scene_path)
if result == 0:
    raise RuntimeError("Erro ao carregar a cena.")
print("[SUCCESS] Manipulador %s carregado com sucesso." %manipulador)

# Converte os dígitos da matrícula
matricula = matricula.zfill(6)
R, S, T, X, Y, Z = map(int, matricula)

# Define a estratégia
if Z % 2 == 0:
    estrategia = "bang-bang"
else:
    estrategia = "polinomial"

# Ajusta os valores dos pontos iniciais e finais da reta
partida = [-0.5, -0.5, 0.5]
chegada = [0.5, 0.5, 1.5]

ajuste_A = [0.25 if d % 2 == 0 else -0.25 for d in [R, S, T]]
ajuste_B = [0.25 if d % 2 == 0 else -0.25 for d in [X, Y, Z]]

ponto_A = [p + a for p, a in zip(partida, ajuste_A)]
ponto_B = [p + b for p, b in zip(chegada, ajuste_B)]

# Identificação do Tarefa
print('\nArea de trabalho = 0  |  Cinemática Direta = 1  |  Cinemática Inversa = 2')
tarefa = int(input("Digite o número referente à tarefa: "))
while tarefa not in (0,1,2):
    print("Este número não corresponde a uma tarefa!")
    tarefa = int(input("Digite o número referente à tarefa: "))

# Inicia a simulação
sim.startSimulation()
print("\n[INFO] Simulação iniciada.")

# Fazendo a varredura dos eixos
axes = {}
axes['axis1'] = sim.getObject('/Manipulador/axis1')  # Rotação
axes['axis2'] = sim.getObject('/Manipulador/axis2')  # Rotação ou Translação
axes['axis3'] = sim.getObject('/Manipulador/axis3')  # Rotação ou Translação
axes['tcp']   = sim.getObject('/Manipulador/axis4')  # Efetuador
print('[FOUND] Todos os eixos do manipulador %s foram encontrados' %manipulador)

# -------------------------------------------------------------------------------------------------------------------------------------
# Execução da tarefa proposta
if tarefa == 0: # Area de Trabalho
    print('[INFO] Iniciando varredura da área de trabalho...')

    # Utilize o comando np.radians(np.linspace(MenorValor, MaiorValor, passo)) para ângulos
    # Utilize o comando np.linspace(MenorValor, MaiorValor, passo) para distâncias

    
    #             points.append(position) # Dentro do for
    # points = np.array(points)

    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # ax.scatter(points[:, 0], points[:, 1], points[:, 2], s=1, alpha=0.5, c='blue')
    # ax.set_xlabel('X (m)')
    # ax.set_ylabel('Y (m)')
    # ax.set_zlabel('Z (m)')
    # ax.set_title('Área de Trabalho - Leitura Real do TCP')
    # plt.tight_layout()
    # plt.show()

elif tarefa == 1: # Cinemática direta
    # Armazenar as posições
    pos_x, pos_y, pos_z = []
    def registrar_posicao():
        pose = sim.getObjectPose(axes['tcp'], -1)
        pos_x.append(pose[0])
        pos_y.append(pose[1])
        pos_z.append(pose[2])
    
    # 1. Posição inicial
    sim.setJointTargetPosition(axes['axis1'], 0)
    sim.setJointTargetPosition(axes['axis2'], 0)
    sim.setJointTargetPosition(axes['axis3'], 0)
    print("Todos os eixos na posição inicial")
    time.sleep(1)
    registrar_posicao()

    # 2. Movimento 1
    # time.sleep(1)
    # registrar_posicao()

    # 3. Movimento 2
    # time.sleep(1)
    # registrar_posicao()

    # 4. Movimento 3
    # time.sleep(1)
    # registrar_posicao()

    # 5. Posição inicial
    sim.setJointTargetPosition(axes['axis1'], 0)
    sim.setJointTargetPosition(axes['axis2'], 0)
    sim.setJointTargetPosition(axes['axis3'], 0)
    print("Todos os eixos retornaram para a posição inicial")
    time.sleep(1)
    registrar_posicao()

    # Exibir gráfico 3D com os pontos visitados
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(pos_x, pos_y, pos_z, 'k--', label='Trajetória')
    ax.scatter(pos_x, pos_y, pos_z, c='r', s=60, label='Posições')

    for i, (x, y, z) in enumerate(zip(pos_x, pos_y, pos_z)):
        ax.text(x, y, z, f'{i+1}', fontsize=10, color='blue')

    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    ax.set_title('Posições visitadas pelo TCP')
    ax.legend()
    plt.tight_layout()
    plt.show()

else: # Cinemática Inversa
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X [m]', fontsize=12, fontweight='bold')
    ax.set_ylabel('Y [m]', fontsize=12, fontweight='bold')
    ax.set_zlabel('Z [m]', fontsize=12, fontweight='bold')
    ax.set_xlim([-1.5, 1.5])
    ax.set_ylim([-1.5, 1.5])
    ax.set_zlim([0, 1.5])
    ax.grid(True)

    trajectory_line, = ax.plot([], [], [], 'k-', linewidth=2)
    visited_x, visited_y, visited_z = [], [], []
    current_pos_marker, = ax.plot([], [], [], 'r--', linewidth=2)
    
    # Parâmetros da trajetória
    tf = R + S + T + X + Y + Z
    x0, y0, z0 = ponto_A
    xf, yf, zf = ponto_B
    
    # Mover para posição inicial
    print('[INFO] Levadando o manipulador para a posição inicial: ', ponto_A)
    




    print('[INFO] Estratégia %s configurada para um tempo total de %d segundos.' %(estrategia, tf))
    print('[INFO] Iniciando reta no espaço até a posição final: ', ponto_B)
    # Preparar animação
    
            
        #     # Obter posição atual
        #     pose = sim.getObjectPose(axes['tcp'], -1)
        #     current_pos = pose[:3]
            
        #     # Atualizar plot
        #     trajectory_line.set_data([x0, x], [y0, y])
        #     trajectory_line.set_3d_properties([z0, z])

        #     visited_x.append(current_pos[0])
        #     visited_y.append(current_pos[1])
        #     visited_z.append(current_pos[2])

        #     current_pos_marker.set_data(visited_x, visited_y)
        #     current_pos_marker.set_3d_properties(visited_z)

            
        # except Exception as e:
        #     print(f"Erro: {e}")
        
        # return trajectory_line, current_pos_marker
    
    # Criar animação
    # ani = FuncAnimation(fig, update, frames=np.linspace(0, tf, 10*tf),
    #                     interval=50, blit=False, repeat=False)
    
    plt.tight_layout()
    plt.show()

# Encerra a simulação
sim.stopSimulation()
print("[INFO] Simulação finalizada.")
