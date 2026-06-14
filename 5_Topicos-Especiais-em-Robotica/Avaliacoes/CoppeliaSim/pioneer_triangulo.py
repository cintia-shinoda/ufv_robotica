"""
pioneer_triangulo.py
====================
CoppeliaSim + Python (ZMQ Remote API)
Pioneer 3-DX percorre um TRIANGULO EQUILATERO no plano x-y.

Baseado no script "pioneer.m" da disciplina (mesma conexao, mesmos
handles, mesmo controle por feedback linearization e conversao de rodas).
A tarefa: 3 vertices a 120 graus, avanco por chegada, verificacao final.

COMO USAR:
  1. Abra "Pioneer.ttt" (pasta Ambiente de Simulacao) no CoppeliaSim.
  2. Garanta o add-on "ZMQ remote API" ativo (padrao no 4.x).
  3. Ative o venv e instale as dependencias (ver instrucoes do chat).
  4. Rode:  python pioneer_triangulo.py
"""

import math
import time
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# ----------------------------------------------------------------------
# Parametros do triangulo
# ----------------------------------------------------------------------
CX, CY = 0.0, 0.0          # centro do triangulo [m]
R = 1.5                    # circunraio [m]
ANG = np.deg2rad([90, 210, 330])     # 3 vertices a 120 graus
WP = [(CX + R*math.cos(a), CY + R*math.sin(a)) for a in ANG]
WP.append(WP[0])           # fecha o triangulo (volta ao vertice 0)

TOL_CHEGADA = 0.12         # raio de chegada a um vertice [m]
T_POR_WP = 35.0            # tempo maximo por vertice [s]
TS = 0.1                   # periodo de controle [s] (10 Hz)
A_OFF = 0.15              # ponto de controle fora do eixo [m]
KD = np.diag([0.20, 0.30]) # ganhos do controlador

# Geometria do Pioneer (igual Pioneer3DX.py)
L_EIXO = 0.331             # distancia entre rodas [m]
R_RODA = 0.0975            # raio da roda [m]
MAX_W = 2.4               # saturacao de velocidade da roda [rad/s]


def yaw_from_quat(qx, qy, qz, qw):
    """Yaw (rotacao em z) a partir do quaternion. Valido p/ robo no plano."""
    return math.atan2(2.0*(qw*qz), 1.0 - 2.0*(qz*qz))


def main():
    print("Conectando ao CoppeliaSim...")
    client = RemoteAPIClient()
    sim = client.require('sim')

    sim.startSimulation()
    time.sleep(0.5)

    robot = sim.getObject('/PioneerP3DX')
    left = sim.getObject('/PioneerP3DX/leftMotor')
    right = sim.getObject('/PioneerP3DX/rightMotor')
    print(f"Simulacao iniciada. Triangulo: R={R:.2f} m, centro ({CX:.1f},{CY:.1f}).")

    # Log
    log = {'t': [], 'x': [], 'y': [], 'v': [], 'w': [], 'wp': []}
    atingidos = [None, None, None]

    t_total = time.time()

    for iwp, (xd, yd) in enumerate(WP):
        print(f"\n-> Indo ao vertice {iwp % 3}: ({xd:.2f}, {yd:.2f})")
        t_wp = time.time()
        chegou = False
        rho = float('inf')

        while time.time() - t_wp < T_POR_WP:
            t0 = time.time()

            # ----- Leitura da pose -----
            pose = sim.getObjectPose(robot, -1)   # [x y z qx qy qz qw]
            x, y = pose[0], pose[1]
            yaw = yaw_from_quat(pose[3], pose[4], pose[5], pose[6])

            # ----- Controle (feedback linearization) -----
            A = np.array([[math.cos(yaw), -A_OFF*math.sin(yaw)],
                          [math.sin(yaw),  A_OFF*math.cos(yaw)]])
            erro = np.array([xd - x, yd - y])
            Ud = np.linalg.pinv(A) @ (KD @ erro)   # Xd_dot = 0
            v, w = float(Ud[0]), float(Ud[1])

            # ----- Conversao para rodas (igual Pioneer3DX.py) -----
            vel_left = (v - w*L_EIXO/2) / R_RODA
            vel_right = (v + w*L_EIXO/2) / R_RODA
            vel_left = max(min(vel_left, MAX_W), -MAX_W)
            vel_right = max(min(vel_right, MAX_W), -MAX_W)
            sim.setJointTargetVelocity(left, vel_left)
            sim.setJointTargetVelocity(right, vel_right)

            # ----- Log -----
            log['t'].append(time.time() - t_total)
            log['x'].append(x)
            log['y'].append(y)
            log['v'].append(v)
            log['w'].append(w)
            log['wp'].append(iwp)

            # ----- Chegou? -----
            rho = math.hypot(xd - x, yd - y)
            if rho < TOL_CHEGADA:
                chegou = True
                if iwp < 3:
                    atingidos[iwp] = (x, y)
                print(f"   chegou (erro {rho:.3f} m, {time.time()-t_wp:.1f}s)")
                break

            # mantem ~10 Hz
            dt = time.time() - t0
            if dt < TS:
                time.sleep(TS - dt)

        if not chegou:
            print(f"   [!] tempo esgotado sem atingir (erro {rho:.3f} m)")
            if iwp < 3:
                atingidos[iwp] = (x, y)

    # Para o robo
    sim.setJointTargetVelocity(left, 0.0)
    sim.setJointTargetVelocity(right, 0.0)
    sim.stopSimulation()
    print("\nSimulacao finalizada.")

    verificar(atingidos)
    plotar(log, atingidos)


def verificar(atingidos):
    print("\n========= VERIFICACAO DO TRIANGULO (Pioneer) =========")
    alvo = WP[:3]

    def d(p, q):
        return math.hypot(p[0]-q[0], p[1]-q[1])

    err_v = [d(atingidos[k], alvo[k]) for k in range(3)]
    L = [d(atingidos[0], atingidos[1]),
         d(atingidos[1], atingidos[2]),
         d(atingidos[2], atingidos[0])]
    Lmed = sum(L)/3
    desvio = max(abs(x - Lmed) for x in L)/Lmed*100

    for k in range(3):
        st = "[OK]" if err_v[k] <= TOL_CHEGADA else "[!]"
        print(f"  {st} Vertice {k}: alvo ({alvo[k][0]:.2f},{alvo[k][1]:.2f}) | "
              f"real ({atingidos[k][0]:.2f},{atingidos[k][1]:.2f}) | erro {err_v[k]:.3f} m")
    print(f"  Lados: {L[0]:.3f}, {L[1]:.3f}, {L[2]:.3f} m  (media {Lmed:.3f} m)")
    print(f"  Desvio maximo entre lados: {desvio:.1f}%")
    if all(e <= TOL_CHEGADA for e in err_v) and desvio < 8:
        print("  RESULTADO: TRIANGULO EQUILATERO COMPLETADO COM SUCESSO")
    else:
        print("  RESULTADO: trajetoria com desvios (ver acima)")
    print("======================================================")


def plotar(log, atingidos):
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("\n(matplotlib nao instalado - grafico pulado. "
              "Instale com: pip install matplotlib)")
        return
    alvo = WP[:3]
    plt.figure("Pioneer - Triangulo")
    plt.plot(log['x'], log['y'], 'b-', linewidth=1.5, label='Trajetoria real')
    ax = [p[0] for p in alvo] + [alvo[0][0]]
    ay = [p[1] for p in alvo] + [alvo[0][1]]
    plt.plot(ax, ay, 'r--o', linewidth=1.2, markersize=8,
             markerfacecolor='r', label='Triangulo alvo')
    plt.plot([p[0] for p in atingidos], [p[1] for p in atingidos],
             'gs', markersize=12, label='Vertices atingidos')
    for k in range(3):
        plt.text(alvo[k][0]+0.1, alvo[k][1], f'V{k}')
    plt.axis('equal')
    plt.grid(True)
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.title('Trajetoria do Pioneer vs triangulo desejado')
    plt.legend()
    plt.show()


if __name__ == '__main__':
    main()
