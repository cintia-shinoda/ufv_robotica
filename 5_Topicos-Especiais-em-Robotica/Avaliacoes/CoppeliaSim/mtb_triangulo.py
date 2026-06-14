"""
mtb_triangulo.py
================
CoppeliaSim + Python (ZMQ Remote API)
Manipulador MTB (SCARA: 2 juntas rotativas + 1 prismatica) leva o
efetuador a 3 posicoes que formam um TRIANGULO EQUILATERO no espaco.

Movimento:
  - Posiciona as juntas com setJointPosition 
  - NAO altera o modo da junta

[Inferencia] axis1/axis2 = rotativas (definem x,y); axis3 = prismatica
(z, mantida fixa); axis4 = TCP. O script mede a geometria real.

COMO USAR:
  1. Abra "MTB.ttt" no CoppeliaSim.
  2. Garanta o add-on "ZMQ remote API" ativo.
  3. Rode (com venv ativo):  python mtb_triangulo.py
"""

import math
import time
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

FRAC_CENTRO = 0.55
FRAC_RAIO = 0.22
ANG = np.deg2rad([90, 210, 330])
TOL_JUNTA = math.radians(1.5)
T_SETTLE = 5.0


def getpos(sim, h):
    p = sim.getObjectPosition(h, -1)
    return np.array(p, dtype=float)


def getjoint(sim, h):
    return float(sim.getJointPosition(h))


def angdiff(a, b):
    return math.atan2(math.sin(a-b), math.cos(a-b))


def diag_joint(sim, j, name):
    """Imprime o modo da junta para diagnostico."""
    parts = []
    try:
        parts.append(f"jointMode={sim.getJointMode(j)}")
    except Exception:
        parts.append("jointMode=?")
    try:
        parts.append(f"dynctrl={sim.getObjectInt32Param(j, sim.jointintparam_dynctrlmode)}")
    except Exception:
        parts.append("dynctrl=?")
    print(f"  [diag] {name}: " + " | ".join(str(p) for p in parts))


def enable_motor(sim, j):
    """Coloca a junta em controle de posicao (API 4.x), com fallbacks."""
    # 4.x: modo de controle dinamico = posicao
    try:
        sim.setObjectInt32Param(j, sim.jointintparam_dynctrlmode, sim.jointdynctrl_position)
    except Exception:
        pass
    # legado: motor + malha de controle
    try:
        sim.setObjectInt32Param(j, sim.jointintparam_motor_enabled, 1)
        sim.setObjectInt32Param(j, sim.jointintparam_ctrl_enabled, 1)
    except Exception:
        pass


def move_joints(sim, a1, a2, a3, q1, q2, q3):
    # Modo cinematico: setJointPosition posiciona a junta diretamente
    sim.setJointPosition(a1, q1)
    sim.setJointPosition(a2, q2)
    sim.setJointPosition(a3, q3)


def settle(sim, a1, a2, qd, tmax=T_SETTLE):
    t = time.time()
    while time.time() - t < tmax:
        e1 = abs(angdiff(getjoint(sim, a1), qd[0]))
        e2 = abs(angdiff(getjoint(sim, a2), qd[1]))
        if e1 < TOL_JUNTA and e2 < TOL_JUNTA:
            break
        time.sleep(0.05)
    time.sleep(0.3)


def force_if_stuck(sim, j, val, prismatic=False):
    """Se a junta nao chegou no alvo (junta cinematica), forca a posicao."""
    cur = getjoint(sim, j)
    err = abs(cur - val) if prismatic else abs(angdiff(cur, val))
    lim = 0.01 if prismatic else math.radians(2)
    if err > lim:
        try:
            sim.setJointPosition(j, val)
        except Exception as e:
            print(f"   [aviso] setJointPosition falhou: {e}")


def ir_para(sim, a1, a2, a3, q1, q2, q3):
    """Posiciona as juntas (modo cinematico) e aguarda atualizacao do TCP."""
    move_joints(sim, a1, a2, a3, q1, q2, q3)
    time.sleep(0.5)   # tempo para o TCP atualizar/renderizar


def ik_scara(x, y, L1, L2):
    r2 = x*x + y*y
    c2 = (r2 - L1*L1 - L2*L2) / (2*L1*L2)
    ok = abs(c2) <= 1.0
    c2 = max(min(c2, 1.0), -1.0)
    q2 = math.atan2(math.sqrt(1 - c2*c2), c2)
    q1 = math.atan2(y, x) - math.atan2(L2*math.sin(q2), L1 + L2*math.cos(q2))
    return q1, q2, ok


def main():
    print("Conectando ao CoppeliaSim...")
    client = RemoteAPIClient(host='127.0.0.1')
    sim = client.require('sim')

    sim.startSimulation()
    time.sleep(0.5)

    axis1 = sim.getObject('/MTB/axis1')
    axis2 = sim.getObject('/MTB/axis2')
    axis3 = sim.getObject('/MTB/axis3')
    tcp = sim.getObject('/MTB/axis4')
    print("[OK] Eixos do MTB obtidos.")

    # Juntas em modo CINEMATICO: usamos setJointPosition direto.
    print("Modo das juntas:")
    for j, n in ((axis1, 'axis1'), (axis2, 'axis2'), (axis3, 'axis3')):
        diag_joint(sim, j, n)

    # --- 1. Medir geometria (L1, L2) ---
    p1 = getpos(sim, axis1)
    p2 = getpos(sim, axis2)
    p4 = getpos(sim, tcp)
    base = p1[:2]
    L1 = math.hypot(p2[0]-p1[0], p2[1]-p1[1])
    L2 = math.hypot(p4[0]-p2[0], p4[1]-p2[1])
    q3_hold = getjoint(sim, axis3)
    print(f"Geometria medida: L1={L1:.3f} m, L2={L2:.3f} m, "
          f"base=({base[0]:.2f},{base[1]:.2f})")

    # --- 2. Calibracao: direcao de referencia (q1=q2=0) ---
    ir_para(sim, axis1, axis2, axis3, 0.0, 0.0, q3_hold)
    p_cal = getpos(sim, tcp)
    theta_ref = math.atan2(p_cal[1]-base[1], p_cal[0]-base[0])
    print(f"Calibracao: theta_ref={math.degrees(theta_ref):.1f} deg, "
          f"z do plano={p_cal[2]:.3f} m")

    # Sanity check: o braco se moveu na calibracao?

    # --- 3. Triangulo no frame da base ---
    reach = L1 + L2
    r_c = FRAC_CENTRO * reach
    R_tri = FRAC_RAIO * reach
    Vb = np.array([[r_c + R_tri*math.cos(a), R_tri*math.sin(a)] for a in ANG])
    rmin, rmax = abs(L1-L2) + 0.02, reach - 0.02
    for _ in range(8):
        rr = np.hypot(Vb[:, 0], Vb[:, 1])
        if np.all((rr > rmin) & (rr < rmax)):
            break
        R_tri *= 0.85
        r_c *= 0.9
        Vb = np.array([[r_c + R_tri*math.cos(a), R_tri*math.sin(a)] for a in ANG])
    print(f"Triangulo: r_centro={r_c:.3f} m, R={R_tri:.3f} m")

    # --- 4. Visitar cada vertice ---
    tcp_real = np.full((3, 3), np.nan)
    for k in range(3):
        q1, q2, ok = ik_scara(Vb[k, 0], Vb[k, 1], L1, L2)
        if not ok:
            print(f"[!] Vertice {k} fora do alcance.")
        print(f"\n-> Vertice {k}: q1={math.degrees(q1):.1f} deg, "
              f"q2={math.degrees(q2):.1f} deg")
        ir_para(sim, axis1, axis2, axis3, q1, q2, q3_hold)
        tcp_real[k] = getpos(sim, tcp)
        print(f"   TCP real: ({tcp_real[k,0]:.3f}, {tcp_real[k,1]:.3f}, {tcp_real[k,2]:.3f})")

    sim.stopSimulation()
    print("\nSimulacao finalizada.")

    verificar(tcp_real, Vb, base, theta_ref)
    plotar(tcp_real, Vb, base, theta_ref)


def verificar(tcp_real, Vb, base, theta_ref):
    print("\n========= VERIFICACAO DO TRIANGULO (MTB) =========")
    Rrot = np.array([[math.cos(theta_ref), -math.sin(theta_ref)],
                     [math.sin(theta_ref),  math.cos(theta_ref)]])
    alvo_w = (base.reshape(2, 1) + Rrot @ Vb.T).T

    def d(a, b):
        return math.hypot(a[0]-b[0], a[1]-b[1])

    L = [d(tcp_real[0], tcp_real[1]),
         d(tcp_real[1], tcp_real[2]),
         d(tcp_real[2], tcp_real[0])]
    Lmed = sum(L)/3

    # Guarda: efetuador nao se moveu?
    if Lmed < 1e-6:
        print("  [!] O efetuador NAO se moveu entre os vertices.")
        print("      TCP identico nos 3 pontos -> as juntas nao responderam.")
        print("      Acao: confira no CoppeliaSim se axis1/axis2 estao em modo")
        print("      'Torque/force' com motor + control loop habilitados, ou se")
        print("      sao juntas cinematicas. Me mande um print das propriedades.")
        print("==================================================")
        return

    desvio = max(abs(x - Lmed) for x in L)/Lmed*100
    for k in range(3):
        err = math.hypot(tcp_real[k, 0]-alvo_w[k, 0], tcp_real[k, 1]-alvo_w[k, 1])
        st = "[OK]" if err <= 0.03 else "[!]"
        print(f"  {st} Vertice {k}: alvo ({alvo_w[k,0]:.3f},{alvo_w[k,1]:.3f}) | "
              f"real ({tcp_real[k,0]:.3f},{tcp_real[k,1]:.3f}) | erro {err:.3f} m")
    print(f"  Lados: {L[0]:.3f}, {L[1]:.3f}, {L[2]:.3f} m  (media {Lmed:.3f} m)")
    print(f"  Desvio maximo entre lados: {desvio:.1f}%")
    if desvio < 8:
        print("  RESULTADO: TRIANGULO EQUILATERO COMPLETADO COM SUCESSO")
    else:
        print("  RESULTADO: triangulo com desvios (ver acima)")
    print("==================================================")


def plotar(tcp_real, Vb, base, theta_ref):
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("\n(matplotlib nao instalado - grafico pulado.)")
        return
    if np.any(np.isnan(tcp_real)):
        return
    # se nao se moveu, nao plota triangulo degenerado
    if math.hypot(tcp_real[0,0]-tcp_real[1,0], tcp_real[0,1]-tcp_real[1,1]) < 1e-6:
        return
    Rrot = np.array([[math.cos(theta_ref), -math.sin(theta_ref)],
                     [math.sin(theta_ref),  math.cos(theta_ref)]])
    alvo_w = (base.reshape(2, 1) + Rrot @ Vb.T).T
    plt.figure("MTB - Triangulo")
    tx = list(tcp_real[:, 0]) + [tcp_real[0, 0]]
    ty = list(tcp_real[:, 1]) + [tcp_real[0, 1]]
    plt.plot(tx, ty, 'b-o', linewidth=1.5, markerfacecolor='b', label='TCP real')
    ax = list(alvo_w[:, 0]) + [alvo_w[0, 0]]
    ay = list(alvo_w[:, 1]) + [alvo_w[0, 1]]
    plt.plot(ax, ay, 'r--s', linewidth=1.2, markersize=10, label='Triangulo alvo')
    plt.plot(base[0], base[1], 'k^', markersize=12, label='Base')
    for k in range(3):
        plt.text(tcp_real[k, 0]+0.02, tcp_real[k, 1], f'V{k}')
    plt.axis('equal')
    plt.grid(True)
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.title('TCP do MTB: triangulo real vs desejado (vista superior)')
    plt.legend()
    plt.show()


if __name__ == '__main__':
    main()