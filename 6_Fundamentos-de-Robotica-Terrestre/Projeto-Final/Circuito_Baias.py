"""
============================================================================
Circuito_Baias.py  —  Home -> A -> B -> C -> D -> Home  (Pioneer 3DX / AuRoRA)
============================================================================
Controle HIBRIDO (maquina de estados) por baia:
  FACE  : gira no lugar para apontar ao proximo alvo   (orientacao ao SAIR)
  GOTO  : controle de POSICAO por linearizacao por realimentacao no ponto 'a'
          (mesma matematica de Pioneer3DX.sController / Controller.py)
  ALIGN : gira no lugar ate a orientacao de entrega ψd (orientacao ao CHEGAR)
  DWELL : pausa de entrega

A rota (via-points) foi planejada por GRAFO DE VISIBILIDADE sobre as bancadas
infladas por uma margem de seguranca -> garante navegacao SEM colisao mesmo
sem nenhuma estrategia reativa de evasao de obstaculos.

Coloque este arquivo na mesma pasta que Pioneer3DX.py e Pioneer.ttt.
============================================================================
"""
import time
import types
import numpy as np
import matplotlib.pyplot as plt
from Pioneer3DX import Pioneer3DX

# ============================ PARAMETROS DE CONTROLE ========================
Kp_pos = 0.7                 # ganho de posicao  (Kd na AuRoRA)
Kpsi   = 1.8                 # ganho de orientacao
U_MAX  = 0.50                # saturacao linear  [m/s]
W_MAX  = 1.50                # saturacao angular [rad/s]
EPS_VIA  = 0.22              # tolerancia de chegada em via-point [m]
EPS_DOCK = 0.10              # tolerancia de chegada em baia       [m]
EPS_ANG  = np.deg2rad(2.0)   # tolerancia de orientacao            [rad]
FACE_TH  = np.deg2rad(20)    # gira antes de andar se erro de rumo > isto
GATE_TH  = np.deg2rad(55)    # durante GOTO: zera u se erro de rumo > isto (anti-lunge)
DWELL    = 1.5               # pausa de entrega na baia [s]
WHEEL_CAP = 6.0              # teto de vel. das rodas [rad/s] (~0.585 m/s). Ver nota abaixo.

# ============================ ROTA PLANEJADA ================================
# (x, y, is_baia, psi_deg)  — psi_deg so e usado quando is_baia = True
# Docks:  A(-3,0,-90) B(-5,3,0) C(5,2,180) D(3,-5,90) Home(0,-5,90)
ROUTE = [
    (-1.10, -0.10, False,   0),   # via (corredor central-SO)
    (-3.00,  0.00, True,  -90),   # BAIA A
    (-4.90,  1.10, False,   0),   # via (contorna bancada B)
    (-5.00,  3.00, True,    0),   # BAIA B
    (-4.90,  1.10, False,   0),   # via (retorna do canto de B)
    ( 1.90,  0.10, False,   0),   # via (corredor central-E, passa abaixo de C)
    ( 4.90,  0.10, False,   0),   # via
    ( 5.00,  2.00, True,  180),   # BAIA C
    ( 4.90, -4.90, False,   0),   # via (desce pela direita)
    ( 3.00, -5.00, True,   90),   # BAIA D
    ( 0.00, -5.00, True,   90),   # HOME (recarga)
]
# Bancadas apenas para desenhar o grafico (nao sao obstaculos fisicos na cena):
BENCHES = {"A": (-4.3,-1.7,-2.5,-0.7), "B": (-4.3,-2.5,1.7,4.3),
           "C": (2.5,4.3,0.7,3.3),     "D": (1.7,4.3,-4.3,-2.5)}

def wrap(a): return (a + np.pi) % (2*np.pi) - np.pi

# ============================ SETUP DO ROBO ================================
P = Pioneer3DX()
P.pFlag.TrailON = True
# Pose inicial = Home, ja orientado a 90 graus
P.pPos.X[0] = 0.0; P.pPos.X[1] = -5.0; P.pPos.X[5] = np.deg2rad(90)

# --- NOTA/CORRECAO: o teto padrao max_wheel_vel=1.2 rad/s (~0.12 m/s) em
#     Pioneer3DX.rSendControlSignals e MUITO baixo para um circuito deste tamanho.
#     Elevo para ~6 rad/s (~0.585 m/s) sem editar a framework (monkeypatch):
def _send(self):
    vl = (self.pSC.Ud[0,0] - self.pSC.Ud[1,0]*0.331/2)/0.0975
    vr = (self.pSC.Ud[0,0] + self.pSC.Ud[1,0]*0.331/2)/0.0975
    vl = float(np.clip(vl, -WHEEL_CAP, WHEEL_CAP))
    vr = float(np.clip(vr, -WHEEL_CAP, WHEEL_CAP))
    self.sim.setJointTargetVelocity(self.left_motor_handle, vl)
    self.sim.setJointTargetVelocity(self.right_motor_handle, vr)
P.rSendControlSignals = types.MethodType(_send, P)
print(f"[NOTA] Teto de roda elevado para {WHEEL_CAP} rad/s (~{0.0975*WHEEL_CAP:.2f} m/s).")

P.rConnect()
P.rOpenScene("Pioneer.ttt")

# ---- PISO GRANDE: garante chao sob todo o circuito (+-6 m) ----
# O Floor padrao da cena e pequeno demais; o robo em (0,-5) cairia no vazio.
try:
    _floor = P.sim.createPrimitiveShape(P.sim.primitiveshape_cuboid, [14.0, 14.0, 0.10])
    P.sim.setObjectInt32Param(_floor, P.sim.shapeintparam_static, 1)
    P.sim.setObjectInt32Param(_floor, P.sim.shapeintparam_respondable, 1)
    P.sim.setObjectPosition(_floor, -1, [0.0, 0.0, -0.05])   # topo em z=0
    # recoloca o robo SOBRE o piso novo, no Home, orientado a 90 graus
    _a = np.deg2rad(90) / 2.0
    P.sim.setObjectPose(P.robot_handle, -1,
                        [0.0, -5.0, 0.20, 0.0, 0.0, float(np.sin(_a)), float(np.cos(_a))])
    print("[OK] Piso 14x14 m criado; robo reposicionado no Home (0,-5).")
except Exception as _e:
    print(f"[AVISO] Nao criei o piso via API ({_e}).")
    print("       Use o metodo manual (GUI) descrito no chat e re-salve o Pioneer.ttt.")

input("Pressione Enter para iniciar o circuito...")

# ============================ CONTROLADORES ================================
def ctrl_posicao(x, y, psi, xd, yd):
    """Linearizacao por realimentacao no ponto 'a' (AuRoRA)."""
    a = P.pPar.a
    A = np.array([[np.cos(psi), -a*np.sin(psi)],
                  [np.sin(psi),  a*np.cos(psi)]])
    err = np.array([xd - x, yd - y])
    Ud = np.linalg.pinv(A) @ (Kp_pos * err)      # feedforward de velocidade = 0
    u = float(np.clip(Ud[0], -U_MAX, U_MAX))
    w = float(np.clip(Ud[1], -W_MAX, W_MAX))
    # gate de rumo: evita avancos laterais/reversos -> so gira se muito desalinhado
    if abs(wrap(np.arctan2(yd - y, xd - x) - psi)) > GATE_TH:
        u = 0.0
    return u, w

def ctrl_orientacao(psi, psid):
    w = float(np.clip(Kpsi * wrap(psid - psi), -W_MAX, W_MAX))
    return 0.0, w

# ============================ LOOP + FSM ===================================
tmax_guard = 300.0
t_start = time.time(); tc_start = time.time()
XX = []

ti = 0                      # indice do alvo atual na ROUTE
phase = "FACE"             # FACE -> GOTO -> (ALIGN -> DWELL) -> proximo
dwell_until = 0.0
last_print = 0.0            # DEBUG: controla o print de posicao

while ti < len(ROUTE) and (time.time() - t_start) < tmax_guard:
    if time.time() - tc_start > P.pPar.Ts:
        tc_start = time.time()
        P.rGetSensorData()
        x, y, psi = P.pPos.X[0,0], P.pPos.X[1,0], P.pPos.X[5,0]
        xd, yd, is_baia, psid_deg = ROUTE[ti]
        psid = np.deg2rad(psid_deg)
        P.pPos.Xd[0] = xd; P.pPos.Xd[1] = yd
        dist = np.hypot(xd - x, yd - y)
        eps = EPS_DOCK if is_baia else EPS_VIA

        # ---------------- DEBUG: posicao a cada ~1 s ----------------
        if time.time() - last_print > 1.0:
            last_print = time.time()
            print(f"t={time.time()-t_start:5.1f}  x={x:+.2f}  y={y:+.2f}  "
                  f"z={P.pPos.X[2,0]:+.2f}  psi={np.rad2deg(psi):+6.1f}  "
                  f"fase={phase:5s}  alvo={ti}/{len(ROUTE)}  dist={dist:.2f}")
        if phase == "FACE":
            desired = np.arctan2(yd - y, xd - x)
            if dist < 0.3 or abs(wrap(desired - psi)) < FACE_TH:
                phase = "GOTO"
                u, w = ctrl_posicao(x, y, psi, xd, yd)
            else:
                u, w = ctrl_orientacao(psi, desired)

        elif phase == "GOTO":
            if dist < eps:
                if is_baia:
                    phase = "ALIGN"; u, w = ctrl_orientacao(psi, psid)
                else:
                    ti += 1; phase = "FACE"; u, w = 0.0, 0.0
            else:
                u, w = ctrl_posicao(x, y, psi, xd, yd)

        elif phase == "ALIGN":
            if abs(wrap(psid - psi)) < EPS_ANG:
                phase = "DWELL"; dwell_until = time.time() + DWELL; u, w = 0.0, 0.0
            else:
                u, w = ctrl_orientacao(psi, psid)

        elif phase == "DWELL":
            u, w = 0.0, 0.0
            if time.time() >= dwell_until:
                ti += 1; phase = "FACE"

        # ---------------- aplica e registra ----------------
        P.pSC.Ud = np.array([[u], [w]])
        P.rSendControlSignals()
        XX.append(np.concatenate((
            P.pPos.X[[0,1,5]].flatten(),      # 0,1,2 = x, y, psi
            [xd, yd],                         # 3,4   = xd, yd
            P.pSC.Ud.flatten(),               # 5,6   = u, w (comandados)
            [time.time() - t_start])))        # 7     = t

# Para com seguranca
P.pSC.Ud = np.array([[0.0], [0.0]]); P.rSendControlSignals()
P.rDisconnect()
XX = np.array(XX)
np.savetxt("Data_circuito.txt", XX, fmt="%.4f", delimiter="\t")
print(f"[OK] Circuito concluido em {XX[-1,7]:.1f} s. Dados em Data_circuito.txt")

# ============================ GRAFICOS =====================================
plt.rcParams.update({"figure.dpi": 120})

# Navegacao XY
fig1, ax = plt.subplots(figsize=(7, 6.4))
for name, (xm, xM, ym, yM) in BENCHES.items():
    ax.add_patch(plt.Rectangle((xm, ym), xM-xm, yM-ym, color="0.3"))
    ax.text((xm+xM)/2, (ym+yM)/2, f"Baia {name}", color="w", ha="center", va="center")
ax.plot(XX[:,0], XX[:,1], "b-", lw=2, label="Trajetoria")
for (xd, yd, isb, pd) in ROUTE:
    ax.plot(xd, yd, "r*" if isb else "o", ms=13 if isb else 4,
            color="crimson" if isb else "orange")
ax.set_aspect("equal"); ax.grid(alpha=0.3); ax.legend()
ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
ax.set_title("Navegacao no plano XY"); ax.set_xlim(-7,7); ax.set_ylim(-7,7)

# Temporais X, Y, psi
fig2, axs = plt.subplots(3, 1, figsize=(8, 6.5), sharex=True)
axs[0].plot(XX[:,7], XX[:,0], "b");            axs[0].set_ylabel("x [m]");  axs[0].grid(alpha=0.3)
axs[1].plot(XX[:,7], XX[:,1], "g");            axs[1].set_ylabel("y [m]");  axs[1].grid(alpha=0.3)
axs[2].plot(XX[:,7], np.rad2deg(XX[:,2]), "r");axs[2].set_ylabel("psi [deg]")
axs[2].set_xlabel("tempo [s]"); axs[2].grid(alpha=0.3)

# Sinais de controle u, w
fig3, axs = plt.subplots(2, 1, figsize=(8, 5), sharex=True)
axs[0].plot(XX[:,7], XX[:,5], "b"); axs[0].set_ylabel("u [m/s]");   axs[0].grid(alpha=0.3)
axs[1].plot(XX[:,7], XX[:,6], "r"); axs[1].set_ylabel("w [rad/s]")
axs[1].set_xlabel("tempo [s]"); axs[1].grid(alpha=0.3)

plt.tight_layout(); plt.show()