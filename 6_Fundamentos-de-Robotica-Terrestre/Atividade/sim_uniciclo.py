"""
Simulacao de robo uniciclo (tracao diferencial) em linha de montagem.
Controle desacoplado em fases: controle de percurso (posicao) + controle de orientacao.
Rota: Home -> Baia A -> Baia B -> Baia C -> Baia D -> Home (com via points para evitar colisao).

Convencao de codigo: identificadores em ingles, comentarios em portugues.
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyArrow

# ----------------------------------------------------------------------------
# Modelo cinematico do uniciclo (Euler):
#   x_dot = v*cos(theta)
#   y_dot = v*sin(theta)
#   theta_dot = omega
# ----------------------------------------------------------------------------

def wrap_to_pi(angle):
    """Normaliza angulo para o intervalo [-pi, pi]."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


# ----------------------------------------------------------------------------
# Ganhos e limites do controlador (parametros de projeto, ajustaveis)
# ----------------------------------------------------------------------------
K_V   = 0.8    # ganho do controle de percurso (linear)
K_W   = 2.0    # ganho de direcionamento ao alvo (angular durante deslocamento)
K_TH  = 2.5    # ganho do controle de orientacao final
V_MAX = 0.6    # velocidade linear maxima [m/s]
W_MAX = 1.5    # velocidade angular maxima [rad/s]
DT    = 0.05   # passo de integracao [s]

EPS_POS_DOCK = 0.05   # tolerancia de posicao na baia [m]
EPS_POS_VIA  = 0.15   # tolerancia de posicao em via point [m]
EPS_ANG      = np.deg2rad(1.0)  # tolerancia de orientacao final [rad]

# ----------------------------------------------------------------------------
# Pose inicial e lista de waypoints
#   kind: 'via' (so posicao), ou nome da baia/home (posicao + orientacao final)
# ----------------------------------------------------------------------------
start = (0.0, -5.0, np.deg2rad(90.0))  # Home, orientado a +y (90 graus)

waypoints = [
    (-4.0, -4.5, None,            'via'),
    (-5.0, -0.5, None,            'via'),
    (-3.0,  0.0, np.deg2rad(-90), 'A'),
    (-6.0,  1.0, None,            'via'),
    (-5.0,  3.0, np.deg2rad(0),   'B'),
    (-6.0,  4.3, None,            'via'),
    ( 6.0,  4.3, None,            'via'),
    ( 5.0,  2.0, np.deg2rad(180), 'C'),
    ( 6.0, -6.0, None,            'via'),
    ( 3.0, -5.0, np.deg2rad(90),  'D'),
    ( 0.0, -5.0, np.deg2rad(90),  'Home'),
]

# ----------------------------------------------------------------------------
# Loop de simulacao
# ----------------------------------------------------------------------------
x, y, th = start
T, X, Y, TH, V, W, PHASE = [0.0], [x], [y], [th], [0.0], [0.0], [0]
dock_events = []  # (tempo, nome, x, y) quando uma baia e alinhada

t = 0.0
MAX_STEPS = 200000

for (xd, yd, thd, kind) in waypoints:
    is_dock = thd is not None
    eps_pos = EPS_POS_DOCK if is_dock else EPS_POS_VIA

    # ---- Fase 1: CONTROLE DE PERCURSO (dirige ate (xd, yd)) ----
    steps = 0
    while steps < MAX_STEPS:
        rho = np.hypot(xd - x, yd - y)
        if rho < eps_pos:
            break
        alpha = wrap_to_pi(np.arctan2(yd - y, xd - x) - th)
        # Avanca apenas quando aproximadamente alinhado ao alvo (gira-entao-anda)
        v = min(K_V * rho, V_MAX) * max(0.0, np.cos(alpha))
        w = np.clip(K_W * alpha, -W_MAX, W_MAX)

        x  += v * np.cos(th) * DT
        y  += v * np.sin(th) * DT
        th  = wrap_to_pi(th + w * DT)
        t  += DT
        steps += 1
        T.append(t); X.append(x); Y.append(y); TH.append(th)
        V.append(v); W.append(w); PHASE.append(1)

    # ---- Fase 2: CONTROLE DE ORIENTACAO (alinha theta -> thd, v = 0) ----
    if is_dock:
        steps = 0
        while steps < MAX_STEPS:
            e_th = wrap_to_pi(thd - th)
            if abs(e_th) < EPS_ANG:
                break
            v = 0.0
            w = np.clip(K_TH * e_th, -W_MAX, W_MAX)
            th = wrap_to_pi(th + w * DT)
            t += DT
            steps += 1
            T.append(t); X.append(x); Y.append(y); TH.append(th)
            V.append(v); W.append(w); PHASE.append(2)
        dock_events.append((t, kind, x, y, thd))

# Converte para arrays
T  = np.array(T);  X = np.array(X);  Y = np.array(Y)
TH = np.array(TH); V = np.array(V);  W = np.array(W)

print(f"Tempo total de execucao: {T[-1]:.1f} s  ({len(T)} passos)")
print("Erros finais de docagem (pos[m] / ori[graus]):")
for (te, name, xe, ye, thde) in dock_events:
    # localiza pose registrada ao fim do alinhamento
    idx = np.argmin(np.abs(T - te))
    err_th = np.rad2deg(wrap_to_pi(thde - TH[idx]))
    print(f"  {name:>4}: alvo=({_x},{_y})" if False else
          f"  {name:>4}: erro_ori = {err_th:+.2f} graus  (t={te:.1f}s)")

# theta continuo (unwrap) para grafico legivel
TH_unwrap = np.unwrap(TH)

# ----------------------------------------------------------------------------
# Geometria das baias (estimada a partir da figura) - apenas para visualizacao
# Cada baia: dock na FACE da baia; robo orientado PARA DENTRO da baia.
# (cx, cy, w, h, nome, dock_x, dock_y)
# ----------------------------------------------------------------------------
bays = [
    (-3.0, -1.3, 2.4, 1.2, 'Baia A', -3.0,  0.0),  # robo a -90 (sul) entra na baia
    (-3.7,  3.0, 2.4, 1.4, 'Baia B', -5.0,  3.0),  # robo a 0   (leste) entra na baia
    ( 3.7,  2.0, 2.4, 1.4, 'Baia C',  5.0,  2.0),  # robo a 180 (oeste) entra na baia
    ( 3.0, -3.6, 2.4, 1.2, 'Baia D',  3.0, -5.0),  # robo a 90  (norte) entra na baia
]

COL_PATH  = '#3949ab'  # indigo
COL_BAY   = '#37474f'  # cinza-azulado
COL_DOCK  = '#e53935'  # vermelho
COL_HOME  = '#2e7d32'  # verde
plt.rcParams.update({'font.size': 11, 'axes.grid': True,
                     'grid.alpha': 0.3, 'figure.dpi': 120})

# ====== GRAFICO 1: Navegacao no plano XY ======
fig, ax = plt.subplots(figsize=(8, 8))
for (cx, cy, w, h, name, dx, dy) in bays:
    ax.add_patch(Rectangle((cx - w/2, cy - h/2), w, h,
                           facecolor=COL_BAY, edgecolor='black', alpha=0.85, zorder=2))
    ax.text(cx, cy, name, color='white', ha='center', va='center',
            fontsize=9, fontweight='bold', zorder=3)
    ax.plot(dx, dy, 'o', color=COL_DOCK, ms=9, zorder=4)

ax.plot(X, Y, '-', color=COL_PATH, lw=2.0, label='Trajetoria do robo', zorder=1)
ax.plot(start[0], start[1], 's', color=COL_HOME, ms=13, label='Home / Base de recarga', zorder=5)

# setas de orientacao final em cada docagem
for (te, name, xe, ye, thde) in dock_events:
    ax.annotate('', xy=(xe + 0.7*np.cos(thde), ye + 0.7*np.sin(thde)),
                xytext=(xe, ye),
                arrowprops=dict(arrowstyle='-|>', color=COL_DOCK, lw=2.2), zorder=6)

ax.set_xlabel('X [m]'); ax.set_ylabel('Y [m]')
ax.set_title('Navegacao no plano XY  (rota: Home -> A -> B -> C -> D -> Home)')
ax.set_aspect('equal'); ax.set_xlim(-7.5, 7.5); ax.set_ylim(-7.5, 6.5)
ax.legend(loc='lower right', fontsize=9)
plt.tight_layout(); plt.savefig('/home/claude/01_trajetoria_xy.png', dpi=140); plt.close()

# Tempos de docagem para marcar nos graficos temporais
dock_times = [(te, name) for (te, name, *_ ) in dock_events]

def mark_docks(ax):
    for (te, name) in dock_times:
        ax.axvline(te, color=COL_DOCK, ls='--', lw=0.9, alpha=0.6)
        ax.text(te, ax.get_ylim()[1], f' {name}', color=COL_DOCK,
                fontsize=8, va='top', ha='left')

# ====== GRAFICO 2: X(t) ======
fig, ax = plt.subplots(figsize=(9, 3.4))
ax.plot(T, X, color=COL_PATH, lw=1.8)
ax.set_xlabel('Tempo [s]'); ax.set_ylabel('X [m]'); ax.set_title('Evolucao temporal de X')
mark_docks(ax); plt.tight_layout(); plt.savefig('/home/claude/02_x_t.png', dpi=140); plt.close()

# ====== GRAFICO 3: Y(t) ======
fig, ax = plt.subplots(figsize=(9, 3.4))
ax.plot(T, Y, color=COL_PATH, lw=1.8)
ax.set_xlabel('Tempo [s]'); ax.set_ylabel('Y [m]'); ax.set_title('Evolucao temporal de Y')
mark_docks(ax); plt.tight_layout(); plt.savefig('/home/claude/03_y_t.png', dpi=140); plt.close()

# ====== GRAFICO 4: theta(t) ======
fig, ax = plt.subplots(figsize=(9, 3.4))
ax.plot(T, np.rad2deg(TH_unwrap), color=COL_PATH, lw=1.8)
# linhas de referencia das orientacoes alvo
for (te, name, xe, ye, thde) in dock_events:
    idx = np.argmin(np.abs(T - te))
    ax.plot(te, np.rad2deg(TH_unwrap[idx]), 'o', color=COL_DOCK, ms=6)
ax.set_xlabel('Tempo [s]'); ax.set_ylabel('theta [graus]')
ax.set_title('Evolucao temporal da orientacao (theta)')
mark_docks(ax); plt.tight_layout(); plt.savefig('/home/claude/04_theta_t.png', dpi=140); plt.close()

# ====== GRAFICO 5: velocidade linear v(t) ======
fig, ax = plt.subplots(figsize=(9, 3.4))
ax.plot(T, V, color='#00897b', lw=1.6)
ax.axhline(V_MAX, color='gray', ls=':', lw=1, label=f'v_max = {V_MAX} m/s')
ax.set_xlabel('Tempo [s]'); ax.set_ylabel('v [m/s]')
ax.set_title('Sinal de controle: velocidade linear v(t)')
ax.legend(fontsize=8, loc='upper right')
mark_docks(ax); plt.tight_layout(); plt.savefig('/home/claude/05_v_t.png', dpi=140); plt.close()

# ====== GRAFICO 6: velocidade angular omega(t) ======
fig, ax = plt.subplots(figsize=(9, 3.4))
ax.plot(T, W, color='#c62828', lw=1.4)
ax.axhline( W_MAX, color='gray', ls=':', lw=1)
ax.axhline(-W_MAX, color='gray', ls=':', lw=1, label=f'+/- w_max = {W_MAX} rad/s')
ax.set_xlabel('Tempo [s]'); ax.set_ylabel('omega [rad/s]')
ax.set_title('Sinal de controle: velocidade angular omega(t)')
ax.legend(fontsize=8, loc='upper right')
mark_docks(ax); plt.tight_layout(); plt.savefig('/home/claude/06_omega_t.png', dpi=140); plt.close()

print("\nGraficos gerados: 01..06 PNG")
