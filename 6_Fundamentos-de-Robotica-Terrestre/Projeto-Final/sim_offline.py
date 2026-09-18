"""
Simulacao cinematica offline do circuito Home->A->B->C->D->Home.
Replica a matematica do controlador da AuRoRA (Pioneer3DX.sController):
  - Controle de POSICAO por linearizacao por realimentacao no ponto deslocado 'a'
  - Controle de ORIENTACao por rotacao no lugar (u=0, w = Kpsi * erro_psi)
Roteamento livre de colisao por GRAFO DE VISIBILIDADE sobre as bancadas infladas.

Objetivo: gerar os graficos (Q5 e Q6) e validar/afinar os ganhos ANTES do CoppeliaSim.
NAO usa CoppeliaSim: e um "digital twin" cinematico. As curvas do CoppeliaSim tendem
a ser proximas em forma, com pequenas diferencas por dinamica/atrito.
"""
import os
import numpy as np
import matplotlib.pyplot as plt
import heapq

# Pasta de saida = pasta deste script (funciona rode-se de onde for)
OUT = os.path.dirname(os.path.abspath(__file__))

# ============================================================
# 1) AMBIENTE: poses-alvo e bancadas (obstaculos)
# ============================================================
# Poses de atracacao (dock): (x, y, psi_desejado_em_graus)
DOCKS = {
    "Home": (0.0, -5.0, 90.0),
    "A":    (-3.0, 0.0, -90.0),
    "B":    (-5.0, 3.0,   0.0),
    "C":    ( 5.0, 2.0, 180.0),
    "D":    ( 3.0, -5.0, 90.0),
}
ORDER = ["Home", "A", "B", "C", "D", "Home"]

# [ESTIMATIVA] Footprint das bancadas. Nao consegui extrair da cena .ttt (comprimida).
# Modelo: cada bancada fica "a frente" do dock, na direcao psi (robo encara a bancada
# para entregar). Retangulos alinhados aos eixos: (xmin, xmax, ymin, ymax).
BENCHES = {
    "A": (-4.3, -1.7, -2.5, -0.7),   # bancada ao sul do dock A (psi=-90)
    "B": (-4.3, -2.5,  1.7,  4.3),   # bancada a leste do dock B (psi=0)
    "C": ( 2.5,  4.3,  0.7,  3.3),   # bancada a oeste do dock C (psi=180)
    "D": ( 1.7,  4.3, -4.3, -2.5),   # bancada ao norte do dock D (psi=90)
}
MARGIN = 0.55  # margem de seguranca (raio do robo + folga) [m]

# ============================================================
# 2) PLANEJADOR DE ROTA: grafo de visibilidade
# ============================================================
def inflate(rect, m):
    xmin, xmax, ymin, ymax = rect
    return (xmin - m, xmax + m, ymin - m, ymax + m)

def seg_hits_rect(p, q, rect, eps=1e-6):
    """True se o segmento p-q cruza o INTERIOR do retangulo (com leve encolhimento)."""
    xmin, xmax, ymin, ymax = rect
    xmin += eps; xmax -= eps; ymin += eps; ymax -= eps
    if xmax <= xmin or ymax <= ymin:
        return False
    # Liang-Barsky clipping
    x0, y0 = p; x1, y1 = q
    dx, dy = x1 - x0, y1 - y0
    t0, t1 = 0.0, 1.0
    for pk, qk in [(-dx, x0 - xmin), (dx, xmax - x0), (-dy, y0 - ymin), (dy, ymax - y0)]:
        if abs(pk) < 1e-12:
            if qk < 0:
                return False
        else:
            r = qk / pk
            if pk < 0:
                if r > t1: return False
                if r > t0: t0 = r
            else:
                if r < t0: return False
                if r < t1: t1 = r
    return t0 < t1

def visible(p, q, benches_infl):
    return not any(seg_hits_rect(p, q, r) for r in benches_infl)

def build_route():
    benches_infl = [inflate(r, MARGIN) for r in BENCHES.values()]
    # nos: docks + cantos das bancadas infladas (levemente afastados p/ nao ficar na borda)
    nodes = []
    for name in DOCKS:
        nodes.append((round(DOCKS[name][0], 3), round(DOCKS[name][1], 3)))
    for r in [inflate(rr, MARGIN + 0.05) for rr in BENCHES.values()]:
        xmin, xmax, ymin, ymax = r
        nodes += [(xmin, ymin), (xmin, ymax), (xmax, ymin), (xmax, ymax)]
    nodes = list(dict.fromkeys(nodes))  # unicos, preservando ordem

    # grafo de visibilidade
    idx = {n: i for i, n in enumerate(nodes)}
    adj = {i: [] for i in range(len(nodes))}
    for i in range(len(nodes)):
        for j in range(i + 1, len(nodes)):
            if visible(nodes[i], nodes[j], benches_infl):
                d = np.hypot(nodes[i][0] - nodes[j][0], nodes[i][1] - nodes[j][1])
                adj[i].append((j, d)); adj[j].append((i, d))

    def dijkstra(src, dst):
        dist = {i: np.inf for i in range(len(nodes))}
        prev = {i: None for i in range(len(nodes))}
        dist[src] = 0; pq = [(0, src)]
        while pq:
            d, u = heapq.heappop(pq)
            if u == dst: break
            if d > dist[u]: continue
            for v, w in adj[u]:
                nd = d + w
                if nd < dist[v]:
                    dist[v] = nd; prev[v] = u; heapq.heappush(pq, (nd, v))
        path = []; u = dst
        while u is not None:
            path.append(nodes[u]); u = prev[u]
        return path[::-1]

    # concatena as pernas do circuito
    full = []
    for k in range(len(ORDER) - 1):
        a = idx[(round(DOCKS[ORDER[k]][0], 3), round(DOCKS[ORDER[k]][1], 3))]
        b = idx[(round(DOCKS[ORDER[k + 1]][0], 3), round(DOCKS[ORDER[k + 1]][1], 3))]
        leg = dijkstra(a, b)
        if full and leg and full[-1] == leg[0]:
            leg = leg[1:]
        full += leg
    return full

# ============================================================
# 3) CONTROLADOR (replica AuRoRA) + maquina de estados
# ============================================================
a_off = 0.15          # ponto deslocado (Pioneer3DX.pPar.a)
Ts    = 0.05          # passo de integracao (sim mais fino que o Ts=0.1 do hardware)
Kp_pos = 0.7          # ganho do controle de posicao (Kd na AuRoRA)
Kpsi   = 1.8          # ganho do controle de orientacao
U_MAX  = 0.55         # saturacao de velocidade linear [m/s]
W_MAX  = 1.5          # saturacao de velocidade angular [rad/s]
EPS_VIA = 0.22        # tolerancia p/ via-point [m]
EPS_DOCK = 0.08       # tolerancia p/ dock [m]
EPS_ANG = np.deg2rad(1.5)
FACE_TH = np.deg2rad(20)  # se erro de rumo > isso, gira no lugar antes de andar
GATE_TH = np.deg2rad(55)  # durante GOTO: se erro de rumo > isso, zera u (so gira)
DWELL = 1.0           # tempo parado na baia (entrega) [s]

def wrap(ang):
    return (ang + np.pi) % (2 * np.pi) - np.pi

def pos_control(x, y, psi, xd, yd):
    A = np.array([[np.cos(psi), -a_off * np.sin(psi)],
                  [np.sin(psi),  a_off * np.cos(psi)]])
    err = np.array([xd - x, yd - y])
    Ud = np.linalg.pinv(A) @ (Kp_pos * err)   # Xd_dot=0 em regulacao de waypoint
    u, w = Ud[0], Ud[1]
    return np.clip(u, -U_MAX, U_MAX), np.clip(w, -W_MAX, W_MAX)

def simulate(route):
    # estado inicial = Home, psi inicial 90 graus (ja na pose Home)
    x, y = DOCKS["Home"][0], DOCKS["Home"][1]
    psi = np.deg2rad(90.0)
    log = {k: [] for k in ["t", "x", "y", "psi", "u", "w", "xd", "yd"]}
    t = 0.0

    # marca quais indices da rota sao docks (com psi alvo e dwell)
    dock_xy = {(round(DOCKS[n][0], 3), round(DOCKS[n][1], 3)): DOCKS[n][2]
               for n in ORDER}

    def step(u, w, xd, yd):
        nonlocal x, y, psi, t
        psi = wrap(psi + w * Ts)
        x += u * np.cos(psi) * Ts
        y += u * np.sin(psi) * Ts
        for key, val in zip(["t","x","y","psi","u","w","xd","yd"],
                            [t, x, y, psi, u, w, xd, yd]):
            log[key].append(val)
        t += Ts

    for k, (xd, yd) in enumerate(route):
        key = (round(xd, 3), round(yd, 3))
        is_dock = key in dock_xy
        eps = EPS_DOCK if is_dock else EPS_VIA

        # --- FACE: gira no lugar p/ apontar ao alvo se erro de rumo grande (saida da baia)
        if np.hypot(xd - x, yd - y) > 0.3:
            desired = np.arctan2(yd - y, xd - x)
            guard = 0
            while abs(wrap(desired - psi)) > FACE_TH and guard < 400:
                w = np.clip(Kpsi * wrap(desired - psi), -W_MAX, W_MAX)
                step(0.0, w, xd, yd); guard += 1

        # --- GOTO: controle de posicao ate chegar (FL da AuRoRA)
        guard = 0
        while np.hypot(xd - x, yd - y) > eps and guard < 4000:
            u, w = pos_control(x, y, psi, xd, yd)
            # gate de rumo: evita "lunges" laterais/reversos -> gira primeiro
            head_err = wrap(np.arctan2(yd - y, xd - x) - psi)
            if abs(head_err) > GATE_TH:
                u = 0.0
            step(u, w, xd, yd); guard += 1

        # --- ALIGN + DWELL: no dock, gira p/ orientacao de entrega e pausa
        if is_dock and k > 0:  # nao realinha no Home inicial
            psid = np.deg2rad(dock_xy[key])
            guard = 0
            while abs(wrap(psid - psi)) > EPS_ANG and guard < 800:
                w = np.clip(Kpsi * wrap(psid - psi), -W_MAX, W_MAX)
                step(0.0, w, xd, yd); guard += 1
            for _ in range(int(DWELL / Ts)):
                step(0.0, 0.0, xd, yd)

    return {k: np.array(v) for k, v in log.items()}

# ============================================================
# 4) EXECUCAO + GRAFICOS
# ============================================================
if __name__ == "__main__":
    route = build_route()
    print("Rota (via-points):")
    for p in route:
        print(f"  ({p[0]:+.2f}, {p[1]:+.2f})")
    L = sim = simulate(route)
    print(f"\nTempo total simulado: {L['t'][-1]:.1f} s | passos: {len(L['t'])}")

    # cores/estilo
    plt.rcParams.update({"figure.dpi": 130, "font.size": 10})

    # ---- FIG 1: Navegacao no plano XY ----
    fig1, ax = plt.subplots(figsize=(7.2, 6.4))
    for name, r in BENCHES.items():
        xmin, xmax, ymin, ymax = r
        ax.add_patch(plt.Rectangle((xmin, ymin), xmax - xmin, ymax - ymin,
                                    color="0.25", alpha=0.9, zorder=2))
        ri = inflate(r, MARGIN)
        ax.add_patch(plt.Rectangle((ri[0], ri[2]), ri[1]-ri[0], ri[3]-ri[2],
                                    fill=False, ls="--", ec="0.6", lw=1, zorder=2))
        ax.text((xmin+xmax)/2, (ymin+ymax)/2, f"Baia {name}", color="w",
                ha="center", va="center", fontsize=9, zorder=3)
    ax.plot(L["x"], L["y"], color="#1f77b4", lw=2, label="Trajetoria do robo", zorder=4)
    rx = [p[0] for p in route]; ry = [p[1] for p in route]
    ax.plot(rx, ry, "o--", color="#ff7f0e", ms=4, lw=1, alpha=0.7,
            label="Rota planejada (via-points)", zorder=3)
    for name, (dx, dy, dpsi) in DOCKS.items():
        ax.plot(dx, dy, "*", color="crimson", ms=15, zorder=5)
        ax.annotate(f"{name}\n({dx:+.0f},{dy:+.0f},{dpsi:+.0f}°)",
                    (dx, dy), textcoords="offset points", xytext=(8, 8),
                    fontsize=8, color="crimson")
        L_ = 0.7
        ax.arrow(dx, dy, L_*np.cos(np.deg2rad(dpsi)), L_*np.sin(np.deg2rad(dpsi)),
                 head_width=0.18, color="crimson", zorder=5)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title("Navegacao no plano XY — circuito Home→A→B→C→D→Home")
    ax.set_aspect("equal"); ax.grid(True, alpha=0.3); ax.legend(loc="upper left", fontsize=8)
    ax.set_xlim(-7, 7); ax.set_ylim(-7, 7)
    fig1.tight_layout(); fig1.savefig(os.path.join(OUT, "nav_xy.png"), bbox_inches="tight")

    # ---- FIG 2: Evolucao temporal de X, Y, psi ----
    fig2, axs = plt.subplots(3, 1, figsize=(8, 6.5), sharex=True)
    axs[0].plot(L["t"], L["x"], color="#1f77b4"); axs[0].set_ylabel("x [m]"); axs[0].grid(alpha=0.3)
    axs[0].set_title("Evolucao temporal de X, Y e ψ")
    axs[1].plot(L["t"], L["y"], color="#2ca02c"); axs[1].set_ylabel("y [m]"); axs[1].grid(alpha=0.3)
    axs[2].plot(L["t"], np.rad2deg(L["psi"]), color="#d62728")
    axs[2].set_ylabel("ψ [°]"); axs[2].set_xlabel("tempo [s]"); axs[2].grid(alpha=0.3)
    # marca as orientacoes-alvo
    fig2.tight_layout(); fig2.savefig(os.path.join(OUT, "temporal_xypsi.png"), bbox_inches="tight")

    # ---- FIG 3: Sinais de controle u e w ----
    fig3, axs = plt.subplots(2, 1, figsize=(8, 5), sharex=True)
    axs[0].plot(L["t"], L["u"], color="#1f77b4"); axs[0].set_ylabel("u [m/s]"); axs[0].grid(alpha=0.3)
    axs[0].set_title("Sinais de controle: velocidade linear u e angular ω")
    axs[0].axhline(U_MAX, ls=":", color="0.6"); axs[0].axhline(-U_MAX, ls=":", color="0.6")
    axs[1].plot(L["t"], L["w"], color="#d62728"); axs[1].set_ylabel("ω [rad/s]")
    axs[1].set_xlabel("tempo [s]"); axs[1].grid(alpha=0.3)
    axs[1].axhline(W_MAX, ls=":", color="0.6"); axs[1].axhline(-W_MAX, ls=":", color="0.6")
    fig3.tight_layout(); fig3.savefig(os.path.join(OUT, "control_signals.png"), bbox_inches="tight")

    # ---- GRAFICOS INDIVIDUAIS (1 por item, p/ o formato do enunciado) ----
    def _single(tvec, yvec, ylabel, title, fname, color, sat=None):
        f, a = plt.subplots(figsize=(7, 3.2))
        a.plot(tvec, yvec, color=color, lw=1.6)
        if sat is not None:
            a.axhline(sat, ls=":", color="0.6"); a.axhline(-sat, ls=":", color="0.6")
        a.set_xlabel("tempo [s]"); a.set_ylabel(ylabel)
        a.set_title(title); a.grid(alpha=0.3)
        f.tight_layout(); f.savefig(os.path.join(OUT, fname), bbox_inches="tight")

    _single(L["t"], L["x"], "x [m]",  "Evolucao temporal de X",  "x_t.png",  "#1f77b4")
    _single(L["t"], L["y"], "y [m]",  "Evolucao temporal de Y",  "y_t.png",  "#2ca02c")
    _single(L["t"], np.rad2deg(L["psi"]), "psi [graus]", "Evolucao temporal de psi", "psi_t.png", "#d62728")
    _single(L["t"], L["u"], "u [m/s]",   "Evolucao temporal de u (vel. linear)",  "u_t.png", "#1f77b4", sat=U_MAX)
    _single(L["t"], L["w"], "w [rad/s]", "Evolucao temporal de w (vel. angular)", "w_t.png", "#d62728", sat=W_MAX)

    print(f"Graficos salvos em: {OUT}")
    plt.show()   # preview (feche as janelas para encerrar)