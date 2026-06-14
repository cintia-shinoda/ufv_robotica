%% sim_uniciclo.m
% Simulacao de robo uniciclo (tracao diferencial) em linha de montagem.
% Controle desacoplado em fases: controle de percurso (posicao) + controle de orientacao.
% Rota: Home -> Baia A -> Baia B -> Baia C -> Baia D -> Home (com via points anti-colisao).
%
% Convencao: identificadores em ingles, comentarios em portugues.
% Como rodar: abra no MATLAB e pressione "Run" (F5). Gera 6 arquivos PNG na pasta atual.
% Requisitos: exportgraphics exige R2020a+; xline/yline exigem R2018b+.
%             Para versoes antigas, ver alternativas comentadas no fim de cada figura.
%
% Modelo cinematico:  x' = v*cos(theta) ; y' = v*sin(theta) ; theta' = omega

clear; clc; close all;

%% ------------------------------------------------------------------------
% Ganhos e limites do controlador (parametros de projeto, ajustaveis)
% ------------------------------------------------------------------------
K_V   = 0.8;            % ganho do controle de percurso (linear)
K_W   = 2.0;            % ganho de direcionamento ao alvo (angular no deslocamento)
K_TH  = 2.5;            % ganho do controle de orientacao final
V_MAX = 0.6;            % velocidade linear maxima [m/s]
W_MAX = 1.5;            % velocidade angular maxima [rad/s]
DT    = 0.05;           % passo de integracao [s]

EPS_POS_DOCK = 0.05;        % tolerancia de posicao na baia [m]
EPS_POS_VIA  = 0.15;        % tolerancia de posicao em via point [m]
EPS_ANG      = deg2rad(1);  % tolerancia de orientacao final [rad]

%% ------------------------------------------------------------------------
% Pose inicial e lista de waypoints
%   Colunas: [x, y, theta_deg]   (theta = NaN  =>  via point, so posicao)
% ------------------------------------------------------------------------
start_x = 0.0;  start_y = -5.0;  start_th = deg2rad(90);  % Home, orientado a +y

WP = [ -4.0  -4.5   NaN ;   % via
       -5.0  -0.5   NaN ;   % via
       -3.0   0.0  -90  ;   % Baia A
       -6.0   1.0   NaN ;   % via
       -5.0   3.0    0  ;   % Baia B
       -6.0   4.3   NaN ;   % via
        6.0   4.3   NaN ;   % via
        5.0   2.0  180  ;   % Baia C
        6.0  -6.0   NaN ;   % via
        3.0  -5.0   90  ;   % Baia D
        0.0  -5.0   90  ];  % Home
names = {'via','via','A','via','B','via','via','C','via','D','Home'};

%% ------------------------------------------------------------------------
% Loop de simulacao
% ------------------------------------------------------------------------
x = start_x;  y = start_y;  th = start_th;

N = 400000;                                   % pre-alocacao generosa
T = zeros(1,N); X = T; Y = T; TH = T; V = T; W = T;
k = 1;
T(1)=0; X(1)=x; Y(1)=y; TH(1)=th; V(1)=0; W(1)=0;
t = 0;

dock_t = []; dock_name = {}; dock_x = []; dock_y = []; dock_th = [];

for i = 1:size(WP,1)
    xd = WP(i,1);  yd = WP(i,2);
    is_dock = ~isnan(WP(i,3));
    if is_dock, thd = deg2rad(WP(i,3)); eps_pos = EPS_POS_DOCK;
    else,       eps_pos = EPS_POS_VIA;  end

    % ---- Fase 1: CONTROLE DE PERCURSO (dirige ate xd, yd) ----
    while true
        rho = hypot(xd - x, yd - y);
        if rho < eps_pos, break; end
        alpha = wrap_to_pi(atan2(yd - y, xd - x) - th);
        % avanca apenas quando aproximadamente alinhado ao alvo (gira-entao-anda)
        v = min(K_V*rho, V_MAX) * max(0, cos(alpha));
        w = max(-W_MAX, min(W_MAX, K_W*alpha));
        x  = x + v*cos(th)*DT;
        y  = y + v*sin(th)*DT;
        th = wrap_to_pi(th + w*DT);
        t  = t + DT;  k = k + 1;
        T(k)=t; X(k)=x; Y(k)=y; TH(k)=th; V(k)=v; W(k)=w;
    end

    % ---- Fase 2: CONTROLE DE ORIENTACAO (alinha theta -> thd, v = 0) ----
    if is_dock
        while true
            e_th = wrap_to_pi(thd - th);
            if abs(e_th) < EPS_ANG, break; end
            v = 0;
            w = max(-W_MAX, min(W_MAX, K_TH*e_th));
            th = wrap_to_pi(th + w*DT);
            t  = t + DT;  k = k + 1;
            T(k)=t; X(k)=x; Y(k)=y; TH(k)=th; V(k)=v; W(k)=w;
        end
        dock_t(end+1)    = t;          %#ok<*SAGROW>
        dock_name{end+1} = names{i};
        dock_x(end+1)    = x;
        dock_y(end+1)    = y;
        dock_th(end+1)   = thd;
    end
end

% trunca os vetores ate o ultimo passo usado
T=T(1:k); X=X(1:k); Y=Y(1:k); TH=TH(1:k); V=V(1:k); W=W(1:k);

fprintf('Tempo total de execucao: %.1f s  (%d passos)\n', T(end), k);
fprintf('Erros finais de orientacao por baia:\n');
for d = 1:numel(dock_t)
    idx = find(T <= dock_t(d), 1, 'last');
    err = rad2deg(wrap_to_pi(dock_th(d) - TH(idx)));
    fprintf('  %4s: %+5.2f graus  (t = %.1f s)\n', dock_name{d}, err, dock_t(d));
end

%% ------------------------------------------------------------------------
% Geometria das baias (estimada a partir da figura) - apenas visualizacao
%   [cx, cy, w, h]  ;  robo entra de frente na baia
% ------------------------------------------------------------------------
bays      = [ -3.0 -1.3 2.4 1.2 ;
              -3.7  3.0 2.4 1.4 ;
               3.7  2.0 2.4 1.4 ;
               3.0 -3.6 2.4 1.2 ];
bay_names = {'Baia A','Baia B','Baia C','Baia D'};
dock_pts  = [ -3 0 ; -5 3 ; 5 2 ; 3 -5 ];

COL_PATH = [0.22 0.29 0.69];   % indigo
COL_BAY  = [0.22 0.28 0.31];   % cinza-azulado
COL_DOCK = [0.90 0.22 0.21];   % vermelho
COL_HOME = [0.18 0.49 0.20];   % verde

%% ====== GRAFICO 1: Navegacao no plano XY ======
f1 = figure('Color','w','Position',[100 100 720 720]); hold on;
for b = 1:size(bays,1)
    cx=bays(b,1); cy=bays(b,2); w=bays(b,3); h=bays(b,4);
    rectangle('Position',[cx-w/2, cy-h/2, w, h], 'FaceColor',COL_BAY, 'EdgeColor','k');
    text(cx, cy, bay_names{b}, 'Color','w', 'HorizontalAlignment','center', ...
         'FontWeight','bold', 'FontSize',9);
    plot(dock_pts(b,1), dock_pts(b,2), 'o', 'Color',COL_DOCK, ...
         'MarkerFaceColor',COL_DOCK, 'MarkerSize',8);
end
plot(X, Y, '-', 'Color',COL_PATH, 'LineWidth',2);
plot(start_x, start_y, 's', 'Color',COL_HOME, 'MarkerFaceColor',COL_HOME, 'MarkerSize',12);
for d = 1:numel(dock_t)   % setas de orientacao final
    quiver(dock_x(d), dock_y(d), 0.7*cos(dock_th(d)), 0.7*sin(dock_th(d)), 0, ...
           'Color',COL_DOCK, 'LineWidth',2, 'MaxHeadSize',2);
end
axis equal; xlim([-7.5 7.5]); ylim([-7.5 6.5]); grid on;
xlabel('X [m]'); ylabel('Y [m]');
title('Navegacao no plano XY (Home -> A -> B -> C -> D -> Home)');
exportgraphics(f1, '01_trajetoria_xy.png', 'Resolution', 140);
% Alternativa R<2020a:  print(f1,'01_trajetoria_xy.png','-dpng','-r140');

%% ====== GRAFICO 2: X(t) ======
f2 = figure('Color','w','Position',[100 100 900 340]);
plot(T, X, 'Color',COL_PATH, 'LineWidth',1.8); grid on;
xlabel('Tempo [s]'); ylabel('X [m]'); title('Evolucao temporal de X');
mark_docks(dock_t, dock_name, COL_DOCK);
exportgraphics(f2, '02_x_t.png', 'Resolution', 140);

%% ====== GRAFICO 3: Y(t) ======
f3 = figure('Color','w','Position',[100 100 900 340]);
plot(T, Y, 'Color',COL_PATH, 'LineWidth',1.8); grid on;
xlabel('Tempo [s]'); ylabel('Y [m]'); title('Evolucao temporal de Y');
mark_docks(dock_t, dock_name, COL_DOCK);
exportgraphics(f3, '03_y_t.png', 'Resolution', 140);

%% ====== GRAFICO 4: theta(t) ======
TH_u = rad2deg(unwrap(TH));   % unwrap para curva continua e legivel
f4 = figure('Color','w','Position',[100 100 900 340]);
plot(T, TH_u, 'Color',COL_PATH, 'LineWidth',1.8); grid on;
xlabel('Tempo [s]'); ylabel('theta [graus]');
title('Evolucao temporal da orientacao (theta)');
mark_docks(dock_t, dock_name, COL_DOCK);
exportgraphics(f4, '04_theta_t.png', 'Resolution', 140);

%% ====== GRAFICO 5: velocidade linear v(t) ======
f5 = figure('Color','w','Position',[100 100 900 340]);
plot(T, V, 'Color',[0 0.54 0.48], 'LineWidth',1.6); grid on; hold on;
yline(V_MAX, ':', sprintf('v_{max} = %.1f m/s', V_MAX), 'Color',[0.5 0.5 0.5]);
xlabel('Tempo [s]'); ylabel('v [m/s]');
title('Sinal de controle: velocidade linear v(t)');
mark_docks(dock_t, dock_name, COL_DOCK);
exportgraphics(f5, '05_v_t.png', 'Resolution', 140);

%% ====== GRAFICO 6: velocidade angular omega(t) ======
f6 = figure('Color','w','Position',[100 100 900 340]);
plot(T, W, 'Color',[0.78 0.16 0.16], 'LineWidth',1.4); grid on; hold on;
yline( W_MAX, ':', sprintf('+w_{max} = %.1f', W_MAX), 'Color',[0.5 0.5 0.5]);
yline(-W_MAX, ':', sprintf('-w_{max} = %.1f', W_MAX), 'Color',[0.5 0.5 0.5]);
xlabel('Tempo [s]'); ylabel('omega [rad/s]');
title('Sinal de controle: velocidade angular omega(t)');
mark_docks(dock_t, dock_name, COL_DOCK);
exportgraphics(f6, '06_omega_t.png', 'Resolution', 140);

fprintf('\nGraficos gerados: 01..06 PNG na pasta atual.\n');

%% ========================= FUNCOES LOCAIS =========================
function a = wrap_to_pi(a)
    % Normaliza angulo para o intervalo [-pi, pi]
    a = mod(a + pi, 2*pi) - pi;
end

function mark_docks(dock_t, dock_name, col)
    % Marca linhas verticais nos instantes de docagem
    for d = 1:numel(dock_t)
        xline(dock_t(d), '--', dock_name{d}, ...
              'Color', col, 'LabelVerticalAlignment','top', ...
              'LabelHorizontalAlignment','right', 'FontSize',8, ...
              'HandleVisibility','off');
    end
end
