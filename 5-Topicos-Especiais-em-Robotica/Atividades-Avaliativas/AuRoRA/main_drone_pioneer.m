%% =====================================================================
%  main_drone_pioneer.m
%  Simulacao cooperativa: Bebop teleoperado + Pioneer 3-DX seguidor
%  Plataforma AuRoRA (NERo/UFV) - MODO SIMULACAO
%
%  Baseado nos padroes de:
%    Ex01_LerBebop_Teclado.m  (teleop + teclado do Bebop)
%    Sim_P3DX_*.m             (controle do Pioneer)
%
%  API real (confirmada por inspecao + exemplos):
%    pose      -> obj.pPos.X      (1-3 xyz, 6 psi)
%    cmd       -> obj.pSC.Ud      (Bebop:[vx;vy;vz;vpsi] | Pioneer:[v;w])
%    integrar  -> obj.rSendControlSignals  (avanca dinamica em simulacao)
%    ler       -> obj.rGetSensorData
%    pose ini  -> obj.rSetPose([x y z psi])  (Pioneer)
% =====================================================================

delete(findall(0,'Type','figure')); clear; clc;   % fecha figuras travadas sem disparar callbacks

%% ---------- 0. PATH (adapte FolderKey se necessario) ----------
% Mesma estrategia dos exemplos da disciplina
FolderCurrent = which(mfilename);
FolderKey = '\ELT 539';
FolderRootId = strfind(FolderCurrent, FolderKey);
if ~isempty(FolderRootId)
    FolderRoot = FolderCurrent(1:FolderRootId(end)+numel(FolderKey)-1);
    addpath(genpath(FolderRoot));
end
% Em Mac, se a linha acima nao pegar, garanta manualmente:
%   addpath(genpath('/Users/.../1_AuRoRA'))

%% ---------- 1. PARAMETROS ----------
tmax    = 60;          % duracao da simulacao [s]
Tctrl   = 1/30;        % passo de controle [s]
Tplot   = 0.1;         % passo de plotagem [s]

% Teleoperacao do Bebop (ganho de velocidade, como no Ex01)
kdrone  = 0.5;

% Controlador do Pioneer (seguir x,y do drone)
umax    = 0.35;        % velocidade linear max [m/s]
wmax    = 0.44;        % velocidade angular max [rad/s]
rho_min = 0.03;        % erro minimo de posicao [m]

%% ---------- 2. INSTANCIACAO ----------
disp('Inicializando AuRoRA (modo simulacao)...');
B = Bebop;             % drone
P = Pioneer3DX;        % robo terrestre

% Pose inicial
B.pPos.X(3) = 1.0;             % drone a 1 m de altura
P.rSetPose([0 0 0 0]);         % Pioneer na origem

disp('Bebop e Pioneer3DX instanciados.');

%% ---------- 3. FIGURA + TECLADO ----------
limites = [-5 5 -5 5 0 2];
f1 = figure('Name','Bebop (teclado) + Pioneer (seguidor)','NumberTitle','off');
f1.Position = [80 80 950 680];
set(gcf, 'KeyPressFcn', @keyDown, 'KeyReleaseFcn', @keyUp);

Ground = patch(limites([1 1 2 2]), limites([3 4 4 3]), [0 0 0 0], [0.6 1 0.6]);
Ground.FaceAlpha = 0.3;
view(45,30); axis equal; axis(limites); grid on; hold on;
lighting phong; material shiny; light;
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');

B.mCADplot;  B.mCADcolor([0 51 80]/255);
P.mCADplot;  P.mCADcolor([0.5 0.5 0.5]);

% Rastros e titulo sao criados/recriados dentro do loop (mCADplot limpa o eixo)
drawnow;

% Estado do teclado (padrao do Ex01: global keyState)
global keyState
keyState = struct('up',false,'down',false,'left',false,'right',false, ...
                  'w',false,'s',false,'a',false,'d',false, ...
                  'space',false,'abort',false);

disp('Controles do drone (clique na area vazia da figura primeiro):');
disp('  Setas = mover no plano | W/S = subir/descer | A/D = girar');
disp('  ESPACO = parar | ESC ou fechar janela = encerrar');
fprintf('Simulando %.0fs...\n', tmax);
pause(1);

%% ---------- 4. PRE-ALOCACAO DO LOG ----------
N = ceil(tmax/Tctrl) + 10;
log.t       = zeros(N,1);
log.u_drone = zeros(N,4);
log.x_drone = zeros(N,4);
log.u_pio   = zeros(N,2);
log.x_pio   = zeros(N,3);
log.rho     = zeros(N,1);
log.psi_err = zeros(N,1);
log.Ts = Tctrl;
ki = 0;

%% ---------- 5. LOOP PRINCIPAL ----------
t  = tic;
tc = tic;
tp = tic;

while toc(t) < tmax
    if ~ishandle(f1) || keyState.abort
        disp('Encerrado pelo operador.'); break;
    end

    %% --- Ciclo de controle (30 Hz) ---
    if toc(tc) > Tctrl
        tc = tic;

        % ===== BEBOP: ler -> teleoperar -> integrar =====
        B.rGetSensorData;
        vx=0; vy=0; vz=0; vpsi=0;
        if ~keyState.space
            if keyState.up,    vx =  kdrone; elseif keyState.down,  vx = -kdrone; end
            if keyState.right, vy =  kdrone; elseif keyState.left,  vy = -kdrone; end
            if keyState.w,     vz =  kdrone; elseif keyState.s,     vz = -kdrone; end
            if keyState.a,     vpsi =  kdrone; elseif keyState.d,   vpsi = -kdrone; end
        end
        B.pSC.Ud = [vx; vy; vz; vpsi];
        B.rSendControlSignals;     % integra a dinamica do drone

        % ===== PIONEER: ler -> controlar p/ seguir (x,y) do drone =====
        P.rGetSensorData;

        % Referencia = posicao XY do drone
        P.pPos.Xd(1:2) = [B.pPos.X(1); B.pPos.X(2)];

        % Erro e coordenadas polares (padrao AuRoRA)
        P.pPos.Xtil = P.pPos.Xd - P.pPos.X;
        rho   = norm(P.pPos.Xtil(1:2));
        theta = atan2(P.pPos.Xtil(2), P.pPos.Xtil(1));
        alpha = theta - P.pPos.X(6);
        if abs(alpha) > pi
            if alpha > 0, alpha = -2*pi + alpha; else, alpha = 2*pi + alpha; end
        end

        % Lei de controle (mesma forma do exemplo do Pioneer)
        if rho > rho_min
            P.pSC.Ud(1) = umax*tanh(rho)*cos(alpha);
            P.pSC.Ud(2) = wmax*alpha + umax*(tanh(rho)/rho)*sin(alpha)*cos(alpha);
        else
            P.pSC.Ud(1:2) = 0;
        end
        P.rSendControlSignals;     % integra a cinematica do Pioneer

        % ===== LOG =====
        ki = ki + 1;
        log.t(ki)         = toc(t);
        log.u_drone(ki,:) = [vx vy vz vpsi];
        log.x_drone(ki,:) = [B.pPos.X(1) B.pPos.X(2) B.pPos.X(3) B.pPos.X(6)];
        log.u_pio(ki,:)   = P.pSC.Ud(1:2).';
        log.x_pio(ki,:)   = [P.pPos.X(1) P.pPos.X(2) P.pPos.X(6)];
        log.rho(ki)       = rho;
        log.psi_err(ki)   = alpha;
    end

    %% --- Plotagem (10 Hz) ---
    if toc(tp) > Tplot
        tp = tic;
        % mCADplot pode limpar objetos do eixo; recriamos os rastros a cada frame
        if exist('hd','var') && all(ishandle([hd hp hl])), delete([hd hp hl]); end
        try, B.mCADplot; P.mCADplot; catch, end
        hold on;
        if ki>0
            hd = plot3(log.x_drone(1:ki,1),log.x_drone(1:ki,2),log.x_drone(1:ki,3),'b-','LineWidth',1.2);
            hp = plot3(log.x_pio(1:ki,1),  log.x_pio(1:ki,2),  zeros(ki,1),'r-','LineWidth',1.2);
            hl = plot3([B.pPos.X(1) P.pPos.X(1)],[B.pPos.X(2) P.pPos.X(2)],[B.pPos.X(3) 0],'k:','LineWidth',1);
        end
        axis(limites); grid on;
        title(sprintf('Time: %05.2fs | rho=%.2fm', toc(t), log.rho(max(ki,1))));
        drawnow;
    end
end

%% ---------- 6. ENCERRAMENTO ----------
B.pSC.Ud = [0;0;0;0]; try, B.rSendControlSignals; catch, end
P.pSC.Ud = [0;0];     try, P.rSendControlSignals; catch, end

% Trunca log
campos = {'t','u_drone','x_drone','u_pio','x_pio','rho','psi_err'};
for c=1:numel(campos), log.(campos{c}) = log.(campos{c})(1:ki,:); end

LOG_FILE = sprintf('log_voo_%s.mat', datestr(now,'yyyymmdd_HHMMSS'));
save(LOG_FILE,'log');
fprintf('Simulacao encerrada (%d passos). Log: %s\n', ki, LOG_FILE);

disp('Rodando analise...');
analise_logs(log);

%% ===================== CALLBACKS DE TECLADO =====================
function keyDown(~, event)
    global keyState
    switch event.Key
        case 'uparrow',    keyState.up = true;
        case 'downarrow',  keyState.down = true;
        case 'leftarrow',  keyState.left = true;
        case 'rightarrow', keyState.right = true;
        case 'w', keyState.w = true;
        case 's', keyState.s = true;
        case 'a', keyState.a = true;
        case 'd', keyState.d = true;
        case 'space', keyState.space = true;
        case 'escape', keyState.abort = true;
    end
end

function keyUp(~, event)
    global keyState
    switch event.Key
        case 'uparrow',    keyState.up = false;
        case 'downarrow',  keyState.down = false;
        case 'leftarrow',  keyState.left = false;
        case 'rightarrow', keyState.right = false;
        case 'w', keyState.w = false;
        case 's', keyState.s = false;
        case 'a', keyState.a = false;
        case 'd', keyState.d = false;
        case 'space', keyState.space = false;
    end
end