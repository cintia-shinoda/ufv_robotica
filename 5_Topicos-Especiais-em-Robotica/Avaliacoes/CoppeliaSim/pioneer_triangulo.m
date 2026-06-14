%% =====================================================================
%  pioneer_triangulo.m
%  CoppeliaSim + MATLAB (ZMQ Remote API)
%  Pioneer 3-DX percorre um TRIANGULO EQUILATERO no plano x-y.
%
%  Baseado no script "pioneer.m" da disciplina (mesma conexao,
%  mesmos handles, mesmo controle e conversao de rodas). A unica
%  mudanca: referencia agora sao 3 vertices de um triangulo
%  equilatero, com avanco por chegada e verificacao final.
%
%  COMO USAR:
%   1. Abra "Pioneer.ttt" (pasta Ambiente de Simulacao) no CoppeliaSim.
%   2. Garanta o add-on "ZMQ remote API" ativo (vem por padrao no 4.x).
%   3. Rode este script no MATLAB (instalado localmente, nao Online).
% =====================================================================

close all; clear; clc;

%% ---------- Path (cross-platform: Windows '\' ou Mac/Linux '/') ----
FolderCurrent = which(mfilename);
key = [filesep 'ELT 539'];
idx = strfind(FolderCurrent, key);
if isempty(idx)
    addpath(genpath(pwd));   % fallback: pasta atual
else
    FolderRoot = FolderCurrent(1:idx(end)+numel(key)-1);
    addpath(genpath(FolderRoot));
end

%% ---------- Classe AuRoRA (para guardar o estado) ----------
P = Pioneer3DX;

% Guarda de seguranca: o controle usa P.pPar.a (ponto fora do eixo).
% Se for 0, a matriz A fica singular e o robo nao controla y.
if ~isprop(P,'pPar') || ~isfield(P.pPar,'a') || P.pPar.a == 0
    P.pPar.a = 0.15;
    fprintf('[Aviso] P.pPar.a era 0/indefinido; usando 0.15 m.\n');
end

%% ---------- Parametros do triangulo ----------
cx = 0.0;  cy = 0.0;       % centro do triangulo [m]
R  = 1.5;                  % circunraio [m] (mesma escala do exemplo)
ang = deg2rad([90, 210, 330]);   % 3 vertices a 120 graus
WP = [cx + R*cos(ang); cy + R*sin(ang)]';   % 3x2: cada linha = [x y]
WP = [WP; WP(1,:)];        % fecha o triangulo (volta ao vertice 0)

tol_chegada = 0.12;        % raio de chegada a um vertice [m]
t_por_wp    = 35;          % tempo maximo por vertice [s]
Kd = diag([0.20, 0.30]);   % ganhos do controlador (ajustaveis)

%% ---------- Conexao ----------
fprintf('Conectando ao CoppeliaSim...\n');
client = RemoteAPIClient();
sim = client.getObject('sim');
sim.startSimulation();
pause(0.5);

robot      = sim.getObject('/PioneerP3DX');
leftMotor  = sim.getObject('/PioneerP3DX/leftMotor');
rightMotor = sim.getObject('/PioneerP3DX/rightMotor');
fprintf('Simulacao iniciada. Triangulo: R=%.2f m, centro (%.1f,%.1f).\n', R, cx, cy);

%% ---------- Log ----------
LOG.t = []; LOG.x = []; LOG.y = []; LOG.v = []; LOG.w = []; LOG.wp = [];
atingidos = nan(3,2);      % posicao real ao atingir cada vertice

%% ---------- Loop por waypoint ----------
t_total = tic;
for iwp = 1:size(WP,1)
    xd = WP(iwp,1);  yd = WP(iwp,2);
    fprintf('\n-> Indo ao vertice %d: (%.2f, %.2f)\n', mod(iwp-1,3), xd, yd);
    t_wp = tic; tc = tic; chegou = false;

    while toc(t_wp) < t_por_wp
        if toc(tc) > 1/10
            tc = tic;

            % ----- Leitura da pose -----
            pose = sim.getObjectPose(robot, -1);
            if iscell(pose), pose = cell2mat(pose); end
            pos  = pose(1:3);  quat = pose(4:7);
            yaw  = atan2(2*(quat(4)*quat(3)), 1 - 2*(quat(3)^2));

            Xa = P.pPos.X;
            P.pPos.X(1:3) = pos(:);
            P.pPos.X(6)   = yaw;
            P.pPos.X(7:12)= (P.pPos.X(1:6) - Xa(1:6))/P.pPar.Ts;

            % ----- Referencia -----
            P.pPos.Xd(1) = xd;  P.pPos.Xd(2) = yd;

            % ----- Controle (feedback linearization, igual disciplina) -----
            psi = P.pPos.X(6);
            A = [cos(psi), -P.pPar.a*sin(psi);
                 sin(psi),  P.pPar.a*cos(psi)];
            P.pPos.Xtil = P.pPos.Xd - P.pPos.X;
            P.pSC.Ud = pinv(A) * (P.pPos.Xd([7 8]) + Kd * P.pPos.Xtil([1 2]));

            v = P.pSC.Ud(1);  w = P.pSC.Ud(2);

            % ----- Conversao para rodas (igual Pioneer3DX.py) -----
            vel_left  = (v - w*0.331/2)/0.0975;
            vel_right = (v + w*0.331/2)/0.0975;
            max_w = 2.4;
            vel_left  = max(min(vel_left,  max_w), -max_w);
            vel_right = max(min(vel_right, max_w), -max_w);
            sim.setJointTargetVelocity(leftMotor,  vel_left);
            sim.setJointTargetVelocity(rightMotor, vel_right);

            % ----- Log -----
            LOG.t(end+1)  = toc(t_total);
            LOG.x(end+1)  = P.pPos.X(1);
            LOG.y(end+1)  = P.pPos.X(2);
            LOG.v(end+1)  = v;
            LOG.w(end+1)  = w;
            LOG.wp(end+1) = iwp;

            % ----- Chegou? -----
            rho = hypot(xd - P.pPos.X(1), yd - P.pPos.X(2));
            if rho < tol_chegada
                chegou = true;
                if iwp <= 3, atingidos(iwp,:) = [P.pPos.X(1) P.pPos.X(2)]; end
                fprintf('   chegou (erro %.3f m, %.1fs)\n', rho, toc(t_wp));
                break;
            end
        end
    end

    if ~chegou
        fprintf('   [!] tempo esgotado sem atingir (erro %.3f m)\n', rho);
        if iwp <= 3, atingidos(iwp,:) = [P.pPos.X(1) P.pPos.X(2)]; end
    end
end

% Para o robo
sim.setJointTargetVelocity(leftMotor, 0);
sim.setJointTargetVelocity(rightMotor, 0);
sim.stopSimulation();
fprintf('\nSimulacao finalizada.\n');

%% ---------- VERIFICACAO ----------
fprintf('\n========= VERIFICACAO DO TRIANGULO (Pioneer) =========\n');
alvo = WP(1:3,:);
err_v = sqrt(sum((atingidos - alvo).^2, 2));     % erro por vertice
d = @(a,b) hypot(a(1)-b(1), a(2)-b(2));
L = [d(atingidos(1,:),atingidos(2,:)), ...
     d(atingidos(2,:),atingidos(3,:)), ...
     d(atingidos(3,:),atingidos(1,:))];
Lmed = mean(L);  desvio = max(abs(L - Lmed))/Lmed*100;

for k = 1:3
    st = '[OK]'; if err_v(k) > tol_chegada, st = '[!]'; end
    fprintf('  %s Vertice %d: alvo (%.2f,%.2f) | real (%.2f,%.2f) | erro %.3f m\n', ...
        st, k-1, alvo(k,1), alvo(k,2), atingidos(k,1), atingidos(k,2), err_v(k));
end
fprintf('  Lados: %.3f, %.3f, %.3f m  (media %.3f m)\n', L(1), L(2), L(3), Lmed);
fprintf('  Desvio maximo entre lados: %.1f%%\n', desvio);
if all(err_v <= tol_chegada) && desvio < 8
    fprintf('  RESULTADO: TRIANGULO EQUILATERO COMPLETADO COM SUCESSO\n');
else
    fprintf('  RESULTADO: trajetoria com desvios (ver acima)\n');
end
fprintf('======================================================\n');

%% ---------- Grafico ----------
figure('Name','Pioneer - Triangulo','Color','w');
plot(LOG.x, LOG.y, 'b-', 'LineWidth', 1.5); hold on;
plot([alvo(:,1); alvo(1,1)], [alvo(:,2); alvo(1,2)], 'r--o', ...
     'LineWidth', 1.2, 'MarkerSize', 8, 'MarkerFaceColor', 'r');
plot(atingidos(:,1), atingidos(:,2), 'gs', 'MarkerSize', 12, 'LineWidth', 1.5);
for k = 1:3, text(alvo(k,1)+0.1, alvo(k,2), sprintf('V%d', k-1)); end
axis equal; grid on;
xlabel('x [m]'); ylabel('y [m]');
title('Trajetoria do Pioneer vs triangulo desejado');
legend({'Trajetoria real','Triangulo alvo','Vertices atingidos'}, 'Location','best');
