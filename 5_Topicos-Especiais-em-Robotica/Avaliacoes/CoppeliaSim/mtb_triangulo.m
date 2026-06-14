%% =====================================================================
%  mtb_triangulo.m
%  CoppeliaSim + MATLAB (ZMQ Remote API)
%  Manipulador MTB (SCARA: 2 juntas rotativas + 1 prismatica) leva o
%  efetuador a 3 posicoes que formam um TRIANGULO EQUILATERO no espaco.
%
%  Estrategia robusta:
%   1. Mede L1, L2 lendo as posicoes das juntas (nao depende da cena).
%   2. Calibra a direcao de referencia (q1=q2=0).
%   3. Calcula cinematica inversa do SCARA para cada vertice.
%   4. Comanda as juntas, le o TCP real, verifica o triangulo.
%
%  [Inferencia] axis1/axis2 = rotativas (definem x,y); axis3 = prismatica
%  (z, mantida fixa); axis4 = TCP. Confirme na cena se algo destoar.
%
%  COMO USAR:
%   1. Abra "MTB.ttt" (pasta Ambiente de Simulacao) no CoppeliaSim.
%   2. Garanta o add-on "ZMQ remote API" ativo.
%   3. Rode este script no MATLAB.
% =====================================================================

clc; clear; close all;

%% ---------- Path ----------
FolderCurrent = which(mfilename);
key = [filesep 'ELT 539'];
idx = strfind(FolderCurrent, key);
if isempty(idx), addpath(genpath(pwd));
else, addpath(genpath(FolderCurrent(1:idx(end)+numel(key)-1))); end

%% ---------- Conexao ----------
fprintf('Conectando ao CoppeliaSim...\n');
client = RemoteAPIClient();
sim = client.getObject('sim');
sim.startSimulation();
pause(0.5);

axis1 = sim.getObject('/MTB/axis1');
axis2 = sim.getObject('/MTB/axis2');
axis3 = sim.getObject('/MTB/axis3');
tcp   = sim.getObject('/MTB/axis4');
fprintf('[OK] Eixos do MTB obtidos.\n');

%% ---------- 1. Medir geometria (L1, L2) ----------
p1 = getpos(sim, axis1);
p2 = getpos(sim, axis2);
p4 = getpos(sim, tcp);
base = p1(1:2);
L1 = hypot(p2(1)-p1(1), p2(2)-p1(2));
L2 = hypot(p4(1)-p2(1), p4(2)-p2(2));
q3_hold = getjoint(sim, axis3);     % mantem a prismatica fixa
fprintf('Geometria medida: L1=%.3f m, L2=%.3f m, base=(%.2f,%.2f)\n', ...
        L1, L2, base(1), base(2));

%% ---------- 2. Calibracao: direcao de referencia (q1=q2=0) ----------
moveJoints(sim, axis1, axis2, axis3, 0, 0, q3_hold);
settle(sim, axis1, axis2, [0 0], 4);
p_cal = getpos(sim, tcp);
theta_ref = atan2(p_cal(2)-base(2), p_cal(1)-base(1));
z_plano = p_cal(3);
fprintf('Calibracao: theta_ref=%.1f deg, z do plano=%.3f m\n', ...
        rad2deg(theta_ref), z_plano);

%% ---------- 3. Triangulo equilatero (no frame da base) ----------
reach = L1 + L2;
r_c   = 0.55*reach;        % distancia do centro do triangulo a base
R_tri = 0.22*reach;        % circunraio do triangulo
ang   = deg2rad([90 210 330]);

% Vertices no frame da base (x = direcao theta_ref)
Vb = [r_c + R_tri*cos(ang); R_tri*sin(ang)]';   % 3x2

% Auto-ajuste de alcancabilidade
rmin = abs(L1-L2) + 0.02;  rmax = reach - 0.02;
for tent = 1:8
    rr = hypot(Vb(:,1), Vb(:,2));
    if all(rr > rmin & rr < rmax), break; end
    R_tri = 0.85*R_tri; r_c = 0.9*r_c;     % encolhe e re-centra
    Vb = [r_c + R_tri*cos(ang); R_tri*sin(ang)]';
end
fprintf('Triangulo: r_centro=%.3f m, R=%.3f m\n', r_c, R_tri);

%% ---------- 4. Visitar cada vertice (IK -> comando -> leitura) ----------
TCPreal = nan(3,3);
for k = 1:3
    [q1, q2, ok] = ik_scara(Vb(k,1), Vb(k,2), L1, L2);
    if ~ok, fprintf('[!] Vertice %d fora do alcance.\n', k-1); end
    fprintf('\n-> Vertice %d: q1=%.1f deg, q2=%.1f deg\n', k-1, rad2deg(q1), rad2deg(q2));
    moveJoints(sim, axis1, axis2, axis3, q1, q2, q3_hold);
    settle(sim, axis1, axis2, [q1 q2], 5);
    TCPreal(k,:) = getpos(sim, tcp);
    fprintf('   TCP real: (%.3f, %.3f, %.3f)\n', TCPreal(k,1), TCPreal(k,2), TCPreal(k,3));
end

sim.stopSimulation();
fprintf('\nSimulacao finalizada.\n');

%% ---------- VERIFICACAO ----------
fprintf('\n========= VERIFICACAO DO TRIANGULO (MTB) =========\n');
% Alvo em coordenadas do mundo: base + rotacao(theta_ref)*Vb
Rrot = [cos(theta_ref) -sin(theta_ref); sin(theta_ref) cos(theta_ref)];
alvo_w = (base(:) + Rrot*Vb.').';        % 3x2 (xy no mundo)

d = @(a,b) norm(a-b);
L = [d(TCPreal(1,1:2),TCPreal(2,1:2)), ...
     d(TCPreal(2,1:2),TCPreal(3,1:2)), ...
     d(TCPreal(3,1:2),TCPreal(1,1:2))];
Lmed = mean(L);  desvio = max(abs(L-Lmed))/Lmed*100;

for k = 1:3
    err = hypot(TCPreal(k,1)-alvo_w(k,1), TCPreal(k,2)-alvo_w(k,2));
    st = '[OK]'; if err > 0.03, st = '[!]'; end
    fprintf('  %s Vertice %d: alvo (%.3f,%.3f) | real (%.3f,%.3f) | erro %.3f m\n', ...
        st, k-1, alvo_w(k,1), alvo_w(k,2), TCPreal(k,1), TCPreal(k,2), err);
end
fprintf('  Lados: %.3f, %.3f, %.3f m  (media %.3f m)\n', L(1), L(2), L(3), Lmed);
fprintf('  Desvio maximo entre lados: %.1f%%\n', desvio);
if desvio < 8
    fprintf('  RESULTADO: TRIANGULO EQUILATERO COMPLETADO COM SUCESSO\n');
else
    fprintf('  RESULTADO: triangulo com desvios (ver acima)\n');
end
fprintf('==================================================\n');

%% ---------- Grafico ----------
figure('Name','MTB - Triangulo','Color','w');
plot([TCPreal(:,1); TCPreal(1,1)], [TCPreal(:,2); TCPreal(1,2)], ...
     'b-o', 'LineWidth', 1.5, 'MarkerFaceColor','b'); hold on;
plot([alvo_w(:,1); alvo_w(1,1)], [alvo_w(:,2); alvo_w(1,2)], ...
     'r--s', 'LineWidth', 1.2, 'MarkerSize', 10);
plot(base(1), base(2), 'k^', 'MarkerSize', 12, 'MarkerFaceColor','k');
for k = 1:3, text(TCPreal(k,1)+0.02, TCPreal(k,2), sprintf('V%d',k-1)); end
axis equal; grid on; xlabel('x [m]'); ylabel('y [m]');
title('TCP do MTB: triangulo real vs desejado (vista superior)');
legend({'TCP real','Triangulo alvo','Base'}, 'Location','best');

%% ===================== FUNCOES AUXILIARES =====================
function p = getpos(sim, h)
    p = sim.getObjectPosition(h, -1);
    if iscell(p), p = cell2mat(p); end
    p = p(:).';
end

function q = getjoint(sim, h)
    q = sim.getJointPosition(h);
    if iscell(q), q = cell2mat(q); end
    q = q(1);
end

function moveJoints(sim, a1, a2, a3, q1, q2, q3)
    sim.setJointTargetPosition(a1, q1);
    sim.setJointTargetPosition(a2, q2);
    sim.setJointTargetPosition(a3, q3);
end

function settle(sim, a1, a2, qd, tmax)
    % Espera as juntas rotativas estabilizarem perto do alvo (ou timeout)
    t = tic;
    while toc(t) < tmax
        e1 = abs(angdiff(getjoint(sim,a1), qd(1)));
        e2 = abs(angdiff(getjoint(sim,a2), qd(2)));
        if e1 < deg2rad(1.5) && e2 < deg2rad(1.5), break; end
        pause(0.05);
    end
    pause(0.3);   % margem extra de acomodacao
end

function dd = angdiff(a, b)
    dd = atan2(sin(a-b), cos(a-b));
end

function [q1, q2, ok] = ik_scara(x, y, L1, L2)
    % Cinematica inversa do SCARA planar (2 elos), cotovelo "para cima"
    r2 = x^2 + y^2;
    c2 = (r2 - L1^2 - L2^2)/(2*L1*L2);
    ok = abs(c2) <= 1;
    c2 = max(min(c2,1),-1);
    q2 = atan2(sqrt(1-c2^2), c2);
    q1 = atan2(y,x) - atan2(L2*sin(q2), L1 + L2*cos(q2));
end
