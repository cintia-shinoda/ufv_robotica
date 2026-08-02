% =====================================================================
% Sinais de controle (pSC.Ud), ja saturados por tanh em (-1,+1):
%   Ud(1) = phi   (rolagem)
%   Ud(2) = theta (arfagem)
%   Ud(3) = dZ    (propulsao / velocidade vertical)
%   Ud(4) = dPsi  (guinada)
% =====================================================================
clearvars; close all; clc

if isempty(which('ArDrone'))
    d = fileparts(which(mfilename)); k = 'AuRoRA'; i = strfind(d,k);
    if ~isempty(i)
        addpath(genpath(d(1:i(end)+numel(k)-1)));
    else
        error('Adicione a pasta AuRoRA ao path (Home > Set Path) e rode de novo.');
    end
end

A = ArDrone;                              % cria o drone (simulacao)

% ---- Estado inicial: no solo, sob a Base de Recarga ----
A.pPos.X([1 2 3 6]) = [0 -5 0 90*pi/180]';
A.pPos.Xc = A.pPos.X;                     % espelha estado interno
A.pPos.Xd  = zeros(12,1);                 % pose/vel desejada
A.pPos.dXd = zeros(12,1);                 % aceleracao desejada (usada pelo controlador)
A.pSC.Ur   = zeros(4,1);                  % controle cinematico anterior (seguranca)
% A.rTakeOff;                             % descomente se nao decolar

% ---- Missao: waypoints [x y z psi(graus)] ----
WP = [  0 -5 0   90;    % 1 solo (inicio)
        0 -5 1   90;    % 2 decolagem / hover na Base
       -3  0 3  -90;    % 3 Baia A
       -5  3 2    0;    % 4 Baia B
        5  2 1  180;    % 5 Baia C
        3 -5 3   90;    % 6 Baia D
        0 -5 1   90;    % 7 retorno a Base
        0 -5 0   90];   % 8 pouso
segT  = [3 7 5 11 8 4.5 3];   % duracao de cada trecho (s) - mais lento => menos overshoot
holdT = [0 0 2 2 2 2 0 0];    % hover de inspecao ao chegar (s)

% guinada: usa os alvos como estao (todos os saltos consecutivos sao <=180 graus,
% entao o min-jerk interpola pelo caminho curto e o angulo fica em [-180,180]).
% (Para waypoints com salto >180 graus, seria preciso ajuste de caminho curto por trecho.)

[segs,Ttot] = buildSchedule(WP,segT,holdT);

% ---- Figura de navegacao (animacao) ----
figure(1); A.mCADplot; hold on; grid on
axis([-6 6 -6 6 0 4]); view(-60,22)
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]'); title('Missao de inspecao - ArDrone')
Rc = plot3(A.pPos.X(1),A.pPos.X(2),A.pPos.X(3),'-','Color',[.12 .31 .47],'LineWidth',1.5);
for i=3:6   % baias A,B,C,D
    plot3(WP(i,1),WP(i,2),WP(i,3),'o','Color',[.75 .22 .17],'MarkerFaceColor',[.75 .22 .17],'MarkerSize',7);
end

XX = [];
t = tic; tc = tic;
while toc(t) < Ttot
    if toc(tc) > A.pPar.Ts               % passo de amostragem do proprio drone (1/30 s)
        tc = tic;
        tt = toc(t);

        A.rGetSensorData;                % atualiza A.pPos.X
        Xc = A.pPos.X(1:12)';            % estado atual (antes de aplicar controle)

        % ---- Referencia (trajetoria min-jerk, com feedforward) ----
        [pd,vd,ad,psid,dpsid,ddpsid] = refTraj(tt,segs);
        A.pPos.Xd(1:3) = pd;             % posicao desejada
        A.pPos.Xd(6)   = psid;           % guinada desejada
        A.pPos.Xd(7:9) = vd;             % feedforward de velocidade linear
        A.pPos.Xd(12)  = dpsid;          % feedforward de velocidade de guinada
        A.pPos.dXd(7:9)= ad;             % feedforward de aceleracao linear
        A.pPos.dXd(12) = ddpsid;         % feedforward de aceleracao de guinada
        Xdc = A.pPos.Xd(1:12)';

        % ---- Controlador embutido do ArDrone (ganhos default) ----
        A.cArDrone_InverseDynamic_wDynamicCompensator;
        Ud = A.pSC.Ud(1:4)';             % CAPTURA o comando AGORA (antes do send zerar)

        % armazena ANTES do send: [ Xd(1:12)  X(1:12)  Ud(1:4)  t ]
        XX = [XX; Xdc  Xc  Ud  tt];      %#ok<AGROW>

        A.rSendControlSignals;           % aplica controle e avanca a simulacao
        A.mCADplot;

        Rc.XData=[Rc.XData A.pPos.X(1)];
        Rc.YData=[Rc.YData A.pPos.X(2)];
        Rc.ZData=[Rc.ZData A.pPos.X(3)];
        drawnow
    end
end

% =====================================================================
%                    GRAFICOS (Q5 e Q6)
% =====================================================================
tv=XX(:,29);
xr=XX(:,1);  yr=XX(:,2);  zr=XX(:,3);  psir=XX(:,6)*180/pi;    % referencias
x =XX(:,13); y =XX(:,14); z =XX(:,15); psi =XX(:,18)*180/pi;   % reais (ja limitados a [-180,180])
Uphi=XX(:,25); Utheta=XX(:,26); Uz=XX(:,27); Upsi=XX(:,28);    % Ud = [phi theta dZ dpsi]
navy=[.12 .31 .47]; red=[.75 .22 .17]; grn=[.18 .49 .20]; org=[.88 .54 0];

% Q5 - navegacao 3D
figure(2); plot3(x,y,z,'Color',navy,'LineWidth',2); grid on; hold on
for i=3:6, plot3(WP(i,1),WP(i,2),WP(i,3),'o','Color',red,'MarkerFaceColor',red,'MarkerSize',7); end
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]'); title('Navegacao 3D (XYZ)'); view(-60,22)

% Q5 - series temporais
serie(tv,xr,x,'X [m]','Evolucao temporal de X',navy,red);
serie(tv,yr,y,'Y [m]','Evolucao temporal de Y',navy,red);
serie(tv,zr,z,'Z [m]','Evolucao temporal de Z',navy,red);
serie(tv,psir,psi,'\psi [graus]','Evolucao temporal de \psi',navy,red);

% Q6 - propulsao vertical  (Ud(3) = comando de velocidade vertical)
figure(7); hold on; grid on
plot(tv([1 end]),[ 1  1],':','Color',[.6 .6 .6]);
plot(tv([1 end]),[-1 -1],':','Color',[.6 .6 .6]);
plot(tv,Uz,'Color',grn,'LineWidth',2);
ylim([-1.05 1.05]); xlabel('Tempo [s]'); ylabel('Sinal normalizado');
title('Sinal de controle - Propulsao vertical'); legend('u_z (dZ)')

% Q6 - "torques" do corpo (comandos de atitude phi/theta/psi)
figure(8); hold on; grid on
plot(tv([1 end]),[ 1  1],':','Color',[.6 .6 .6]);
plot(tv([1 end]),[-1 -1],':','Color',[.6 .6 .6]);
plot(tv,Uphi ,'Color',red ,'LineWidth',1.6);
plot(tv,Utheta,'Color',navy,'LineWidth',1.6);
plot(tv,Upsi ,'Color',org ,'LineWidth',1.6);
ylim([-1.05 1.05]); xlabel('Tempo [s]'); ylabel('Sinal normalizado');
title('Sinais de controle - Comandos de atitude (corpo)');
legend('u_\phi (rolagem)','u_\theta (arfagem)','u_\psi (guinada)')

% ---- salvar PNG (descomente) ----
% for f=2:8, saveas(figure(f),sprintf('estufa_fig%d.png',f)); end

% =====================================================================
%                    FUNCOES LOCAIS
% =====================================================================
function [segs,Ttot]=buildSchedule(WP,segT,holdT)
    P=WP(:,1:3); ps=WP(:,4)*pi/180;
    segs=struct('t0',{},'T',{},'p0',{},'p1',{},'ps0',{},'ps1',{});
    tacc=0; n=size(WP,1);
    for i=1:n-1
        segs(end+1)=struct('t0',tacc,'T',segT(i),'p0',P(i,:)','p1',P(i+1,:)',...
                           'ps0',ps(i),'ps1',ps(i+1)); %#ok<AGROW>
        tacc=tacc+segT(i);
        if numel(holdT)>=i+1 && holdT(i+1)>0
            segs(end+1)=struct('t0',tacc,'T',holdT(i+1),'p0',P(i+1,:)','p1',P(i+1,:)',...
                               'ps0',ps(i+1),'ps1',ps(i+1)); %#ok<AGROW>
            tacc=tacc+holdT(i+1);
        end
    end
    Ttot=tacc;
end

function [pd,vd,ad,psid,dpsid,ddpsid]=refTraj(tt,segs)
    for i=1:numel(segs)
        s=segs(i);
        if tt>=s.t0 && tt<=s.t0+s.T+1e-9
            [pd,vd,ad]=minjerk(s.p0,s.p1,s.T,tt-s.t0);
            [pp,pv,pa]=minjerk(s.ps0,s.ps1,s.T,tt-s.t0);
            psid=pp; dpsid=pv; ddpsid=pa; return;
        end
    end
    s=segs(end); pd=s.p1; vd=zeros(3,1); ad=zeros(3,1); psid=s.ps1; dpsid=0; ddpsid=0;
end

function [pos,vel,acc]=minjerk(p0,p1,T,tt)
    if T<=0, pos=p1; vel=zeros(size(p1)); acc=zeros(size(p1)); return; end
    s=min(max(tt/T,0),1);
    pos=p0+(p1-p0).*(10*s^3-15*s^4+6*s^5);
    vel=(p1-p0).*(30*s^2-60*s^3+30*s^4)/T;
    acc=(p1-p0).*(60*s-180*s^2+120*s^3)/T^2;
end

function serie(tv,ref,act,ylab,ttl,cact,cref)
    figure; hold on; grid on
    plot(tv,ref,'--','Color',cref,'LineWidth',2);
    plot(tv,act,'Color',cact,'LineWidth',1.8);
    xlabel('Tempo [s]'); ylabel(ylab); title(ttl);
    legend('Referencia','Real','Location','best')
end
