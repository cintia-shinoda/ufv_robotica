clearvars; close all; clc;

%% Conecta ao Coppelia
sim = remApi('remoteApi');
sim.simxFinish(-1);

clientID = -1;
for attempt = 1:20
    clientID = sim.simxStart('127.0.0.1',19997,true,true,5000,5);
    if clientID > -1
        fprintf('[SUCCESS] Conectado ao CoppeliaSim\n');
        break;
    else
        fprintf('[WAITING] Tentando conectar... (%d/20)\n', attempt);
        pause(1);
    end
end

if clientID == -1
    error('Não foi possível conectar ao CoppeliaSim.');
end

%% Solicita matrícula
matricula = input('Digite sua Matrícula: ', 's');
matricula = pad(matricula, 6, 'left', '0');
R = str2double(matricula(1));
S = str2double(matricula(2));
T = str2double(matricula(3));
X = str2double(matricula(4));
Y = str2double(matricula(5));
Z = str2double(matricula(6));

% Seleção da cena
modulo = mod(str2double(matricula), 3);
if modulo == 0
    scene = 'Scara.ttt';
    manipulador = 'SCARA';
elseif modulo == 1
    scene = 'Cilindrico.ttt';
    manipulador = 'CILÍNDRICO';
else
    scene = 'Esferico.ttt';
    manipulador = 'ESFÉRICO';
end

% Carrega a cena
sim.simxLoadScene(clientID, which(scene), 0, sim.simx_opmode_blocking);
fprintf('[SUCCESS] Manipulador %s carregado com sucesso.\n', manipulador);

% Estratégia
if mod(Z, 2) == 0
    estrategia = 'bang-bang';
else
    estrategia = 'polinomial';
end

% Define pontos A e B
partida = [-0.5, -0.5, 0.5];
chegada = [0.5, 0.5, 1.5];
ajusteA = arrayfun(@(d) 0.25 * (1 - 2 * mod(d, 2)), [R S T]);
ajusteB = arrayfun(@(d) 0.25 * (1 - 2 * mod(d, 2)), [X Y Z]);
pontoA = partida + ajusteA;
pontoB = chegada + ajusteB;

% Seleção da tarefa
fprintf('\nArea de trabalho = 0  |  Cinemática Direta = 1  |  Cinemática Inversa = 2\n');
tarefa = input('Digite o número referente à tarefa: ');
while ~ismember(tarefa, [0 1 2])
    tarefa = input('Número inválido. Digite novamente: ');
end

%% Inicia a simulação
sim.simxStartSimulation(clientID, sim.simx_opmode_blocking);
fprintf('[INFO] Simulação iniciada.\n');

% Handles dos eixos e TCP
[~, h1] = sim.simxGetObjectHandle(clientID, '/Manipulador/axis1', sim.simx_opmode_blocking);
[~, h2] = sim.simxGetObjectHandle(clientID, '/Manipulador/axis2', sim.simx_opmode_blocking);
[~, h3] = sim.simxGetObjectHandle(clientID, '/Manipulador/axis3', sim.simx_opmode_blocking);
[~, tcp] = sim.simxGetObjectHandle(clientID, '/Manipulador/axis4', sim.simx_opmode_blocking);
fprintf('[FOUND] Todos os eixos do manipulador %s foram encontrados\n', manipulador);

% --- Função auxiliar para registrar pose
function pose = obterPose(sim, clientID, tcp)
    [~, pose] = sim.simxGetObjectPosition(clientID, tcp, -1, sim.simx_opmode_blocking);
end

function registrar()
    pose = obterPose(sim, clientID, tcp);
    posX(end+1) = pose(1);
    posY(end+1) = pose(2);
    posZ(end+1) = pose(3);
end

if tarefa == 0
    % --- Tarefa 0: Varredura da área de trabalho
    fprintf('[TASK] Varredura da área de trabalho\n');
    poses = [];

  

    % Plot
    figure;
    scatter3(poses(:,1), poses(:,2), poses(:,3), 10, 'filled');
    title('Área de Trabalho do Manipulador');
    xlabel('X'); ylabel('Y'); zlabel('Z');
    grid on;
    axis equal;

elseif tarefa == 1
    % --- Tarefa 1: Cinemática direta
    fprintf('[TASK] Cinemática Direta\n');
    posX = [];
    posY = [];
    posZ = [];

    % Sequência de posições
    sim.simxSetJointTargetPosition(clientID, h1, 0, sim.simx_opmode_blocking);
    sim.simxSetJointTargetPosition(clientID, h2, 0, sim.simx_opmode_blocking);
    sim.simxSetJointTargetPosition(clientID, h3, 0, sim.simx_opmode_blocking);
    pause(1); registrar();

    % Movimento 1
    % pause(1); registrar();

    % Movimento 2
    % pause(1); registrar();

    % Movimento 3
    % pause(1); registrar();

    % Retorna
    sim.simxSetJointTargetPosition(clientID, h1, 0, sim.simx_opmode_blocking);
    sim.simxSetJointTargetPosition(clientID, h2, 0, sim.simx_opmode_blocking);
    sim.simxSetJointTargetPosition(clientID, h3, 0, sim.simx_opmode_blocking);
    pause(1); registrar();

    figure;
    plot3(posX, posY, posZ, 'k--o', 'LineWidth', 2);
    title('Cinemática Direta - Caminho do TCP');
    xlabel('X'); ylabel('Y'); zlabel('Z');
    grid on;

else
    % --- Tarefa 2: Cinemática inversa (trajetória)
    fprintf('[TASK] Cinemática Inversa - %s\n', estrategia);
end

% Finaliza a simulação
sim.simxStopSimulation(clientID, sim.simx_opmode_blocking);
fprintf('[INFO] Simulação finalizada.\n');
sim.simxFinish(clientID);
sim.delete();
