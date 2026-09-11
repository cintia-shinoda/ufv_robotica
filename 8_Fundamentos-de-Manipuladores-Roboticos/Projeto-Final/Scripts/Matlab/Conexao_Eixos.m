% Conexão como no exemplo
sim = remApi('remoteApi');
sim.simxFinish(-1);
clientID = sim.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID > -1)
    disp('Connected to remote API server');
    
    % Configuração inicial
    sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot_wait);
    [~,axis1] = sim.simxGetObjectHandle(clientID,'/Manipulador/axis1',sim.simx_opmode_blocking);
    [~,axis2] = sim.simxGetObjectHandle(clientID,'/Manipulador/axis2',sim.simx_opmode_blocking);
    [~,axis3] = sim.simxGetObjectHandle(clientID,'/Manipulador/axis3',sim.simx_opmode_blocking);
    [~,tcp] = sim.simxGetObjectHandle(clientID,'/Manipulador/axis4',sim.simx_opmode_blocking);

    % Seu Código Aqui
end