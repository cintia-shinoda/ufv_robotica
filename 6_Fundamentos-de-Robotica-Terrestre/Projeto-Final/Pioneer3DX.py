import time
import numpy as np
import subprocess
import psutil
import os
from scipy.spatial.transform import Rotation as R
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

class pPos():
    def __init__(self):
        self.X = np.array([0, 0, 0.13879, 0, 0, 0, 0, 0, 0, 0, 0, 0]).reshape(-1, 1)
        self.Xd = np.array([0, 0, 0.13879, 0, 0, 0, 0, 0, 0, 0, 0, 0]).reshape(-1, 1)
        self.Xtil = np.zeros((12, 1))
        self.Xa = np.zeros((12, 1))
        self.Xda = np.zeros((12, 1))

class pPar():
    def __init__(self):
        self.model = "P3DX"
        self.a = 0.15
        self.Ts = 0.1
        self.g = 9.8
        self.m = 0.429
        self.alpha = 0
        self.theta = np.array([0.5338, 0.2168, -0.0134, 0.9560, -0.0843, 1.0590]).reshape(-1, 1)

class pSC():
    def __init__(self):
        self.U = np.zeros((2, 1))
        self.Ud = np.zeros((2, 1))

class pFlag():
    def __init__(self):
        self.Connected = 0
        self.JoyON = 0
        self.GPS = 0
        self.EmergencyStop = 0
        self.Opened = 0
        self.Scene = 0
        self.TrailON = False

class Pioneer3DX():
    def __init__(self):
        self.sim = None
        self.client = None
        self.pPos = pPos()
        self.pPar = pPar()
        self.pSC = pSC()
        self.pFlag = pFlag()

    def rConnect(self):
        for proc in psutil.process_iter(['name']):
            if proc.info['name'] and 'coppeliasim' in proc.info['name'].lower():
                self.pFlag.Opened = 1

        if self.pFlag.Opened == 0:
            subprocess.Popen([
                "powershell.exe",
                "-Command",
                "Start-Process",
                r"'C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\coppeliaSim.exe'"
            ], creationflags=subprocess.CREATE_NEW_CONSOLE)
            self.pFlag.Opened = 1

        for attempt in range(20):
            try:
                self.client = RemoteAPIClient()
                self.sim = self.client.getObject('sim')
                print(f"[SUCCESS] Conectado ao CoppeliaSim")
                break
            except Exception:
                print(f"[WAITING] Tentando conectar... ({attempt + 1}/20)")
                time.sleep(1)
        else:
            raise TimeoutError("Não foi possível conectar ao CoppeliaSim.")
    
    def rDisconnect(self):
        if self.pFlag.TrailON:
            if hasattr(self, 'trail_handle'):
                self.sim.removeDrawingObject(self.trail_handle)
                del self.trail_handle
        self.sim.stopSimulation()
        if self.client:
            self.client = None
            self.sim = None
            print("[SUCCESS] Desconectado ao CoppeliaSim")
    
    def rOpenScene(self,scene):
        if self.pFlag.Opened == 0:
            self.rConnect()
        
        if self.sim.getSimulationState() != self.sim.simulation_stopped:
            self.sim.stopSimulation()
            time.sleep(1)

        scene_path = os.path.abspath(scene)
        result = self.sim.loadScene(scene_path)
        if result == 0:
            raise RuntimeError("Erro ao carregar a cena.")
        self.pFlag.Scene = 1
        self.sim.startSimulation()

        if self.pFlag.TrailON:
            self.trail_handle = self.sim.addDrawingObject(
                self.sim.drawing_lines,
                1.5,
                0.0,
                -1,
                9999,
                [0, 0, 0]
            )

        self.robot_handle = self.sim.getObject('/PioneerP3DX')
        self.left_motor_handle = self.sim.getObject('/PioneerP3DX/leftMotor')
        self.right_motor_handle = self.sim.getObject('/PioneerP3DX/rightMotor')

        position = self.pPos.X[0:3].flatten().tolist()
        quaternion = R.from_euler('z', self.pPos.X[5, 0]).as_quat()
        quaternion = [0, 0, quaternion[2], quaternion[3]]
        self.sim.setObjectPose(self.robot_handle, -1, position + quaternion)

    def rGetSensorData(self):
        self.pPos.Xa = self.pPos.X.copy()  
        
        if self.pFlag.Scene == 1:
            pose = self.sim.getObjectPose(self.robot_handle, -1)
            position = pose[0:3]
            quaternion = pose[3:7]

            r = R.from_quat(quaternion)
            euler = r.as_euler('xyz', degrees=False)

            self.pPos.X[0:6] = np.array(position + list(euler)).reshape((6, 1))
            self.pPos.X[6:12] = (self.pPos.X[0:6] - self.pPos.Xa[0:6]) / self.pPar.Ts

            wl = self.sim.getJointVelocity(self.left_motor_handle)
            wr = self.sim.getJointVelocity(self.right_motor_handle)

            self.pSC.U = np.array([[0.0975 * (wr + wl) / 2], [0.0975 * (wr - wl) / 0.331]])

            if self.pFlag.TrailON and hasattr(self, 'trail_handle'):
                line_points = [
                    self.pPos.Xa[0, 0], self.pPos.Xa[1, 0], 0.02,
                    self.pPos.X[0, 0], self.pPos.X[1, 0], 0.02]
                self.sim.addDrawingObjectItem(self.trail_handle, line_points)

    def rSendControlSignals(self):
        vel_left = (self.pSC.Ud[0, 0] - self.pSC.Ud[1, 0]*0.331/2)/0.0975
        vel_right = (self.pSC.Ud[0, 0] + self.pSC.Ud[1, 0]*0.331/2)/0.0975

        max_wheel_vel = 1.2
        vel_left = np.clip(vel_left, -max_wheel_vel, max_wheel_vel)
        vel_right = np.clip(vel_right, -max_wheel_vel, max_wheel_vel)
        
        self.sim.setJointTargetVelocity(self.left_motor_handle, vel_left)
        self.sim.setJointTargetVelocity(self.right_motor_handle, vel_right)

    def sController(self):
        A = np.array([
            [np.cos(self.pPos.X[5, 0]), -self.pPar.a * np.sin(self.pPos.X[5, 0])],
            [np.sin(self.pPos.X[5, 0]),  self.pPar.a * np.cos(self.pPos.X[5, 0])]
        ])

        Kd = np.diag([0.1, 0.1])
        self.pPos.Xtil = self.pPos.Xd - self.pPos.X
        self.pSC.Ud = np.linalg.pinv(A) @ (self.pPos.Xd[[6, 7]] + Kd @ self.pPos.Xtil[[0, 1]])