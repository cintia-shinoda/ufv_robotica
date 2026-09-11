import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

def conectar():
    for attempt in range(20):
        try:
            client = RemoteAPIClient()
            sim = client.getObject('sim')
            print(f"[SUCCESS] Conectado ao CoppeliaSim")
            return sim
        except Exception:
            print(f"[WAITING] Tentando conectar... ({attempt + 1}/20)")
            time.sleep(1)
    raise TimeoutError("Não foi possível conectar ao CoppeliaSim.")