def Pegar_Eixos(sim):    
    return {
            'axis1': sim.getObject('/Manipulador/axis1'),
            'axis2': sim.getObject('/Manipulador/axis2'),
            'axis3': sim.getObject('/Manipulador/axis3'),
            'tcp': sim.getObject('/Manipulador/axis4')
        }