#python
import numpy as np
def sysCall_thread():
    # e.g. non-synchronized loop:
    # sim.setThreadAutomaticSwitch(True)
    # while True:
    #     p=sim.getObjectPosition(objHandle,-1)
    #     p[0]=p[0]+0.001
    #     sim.setObjectPosition(objHandle,-1,p)
        
    # e.g. synchronized loop:
    # sim.setThreadAutomaticSwitch(False)
    # while True:
    #     p=sim.getObjectPosition(objHandle,-1)
    #     p[0]=p[0]+0.001
    #     sim.setObjectPosition(objHandle,-1,p)
    #     sim.switchThread() # resume in next simulation step
    
    sim.setThreadAutomaticSwitch(True)
    quadcopter = sim.getObject('/target')
    
    while True:
        trajectorySignal = sim.getInt32Signal('trajectory')
        timeStep = sim.getSimulationTimeStep()

        if trajectorySignal:
            sim.clearInt32Signal("trajectory")
            trajectoryData = sim.readCustomDataBlock(trajectorySignal, 'PATH')
            trajectoryPath = sim.unpackDoubleTable(trajectoryData)
            trajectoryPath = np.reshape(trajectoryPath, (len(trajectoryPath) // 7, 7))

            for position, orientation in map(lambda p: (p[:3], p[3:]), trajectoryPath):
                sim.setObjectPosition(quadcopter, -1, position.tolist())
                sim.setObjectQuaternion(quadcopter, -1, orientation.tolist())
                sim.wait(timeStep)

    
    pass

# See the user manual or the available code snippets for additional callback functions and details
