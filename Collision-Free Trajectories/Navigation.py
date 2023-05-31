#python
import numpy as np

def sysCall_thread():
    sim.setThreadAutomaticSwitch(True)
    sphere1 = sim.getObject('/Sphere[0]')
    pioneer1 = sim.getObject('/PioneerP3DX[0]')
    sphere2 = sim.getObject('/Sphere[1]')
    pioneer2 = sim.getObject('/PioneerP3DX[1]')

    #sensors = sim.getObject('/PioneerP3DX/ultrasonicSensor[9]')
    sensors = []
    for i in range(16):
        sensor = sim.getObject('/PioneerP3DX/ultrasonicSensor[%d]' % i)
        sensors.append(sensor)

    #sensors = [sim.getObject('./PioneerP3DX/ultrasonicSensor{}'.format(i)) for i in range(1,9)]
        
    while True:
        trajectorySignal = sim.getInt32Signal('trajectory')
        timeStep = sim.getSimulationTimeStep()*0.33

        if trajectorySignal:
            sim.clearInt32Signal("trajectory")
            trajectoryData = sim.readCustomDataBlock(trajectorySignal, 'PATH')
            trajectoryPath = sim.unpackDoubleTable(trajectoryData)
            trajectoryPath = np.reshape(trajectoryPath, (len(trajectoryPath) // 7, 7))
            x = 0
            while x < len(trajectoryPath):
                pioneer1_path = trajectoryPath[x]
                pioneer2_path = trajectoryPath[-x-1]

                sphere1_pos = pioneer1_path[:3].tolist()
                sphere1_ori = pioneer1_path[3:].tolist()
                sphere2_pos = pioneer2_path[:3].tolist()
                sphere2_ori = pioneer2_path[3:].tolist()

                p1, _, _ = sim.checkDistance(sphere1, pioneer1,0.1)
                p2, _, _ = sim.checkDistance(sphere2, pioneer2,0.1)

                if p1 == 1 or p2 == 1:
                    sim.setObjectPosition(sphere1, -1, sphere1_pos)
                    sim.setObjectQuaternion(sphere1, -1, sphere1_ori)

                    sim.setObjectPosition(sphere2, -1, sphere2_pos)
                    sim.setObjectQuaternion(sphere2, -1, sphere2_ori)

                    x += 1

                sim.wait(timeStep)

    pass
