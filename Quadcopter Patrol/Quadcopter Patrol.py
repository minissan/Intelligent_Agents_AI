#python
import random
import math

#Generate Random Target Area within Range (6m X 5m)
def Random_Target(Minimum_X, Maximum_X, Minimum_Y, Maximum_Y):
    Set_Target_X = random.uniform(Minimum_X, Maximum_X)
    Set_Target_Y = random.uniform(Minimum_Y, Maximum_Y)
    
    return Set_Target_X, Set_Target_Y


def sysCall_thread():
    #Get Quadrotor
    quadrotor_handle = sim.getObject("/target")
    
    #Target Area in Meter (6m X 5m)
    Minimum_X = -3
    Maximum_X = 3
    Minimum_Y = -2.5
    Maximum_Y = 2.5
    
    while True:
        Set_Target_X, Set_Target_Y = Random_Target(Minimum_X, Maximum_X, Minimum_Y, Maximum_Y)#Calling target function
        p=sim.getObjectPosition(quadrotor_handle,-1) #get quadrotor location
        p[0]=p[0]+0.01*Set_Target_X #move to target (X-axis)
        if p[0] > Maximum_X: #Check boundaries(X-axis)
            p[0] = random.uniform(Minimum_X, Maximum_X)
        elif p[0] < Minimum_X:
            p[0] = random.uniform(Minimum_X, Maximum_X)
        
        p[1]=p[1]+0.01*Set_Target_Y #move to target (Y-axis)
        if p[1] > Maximum_Y:        #Check boundaries(Y-axis)
            p[1] = random.uniform(Minimum_Y, Maximum_Y)
        elif p[1] < Minimum_Y:
            p[1] = random.uniform(Minimum_Y, Maximum_Y)
        
        sim.setObjectPosition(quadrotor_handle,-1,p) #Set quadrotor position
        
        sim.wait(0.1)

    pass
