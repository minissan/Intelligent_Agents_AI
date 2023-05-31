#python
import numpy as np



def sysCall_init():
    #Sphere radius
    radius = 1.0 
    
    #Points on the surface of the sphere
    points_on_surface = 400
    
    #Shape Diameter
    sphere = sim.createPrimitiveShape(sim.primitiveshape_spheroid, [2, 2, 2])
    
    #sphere= sim.getObject('/Sphere')
    sim.setObjectPosition(sphere,-1,[0,0,1])

    #Generate random angles for theta and phi
    theta = np.random.uniform(0, 2 * np.pi, points_on_surface)
    phi = np.arccos(np.random.uniform(-1, 1, points_on_surface))

    #Convert from spherical to cartesian coordinates
    x = np.sin(phi) * np.cos(theta)
    y = np.sin(phi) * np.sin(theta)
    z = np.cos(phi)

    #Stack the coordinates into an array
    points = radius*np.stack([x, y, z], axis=1)
    
    for i in range(points_on_surface):
        #point coordinates 
        q1,q2,q3= points[i]
    
        p1 = np.random.uniform(0, 1)
        p2 = np.random.uniform(0, 1)
        p3 = np.random.uniform(0, 1)
        
        #Quaternion components calculation
        a = np.sqrt(1 - p1) * np.sin(2 * np.pi * p2)
        b = np.sqrt(1 - p1) * np.cos(2 * np.pi * p2)
        c = np.sqrt(p1) * np.sin(2 * np.pi * p3)
        d = np.sqrt(p1) * np.cos(2 * np.pi * p3)
        
        #Cone on the sphere surface
        cone = sim.createPrimitiveShape(sim.primitiveshape_cone, [0.07, 0.07, 0.07])
        sim.setObjectPosition(cone, sphere, [ q1, q2, q3])
        sim.setObjectQuaternion(cone, sphere,[a,b,c,d])
    
    pass
