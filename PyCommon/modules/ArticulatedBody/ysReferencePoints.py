
import sys
if '..' not in sys.path:
    sys.path.append('..')
import Math.mmMath as mm

def getCM(pos_or_vels, masses, totalMass=None, validIndexes=None):
    if validIndexes==None:
        validIndexes = range(len(pos_or_vels))
    if totalMass==None:
        totalMass = 0.
        for i in validIndexes:
            totalMass += masses[i]
        
    CM = mm.Vec3(0.,0.,0.)
    for i in validIndexes:
        CM += (pos_or_vels[i] * masses[i])
    CM /= totalMass
    return CM

def getCP(contactPositions, contactForces, normal=(0,1,0)):
    if(len(contactPositions) == 0): 
        return None
    
    CP = mm.Vec3(0.,0.,0.)
    totalNormalForce = 0.
    
    for i in range(len(contactPositions)):
        CP += (contactPositions[i] * contactForces[i][1])
        totalNormalForce += contactForces[i][1]
    
    CP /= totalNormalForce
    return CP