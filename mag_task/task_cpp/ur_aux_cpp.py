# Interface for communication with UR10 in V-REP
# Don't modify it!

#import vrep
import sim as vrep
import time, math
import cmath
# image
import numpy as np

def DH(a,al,d,q):
  cq = np.cos(q); sq = np.sin(q)
  ca = np.cos(al); sa = np.sin(al)
  return np.array([
       [cq,-sq*ca, sq*sa,a*cq],
       [sq, cq*ca,-cq*sa,a*sq],
       [ 0,    sa,    ca,   d],
       [ 0,     0,     0,   1]],
       dtype=float)

# Main class for the robot control       
class VrepModel:
  def __init__(self,port):
    # just in case, close all opened connections
    vrep.simxFinish(-1)
    # connect, get scene handl
    self.clientId = vrep.simxStart('127.0.0.1',port,True,True,5000,5) 
    assert (self.clientId != -1), "Impossible to connect!"
    print ("Connected to remote API server")
    # get camera
    err,self.cameraId = vrep.simxGetObjectHandle(self.clientId,'Camera',vrep.simx_opmode_oneshot_wait)
    assert (err != -1), "No camera found!"
    # get joints
    self.jointNo = range(6)
    self.jointId = [-1,-1,-1,-1,-1,-1]
    for i in self.jointNo:
      err,h = vrep.simxGetObjectHandle(self.clientId,'UR10_joint'+str(i+1),vrep.simx_opmode_oneshot_wait)
      assert (err != -1), "No joint "+str(i)+" found!"
      self.jointId[i] = h
    # get gripper
    err,self.gripperId = vrep.simxGetObjectHandle(self.clientId,'RG2',vrep.simx_opmode_oneshot_wait)
    assert (err != -1), "No gripper found!"
    # force/torque sensor
    err,self.sensorId = vrep.simxGetObjectHandle(self.clientId,'UR10_connection',vrep.simx_opmode_oneshot_wait)
    assert (err != -1), "No sensor found!"
    # get tip
    err,self.tipId = vrep.simxGetObjectHandle(self.clientId,'tip',vrep.simx_opmode_oneshot_wait)
    assert (err != -1), "No tip found!"    
    # kinematics 
    self.par_a = [0,    -0.6121,-0.5721,     0,     0,     0]
    self.par_d = [0.128,      0,      0,0.1639,0.1157,0.0947]
    self.par_al = [math.pi*0.5,0,0,math.pi*0.5,-math.pi*0.5,0]
    self.par_dq = [0,-math.pi*0.5,0,-math.pi*0.5,0,0]
    self.base = np.matrix([[0,1,0,-0.025],[-1,0,0,0.375],[0,0,1,0],[0,0,0,1]])   
    #self.base
      
  # Start process
  def startSimulation(self):
    # start simulation
    vrep.simxStartSimulation(self.clientId, vrep.simx_opmode_blocking)
    # open streaming
    vrep.simxGetVisionSensorImage(self.clientId,self.cameraId,0,vrep.simx_opmode_streaming)
    vrep.simxReadForceSensor(self.clientId,self.sensorId,vrep.simx_opmode_streaming)
    time.sleep(1)
    self.beginTime = time.time()
    
  # Stop and exit program
  def stopSimulation(self):
    vrep.simxStopSimulation(self.clientId, vrep.simx_opmode_blocking)
    vrep.simxFinish(self.clientId)
    
  # Go to desired position in joint space
  # joints - list of joint angles
  def ptp(self,joints):
    ba = bytearray()
    res = vrep.simxCallScriptFunction(self.clientId,'UR10',vrep.sim_scripttype_childscript,
                                      'setRobotGoal',[],joints,['ptp'],ba, vrep.simx_opmode_blocking)
    while True:  # waiting for result
      res = vrep.simxCallScriptFunction(self.clientId,'UR10',vrep.sim_scripttype_childscript,
                                      'isReached',[],[],[''],ba, vrep.simx_opmode_blocking)
      if res[1][0] == 1: break
      time.sleep(0.1)
     
  # Go to desired position and orientation in Cartesian space
  # pos - list of coordinates [x,y,z]
  # orient - list of Euler angles in radians (if list is empty, orientation will not be changed)                                            
  def lin(self,pos,orient=[]):
    ba = bytearray()
    res = vrep.simxCallScriptFunction(self.clientId,'UR10',vrep.sim_scripttype_childscript,
                                      'setRobotGoal',[],pos+orient,['lin'],ba, vrep.simx_opmode_blocking)
    while True:   # waiting for result
      res = vrep.simxCallScriptFunction(self.clientId,'UR10',vrep.sim_scripttype_childscript,
                                      'isReached',[],[],[''],ba, vrep.simx_opmode_blocking)
      if res[1][0] == 1: break
      time.sleep(0.1)
  
  # Return list of the current joint angles 
  def getJointPosition(self):
    res = [-1,-1,-1,-1,-1,-1]
    for i in self.jointNo:
      err,v = vrep.simxGetJointPosition(self.clientId,self.jointId[i],vrep.simx_opmode_oneshot_wait)
      assert (err != -1), "Can't get position"
      res[i] = v
    return res
    
      
  # Open gripper if flag is True, close otherwise
  def gripperOpen(self,flag):
    vrep.simxSetIntegerSignal(self.clientId,'RG2_open',flag,vrep.simx_opmode_oneshot_wait)
    time.sleep(0.5)
    
  # Read information from the visual sensor
  # Return RGB image
  def getImage(self):
    err,resolution,image = vrep.simxGetVisionSensorImage(self.clientId,self.cameraId,0,vrep.simx_opmode_buffer)    
    newImg = np.array(image,dtype = np.uint8)        # to byte array
    newImg.resize([resolution[0],resolution[1],3])   # to image    
    return newImg 
      
  # Get current tool position    
  def getToolPosition(self):
    err,pos = vrep.simxGetObjectPosition(self.clientId,self.tipId,-1,vrep.simx_opmode_oneshot_wait)
    assert (err != -1), "Can't get position!"
    return pos
  
  # Get current tool orientation  
  def getToolOrientation(self):
    err,orient = vrep.simxGetObjectOrientation(self.clientId,self.tipId,-1,vrep.simx_opmode_oneshot_wait)
    assert (err != -1), "Can't get orientation!"
    return orient
    
  def getForceTorque(self):
    err,_,force,torque = vrep.simxReadForceSensor(self.clientId,self.sensorId,vrep.simx_opmode_buffer)
    assert (err != -1), "Can't read sensor!"
    return force,torque
    
  def check(self):
    # execution time
    dt = time.time() - self.beginTime
    # get pegs
    pegId = [-1,-1,-1,-1,-1]
    for i in range(len(pegId)):
      err, v = vrep.simxGetObjectHandle(self.clientId,'cylinder'+str(i+1),vrep.simx_opmode_oneshot_wait)
      assert (err != -1), "No cylinder found!"
      pegId[i] = v  
    # check location
    position = [-1,-1,-1,-1,-1]
    for i,p in enumerate(pegId):
      _,pos = vrep.simxGetObjectPosition(self.clientId,p,-1,vrep.simx_opmode_oneshot_wait)
      position[i] = pos    
    # compare
    dist = lambda x,y: math.sqrt((x[0]-y[0])**2 + (x[1]-y[1])**2 + (x[2]-y[2])**2)    
    #N = 0            # number of the found pegs
    #for i in range(25):
    #  err, h = vrep.simxGetObjectHandle(self.clientId,'p'+str(i+1),vrep.simx_opmode_oneshot_wait)  
    #  assert (err != -1), "Object not found!"
    #  _,pos = vrep.simxGetObjectPosition(self.clientId,h,-1,vrep.simx_opmode_oneshot_wait)
    #  for p in position:
    #    if dist(p,pos) < 0.05:
    #      N += 1
    #      break
    N = 0            # number of the pegs in colored hales 
    for i in range(5):
      err, h = vrep.simxGetObjectHandle(self.clientId,'marker'+str(i+1),vrep.simx_opmode_oneshot_wait)  
      assert (err != -1), "Object not found!"
      _,pos = vrep.simxGetObjectPosition(self.clientId,h,-1,vrep.simx_opmode_oneshot_wait)
      for p in position:
        if dist(p,pos) < 0.05:
          N += 1
          break
    print ('Pegs:',N,'Time:',dt)    
      
  # Forward kinematics solution
  # Return matrix of homogenous transformation for the given joint angles
  def fk(self,q):    
    res = self.base.copy()
    for i in self.jointNo:
      res = res.dot(DH(self.par_a[i],self.par_al[i],self.par_d[i],q[i]+self.par_dq[i]))
    return res
  
  # Find Jacobian
  # Return the obtained matrix where each row is a possible solution    
  def jacobian(self,q):
    c1 = np.cos(q[0]+self.par_dq[0]); c2 = np.cos(q[1]+self.par_dq[1]); c3 = np.cos(q[2]+self.par_dq[2]); c4 = np.cos(q[3]+self.par_dq[3]); c5 = np.cos(q[4]+self.par_dq[4]); c6 = np.cos(q[5]+self.par_dq[5])
    s1 = np.sin(q[0]+self.par_dq[0]); s2 = np.sin(q[1]+self.par_dq[1]); s3 = np.sin(q[2]+self.par_dq[2]); s4 = np.sin(q[3]+self.par_dq[3]); s5 = np.sin(q[4]+self.par_dq[4]); s6 = np.sin(q[5]+self.par_dq[5]) 
    m = np.zeros((6,6), dtype=float)
     
    v1 = ((c1*c2*s3+c1*s2*c3)*s4+(c1*s2*s3-c1*c2*c3)*c4);
    v2 = ((c1*s2*s3-c1*c2*c3)*s4-(c1*c2*s3+c1*s2*c3)*c4);
    v3 = ((c2*c3-s2*s3)*s4+(c2*s3+s2*c3)*c4);
    v4 = ((c2*s3+s2*c3)*s4-(c2*c3-s2*s3)*c4);
    v5 = (((-s1*c2*s3)-s1*s2*c3)*s4+(s1*c2*c3-s1*s2*s3)*c4);
    v6 = ((s1*c2*c3-s1*s2*s3)*s4-((-s1*c2*s3)-s1*s2*c3)*c4);
    
    m[0][0] = (-0.0947*((-v1*s5)-s1*c5))-0.1157*v2+0.5721*c1*s2*s3-0.5721*c1*c2*c3-0.6121*c1*c2+0.1639*s1; 
    m[0][1] = -s1*((-0.0947*v3*s5)+0.1157*v4-0.5721*c2*s3-0.5721*s2*c3-0.6121*s2); 
    m[0][2] = -s1*((-0.0947*v3*s5)+0.1157*v4-0.5721*c2*s3-0.5721*s2*c3); 
    m[0][3] = -s1*(0.1157*v4-0.0947*v3*s5); 
    m[0][4] = ((c2*c3-s2*s3)*c4-(c2*s3+s2*c3)*s4)*(0.0947*((-v1*s5)-s1*c5)+0.1157*v2)+v2*(0.1157*v4-0.0947*v3*s5); 
    m[0][5] = ((-v1*s5)-s1*c5)*((-0.0947*v3*s5)-0.1157*v4+0.1157*v4)+0.0947*v3*s5*((-v1*s5)-s1*c5); 
    m[1][0] = 0.0947*((-v5*s5)-c1*c5)+0.1157*v6+0.5721*s1*s2*s3-0.5721*s1*c2*c3-0.6121*s1*c2-0.1639*c1; 
    m[1][1] = c1*((-0.0947*v3*s5)+0.1157*v4-0.5721*c2*s3-0.5721*s2*c3-0.6121*s2); 
    m[1][2] = c1*((-0.0947*v3*s5)+0.1157*v4-0.5721*c2*s3-0.5721*s2*c3); 
    m[1][3] = c1*(0.1157*v4-0.0947*v3*s5); 
    m[1][4] = v4*(0.0947*((-v5*s5)-c1*c5)+0.1157*v6)+(((-s1*c2*s3)-s1*s2*c3)*c4-(s1*c2*c3-s1*s2*s3)*s4)*(0.1157*v4-0.0947*v3*s5); 
    m[1][5] = (v5*s5+c1*c5)*((-0.0947*v3*s5)-0.1157*v4+0.1157*v4)-0.0947*v3*s5*((-v5*s5)-c1*c5); 
    m[2][1] = s1*(0.0947*((-v5*s5)-c1*c5)+0.1157*v6+0.5721*s1*s2*s3-0.5721*s1*c2*c3-0.6121*s1*c2-0.1639*c1)-c1*(0.0947*((-v1*s5)-s1*c5)+0.1157*v2-0.5721*c1*s2*s3+0.5721*c1*c2*c3+0.6121*c1*c2-0.1639*s1); 
    m[2][2] = s1*(0.0947*((-v5*s5)-c1*c5)+0.1157*v6+0.5721*s1*s2*s3-0.5721*s1*c2*c3-0.1639*c1)-c1*(0.0947*((-v1*s5)-s1*c5)+0.1157*v2-0.5721*c1*s2*s3+0.5721*c1*c2*c3-0.1639*s1); 
    m[2][3] = s1*(0.0947*((-v5*s5)-c1*c5)+0.1157*v6-0.1639*c1)-c1*(0.0947*((-v1*s5)-s1*c5)+0.1157*v2-0.1639*s1); 
    m[2][4] = ((c1*c2*s3+c1*s2*c3)*c4-(c1*s2*s3-c1*c2*c3)*s4)*(0.0947*((-v5*s5)-c1*c5)+0.1157*v6)+v6*(0.0947*((-v1*s5)-s1*c5)+0.1157*v2); 
    m[2][5] = 0.0947*(v1*s5+s1*c5)*((-v5*s5)-c1*c5)+0.0947*((-v1*s5)-s1*c5)*((-v5*s5)-c1*c5); 
    m[3][1] = -c1; 
    m[3][2] = -c1; 
    m[3][3] = -c1; 
    m[3][4] = (s1*c2*c3-s1*s2*s3)*s4-((-s1*c2*s3)-s1*s2*c3)*c4; 
    m[3][5] = (-v5*s5)-c1*c5; 
    m[4][1] = -s1; 
    m[4][2] = -s1; 
    m[4][3] = -s1; 
    m[4][4] = (c1*s2*s3-c1*c2*c3)*s4-(c1*c2*s3+c1*s2*c3)*c4; 
    m[4][5] = (-v1*s5)-s1*c5; 
    m[5][0] = 1.0; 
    m[5][4] = (c2*s3+s2*c3)*s4-(c2*c3-s2*s3)*c4; 
    m[5][5] = -v3*s5;     
    return m
  
  # Inverse kinematics solution
  # Return list of found solutions   
  def ik(self,goal):
    theta = np.zeros((8, 6))
            
    T_06 = np.linalg.inv(self.base).dot(goal)

    # theta1
    P_05 = T_06[0:3, 3] - self.par_d[5] * T_06[0:3, 2]
    phi1 = math.atan2(P_05[1], P_05[0])
    phi2 = math.acos(self.par_d[3] / math.sqrt(P_05[0] ** 2 + P_05[1] ** 2))
    theta1 = [math.pi / 2 + phi1 + phi2, math.pi / 2 + phi1 - phi2]
    theta[0:4, 0] = theta1[0]-self.par_dq[0]
    theta[4:8, 0] = theta1[1]-self.par_dq[1]

    # theta5
    P_06 = T_06[0:3, 3]
    theta5 = []
    for i in range(2):
      theta5.append(math.acos((P_06[0] * math.sin(theta1[i]) - P_06[1] * math.cos(theta1[i]) - self.par_d[3]) / self.par_d[5]))
    for i in range(2):
      theta[2*i, 4] = theta5[0]-self.par_dq[0]
      theta[2*i+1, 4] = -theta5[0]-self.par_dq[0]
      theta[2*i+4, 4] = theta5[1]-self.par_dq[1]
      theta[2*i+5, 4] = -theta5[1]-self.par_dq[1]

    # theta6
    T_60 = np.linalg.inv(T_06)
    theta6 = []
    for i in range(2):
      for j in range(2):
        s1 = math.sin(theta1[i])
        c1 = math.cos(theta1[i])
        s5 = math.sin(theta5[j])
        theta6.append(math.atan2((-T_60[1, 0] * s1 + T_60[1, 1] * c1) / s5, (T_60[0, 0] * s1 - T_60[0, 1] * c1) / s5))
    for i in range(2):
      theta[i, 5]   = theta6[0]-self.par_dq[0]
      theta[i+2, 5] = theta6[1]-self.par_dq[1]
      theta[i+4, 5] = theta6[2]-self.par_dq[2]
      theta[i+6, 5] = theta6[3]-self.par_dq[3]

    # theta2, theta3, theta4
    for i in range(8):
      # theta3
      T_46 = DH(self.par_a[4],self.par_al[4],self.par_d[4],theta[i,4]+self.par_dq[4]).dot( DH(self.par_a[5],self.par_al[5],self.par_d[5],theta[i,5]+self.par_dq[5]) )
      T_01 = DH(self.par_a[0],self.par_al[0],self.par_d[0],theta[i,0]+self.par_dq[0])
      T_14 = np.linalg.inv(T_01).dot( T_06.dot(np.linalg.inv(T_46))) 
      P_13 = T_14.dot( np.array([[0, -self.par_d[3], 0, 1]]).T ) - np.array([[0, 0, 0, 1]]).T
      PN = np.linalg.norm(P_13)      
      if j in [0, 2, 4, 6]:
        theta[j, 2] = -cmath.acos((PN ** 2 - self.par_a[1] ** 2 - self.par_a[2] ** 2) / (2 * self.par_a[1] * self.par_a[2])).real - self.par_dq[2]
        theta[j+1, 2] = -theta[j, 2] - self.par_dq[2]
      # theta2            
      theta[i, 1] = -math.atan2(P_13[1], -P_13[0]) + math.asin(self.par_a[2] * math.sin(theta[i, 2]) / PN) - self.par_dq[1]
      # theta4
      T_13 = DH(self.par_a[1],self.par_al[1],self.par_d[1],theta[i,1]+self.par_dq[1]).dot( DH(self.par_a[2],self.par_al[2],self.par_d[2],theta[i,2]+self.par_dq[2]) )
      T_34 = np.linalg.inv(T_13) * T_14
      theta[i, 3] = math.atan2(T_34[1, 0], T_34[0, 0]) - self.par_dq[3]
    
    return theta.tolist()


      
