import numpy as np
import time as t
import sys
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from GripperStateMachine import GripperStateMachine as GSM
from SensorProcessor import SensorProcessor as SP

## Dimesnsions
'''

all in mm
KUKA Media Flange Touch Electric: 61
Two Couplings                     28
1st coupling gap                 -03.5
2nd coupling gap                  03.175
Gripper base                      92
Total offset from connection point in CS model to the attach point in the gripper
should be: 61+28+3.175+92-3.5 = 180.675

'''
# Determine the appropriate getch() function based on the platform
if sys.platform.startswith('win'):
    import msvcrt

    def getch():
        return msvcrt.getch().decode()
else:
    import tty
    import termios

    def getch():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            char = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return char
    
# Custom Varaibles
deg = np.pi/180
rad = 180/np.pi
tol = 2
Limits = [67, 90, 43, -55]

## Remote APT initialization 
client = RemoteAPIClient()
sim = client.getObject('sim')

## Retrieving robot joint handles
robot_joint_handle = []
robot_joint_handle.append(sim.getObject('/LBRiiwa14R820/joint'))
for i in range(1,7):
    robot_joint_handle.append(sim.getObject(f'/LBRiiwa14R820/link{i+1}_resp/joint'))

## Retrieving Gripper Handles
GripperJointHandle = np.zeros(9) 
GripperSensorHandles = np.zeros(9)
GripperTipsConvexHullHandles = np.zeros(9)
GripperPalmSensor = sim.getObject("/LBRiiwa14R820/Coupling/Robotiq3F/Palm_sensor")
attachPoint = sim.getObject("/LBRiiwa14R820/Coupling/Robotiq3F/AttachPoint") 

for i in range(9):

    finger_number = (i // 3) + 1
    link_number = (i % 3) + 1

    JointAlias = f'/LBRiiwa14R820/Coupling/Robotiq3F/finger_{finger_number}_joint_{link_number}'
    SensorAlias = f'/LBRiiwa14R820/Coupling/Robotiq3F/finger{finger_number}_link{link_number}_sensor'
    ConvexHullAlias = f'/LBRiiwa14R820/Coupling/Robotiq3F/finger{finger_number}_link{link_number}_sensor2'

    GripperJointHandle[i] = sim.getObject(JointAlias)
    GripperSensorHandles[i] = sim.getObject(SensorAlias)
    GripperTipsConvexHullHandles[i] = sim.getObject(ConvexHullAlias)

## Object Handle
Shape = sim.getObject("/Cuboid")


# Locking Joints when velocity is zero
for i in range(len(GripperJointHandle)):
    sim.setObjectInt32Param(GripperJointHandle[i],sim.jointintparam_velocity_lock,1)

# Functions
def ReadSensor(Finger, ObjectHandle):
    # Transitions Event = [C1 C2 C3 L1 L2 L3 L4],
    # Ci = 1 indicates that link i is blocked by the object
    # Li = 1 indicates that link i has reached the limit
    # L4 = 1 indicates that link 3 has reached the lower limit
    i = (Finger - 1) * 3 
        
    C1, _, _, _, _= sim.checkProximitySensor(GripperSensorHandles[i], ObjectHandle)
    C2, _, _, _, _= sim.checkProximitySensor(GripperSensorHandles[i+1], ObjectHandle)
    C3, _, _, _, _= sim.checkProximitySensor(GripperSensorHandles[i+2], ObjectHandle)

    L1 = (np.abs(sim.getJointPosition(GripperJointHandle[i]) * rad - Limits[0]) <= tol)
    L2 = (np.abs(sim.getJointPosition(GripperJointHandle[i+1]) * rad - Limits[1]) <= tol)
    #print(f'Finger {Finger}, L1 is {L1}, T_real is {sim.getJointPosition(GripperJointHandle[i]) * rad} and Limit is {Limits[0]}')

    theta3 = sim.getJointPosition(GripperJointHandle[i+2]) * rad
    L3 = Limits[2] - tol <= theta3 <= Limits[2] + tol
    L4 = Limits[3] - tol <= theta3 <= Limits[3] + tol
    L1, L2, L3, L4 = int(L1), int(L2), int(L3), int(L4)

    return [C1, C2, C3, L1, L2, L3, L4]

def Move_Finger_Velocity_Mode(FingerNumber,LinkVelocity):

    i = (FingerNumber - 1) * 3  

    LinkVelocity = [int(bit) for bit in LinkVelocity]
    LinkVelocity[2] = -1 if LinkVelocity[3] == 1 else LinkVelocity[2]
    LinkVelocity.pop()
    LinkHandels = [GripperJointHandle[i], GripperJointHandle[i+1], GripperJointHandle[i+2]]
    sim.setJointTargetVelocity(LinkHandels[0], LinkVelocity[0] * deg * 15)
    sim.setJointTargetVelocity(LinkHandels[1], LinkVelocity[1] * deg * 30)
    sim.setJointTargetVelocity(LinkHandels[2], LinkVelocity[2] * deg * 15)

# Defining objects of Gripper State Machine Class GSM
GSM1 = GSM(finger = 1)
GSM2 = GSM(finger = 2)
GSM3 = GSM(finger = 3)

# Wrapping them into a dict for easy use
GSMdict = {
    'Finger1': GSM1,
    'Finger2': GSM2,
    'Finger3': GSM3
}

# Retriveing final states 
FinalState = GSM1.final_states
FinalStateValues = []
for i in range(len(FinalState)):
    FinalStateValues.append(FinalState[i].value)

# Defining objects of Sensor Processor Calss
SP1 = SP(ReadSensor(1, Shape))
SP2 = SP(ReadSensor(2, Shape))
SP3 = SP(ReadSensor(3, Shape))

# Wrapping them into a Dict for easy use
SPdict = {
    'Finger1': SP1,
    'Finger2': SP2,
    'Finger3': SP3
}

# Runing the simulation
# run_simulation()
sim.startSimulation()

# Sending close commond
for i in range(3):
    Finger = f'Finger{i+1}'
    GSMdict[Finger].send("Close")
    Move_Finger_Velocity_Mode(i+1,GSMdict[Finger].action)

while not (
    (GSM1.current_state.value in FinalStateValues) &
    (GSM2.current_state.value in FinalStateValues) &
    (GSM3.current_state.value in FinalStateValues)
):
    
    for i in range(3):
        Finger = f'Finger{i+1}'
        current_sensor_data = ReadSensor(i+1, Shape)
        #print(f'Current Sensor data for {Finger} is {current_sensor_data}')
        result, index = SPdict[Finger].process_sensor_data(current_sensor_data)
        if (result and GSMdict[Finger].current_state_value not in FinalStateValues):
            for j in range(len(index)):
                Event = f"Event{index[j]+1}"
                try:
                    print(f'index changed and {Event} occured for {Finger}')
                    print(f'Action {GSMdict[Finger].action} is sent to {Finger}')
                    GSMdict[Finger].send(Event)
                    Move_Finger_Velocity_Mode(i+1,GSMdict[Finger].action)
                except GSMdict[Finger].TransitionNotAllowed:
                    pass  
                
# Making sure that the gripper is stopped
t.sleep(1)
for i in range(3):
    Finger = f'Finger{i+1}'
    Move_Finger_Velocity_Mode(i+1,GSMdict[Finger].action)

ObjectDetected = res, _, _, _, _= sim.checkProximitySensor(GripperPalmSensor, Shape)
if res: 
    sim.setObjectParent(Shape,attachPoint,True)
    print('Faking the grasp')

sim.setJointTargetPosition(robot_joint_handle[5], 90 * deg)

print('Press q to quit:')
while True:
    user_input = getch()
    if user_input.lower() == 'q':
        while sim.getSimulationState() != sim.simulation_stopped:
            sim.stopSimulation()
        #sim.stopSimulation()  # Call the stopSimulation() function
        break  # Exit the loop


'''
    -- You have basically 2 alternatives to grasp an object:
    --
    -- 1. You try to grasp it in a realistic way. This is quite delicate and sometimes requires
    --    to carefully adjust several parameters (e.g. motor forces/torques/velocities, friction
    --    coefficients, object masses and inertias)
    --
    -- 2. You fake the grasping by attaching the object to the gripper via a connector. This is
    --    much easier and offers very stable results.
    --
    -- Alternative 2 is explained hereafter:
    --
    --
    -- a) In the initialization phase, retrieve some handles:
    -- 
    -- connector=sim.getObject('./attachPoint')
    -- objectSensor=sim.getObject('./attachProxSensor')
    
    -- b) Before closing the gripper, check which dynamically non-static and respondable object is
    --    in-between the fingers. Then attach the object to the gripper:
    --
    -- index=0
    -- while true do
    --     shape=sim.getObjects(index,sim.object_shape_type)
    --     if (shape==-1) then
    --         break
    --     end
    --     if (
        sim.getObjectInt32Param(shape,sim.shapeintparam_static)==0) and 
        (sim.getObjectInt32Param(shape,sim.shapeintparam_respondable)~=0) and 
        (sim.checkProximitySensor(objectSensor,shape)==1) then
    --         -- Ok, we found a non-static respondable shape that was detected
    --         attachedShape=shape
    --         -- Do the connection:
    --         sim.setObjectParent(attachedShape,connector,true)
    --         break
    --     end
    --     index=index+1
    -- end
    
    -- c) And just before opening the gripper again, detach the previously attached shape:
    --
    -- sim.setObjectParent(attachedShape,-1,true)
'''