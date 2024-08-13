The RobotiQ 3F-Gripper is modeled in CoppeliaSim software using a state machine.
Required Libraries:
```python
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from statemachine import State
from statemachine import StateMachin
```
How to use this package?
1) Download all the files from this repository, and extract the model file
2) Install CoppeliaSim(CS) on your operating system(Linux is recommended)
3) Lunch the CS software
4) Import the model file into CS
5) import a random object and get the object handle of this object
6) Edit the object handle in the RobotiqClose.py script
7) Run the RobotiqClose.py script
