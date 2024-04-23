# ReID_Drone_Scripts

## Models 
Look at this link to aideck/README.md ()[aideck/README.md]

## Important files

[reid_demo_v2.py](reid_demo_v2.py)  
Currently used script to controll the Re-ID sequence. Steers the drone through the whole detection process (pallet and palletblocks)

[Controller.py](Controller.py)  
In case the flying script crashes and the drone has to be landed manually. Call via terminal.

[chooser.py](chooser.py)  
Check if the correct drones are selected for flight and check the status of the batteries.

[check_all_batteries.py](check_all_batteries.py)  
Call via Terminal to get a simple text-based output whether the batteries will last or need to changed before the next flight.

[utils.py](utils.py)  
Utility file with auxiliary functions for reid_demo_v2.

