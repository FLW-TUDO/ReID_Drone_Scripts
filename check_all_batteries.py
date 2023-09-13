import subprocess

cfs = [(114, 85), (99, 94)]


batteryVoltageWarning = 3.8  # V
batteryVoltateCritical = 3.7 # V

for i, c in cfs:
    id = "{0:02X}".format(i)
    uri = "radio://0/{}/1M/E7E7E7E7{}".format(c, id) # 2M
    try:
        voltage = subprocess.check_output(["rosrun crazyflie_tools battery --uri " + uri], shell=True)
        voltage = float(voltage)
        status = "Normal"
        if voltage < batteryVoltateCritical:
            status = "Critical"
        elif voltage < batteryVoltageWarning:
            status = "Warning"
    except:
        print(i, uri, "not reachable...")
        continue

    print(i, uri, voltage, status)
