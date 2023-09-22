import subprocess

cfs = [(108, 85), (99, 94)]

for i, c in cfs:
    id = "{0:02X}".format(i)
    uri = "radio://0/{}/1M/E7E7E7E7{}".format(c, id) # 2M
    print(uri)
    try:
        subprocess.call(["rosrun crazyflie_tools reboot --uri " + uri], shell=True)
    except:
        print(uri, "not reachable...")