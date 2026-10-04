# -*- coding: utf-8 -*-
r"""Write the PA0 settings.json (PX4 HIL, no cameras) to a project-owned path.

Deliberately NOT written into Documents\AirSim: on this machine Documents is
OneDrive-redirected, so there are two AirSim folders and only one is read. The
launcher passes -settings=<this file>, which outranks every search path AirSim
tries (SimHUD.cpp:388), and leaves the user's own config untouched.

IPs are derived, never hardcoded: WSL's address changes on every reboot.
"""
import io
import json
import os
import subprocess
import sys

TARGET = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      "airsim_settings_pa0.json")


def wsl(cmd):
    out = subprocess.run(["wsl.exe", "--", "bash", "-lc", cmd],
                         capture_output=True, text=True, timeout=60)
    return out.stdout.strip()


wsl_ip = wsl("hostname -I | cut -d' ' -f1")
host_ip = wsl("ip route show default | cut -d' ' -f3")
if not wsl_ip or not host_ip:
    print("FAIL: could not derive IPs (wsl_ip=%r host_ip=%r)" % (wsl_ip, host_ip))
    sys.exit(1)

settings = {
    "SettingsVersion": 2.0,
    "SimMode": "Multirotor",
    # Real time, not deterministic: decision D3. LockStep must be said out loud
    # because AirSim defaults it to true.
    "ClockType": "ScalableClock",
    "ClockSpeed": 1,
    "OriginGeopoint": {
        "Latitude": 10.7729,
        "Longitude": 106.6577,
        "Altitude": 10
    },
    "Vehicles": {
        "uav0": {
            "VehicleType": "PX4Multirotor",
            "UseSerial": False,
            "UseTcp": True,
            "TcpPort": 4560,
            "LockStep": False,
            # AirSim binds BOTH the HIL accept and the RPC server to this
            # address (AirSimSettings.hpp:1482 and :2187). 127.0.0.1 would be
            # unreachable from WSL2, which is where PX4 and airsim_node live.
            "LocalHostIp": "0.0.0.0",
            "ControlIp": wsl_ip,
            "ControlPortLocal": 14540,
            "ControlPortRemote": 14580,
            "X": 0.0, "Y": 0.0, "Z": 0.0,
            "Pitch": 0.0, "Roll": 0.0, "Yaw": 0.0
        }
    }
}

tmp = TARGET + ".tmp"
with io.open(tmp, "w", encoding="utf-8") as f:
    f.write(json.dumps(settings, indent=2))
os.replace(tmp, TARGET)

print("wrote %s" % TARGET)
print("  WSL  (PX4 + airsim_node) : %s" % wsl_ip)
print("  Windows host (from WSL)  : %s" % host_ip)
print("  -> PX4_SIM_HOSTNAME=%s" % host_ip)
