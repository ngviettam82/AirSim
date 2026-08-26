# -*- coding: utf-8 -*-
"""Ask the running AirSim what it actually loaded, over its own RPC."""
import sys

import msgpackrpc

HOST = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
c = msgpackrpc.Client(msgpackrpc.Address(HOST, 41451), timeout=10,
                      pack_encoding='utf-8', unpack_encoding='utf-8')

def call(name, *a):
    try:
        return c.call(name, *a)
    except Exception as e:
        return "ERR(%s): %s" % (type(e).__name__, e)

print("ping                :", call("ping"))
print("getServerVersion    :", call("getServerVersion"))
print("simGetVehicleNames  :", call("simGetVehicleNames"))
s = call("getSettingsString")
if isinstance(s, str) and len(s) > 20:
    print("--- settings AirSim dang dung ---")
    print(s[:1600])
else:
    print("getSettingsString   :", s)
