"""Backup-test signal isolation and rail measurements; never switches a rail.

Set MODE to isolate, status, or restore before executing on Jumperless.
Leave the Nano and Jumperless on USB. Only the SAM-M8Q VIN jumper is removed
manually. ADC0/2/3 already sense VIN, 3.3 V and module ground respectively.
"""
import jumperless as j

mode = globals().get("MODE", "status")
if mode not in ("isolate", "status", "restore"):
    raise ValueError("Unknown power-test mode")
if j.context_get() != "global":
    j.context_toggle()
if j.context_get() != "global":
    raise RuntimeError("Global routing context required")
for row, node in ((43, j.ADC0), (44, j.ADC2), (13, j.ADC3)):
    if not j.is_connected(row, node):
        raise RuntimeError("Missing power-sense route on row %d" % row)

expected = ((47, j.A4), (46, j.A5), (17, j.D8), (14, j.D6))
if mode == "isolate":
    if globals().get("_sam_power_isolated", False):
        raise RuntimeError("Already isolated; preserve the saved route snapshot")
    # Run jumperless_controls.py in release mode first. Abort on extra wires.
    allowed = {47: {int(j.A4)}, 46: {int(j.A5)}, 17: {int(j.D8)},
               14: {int(j.D6)}, 16: set(), 18: set(), 48: set()}
    for index in range(j.get_num_bridges()):
        a, b, duplicates = j.get_bridge(index)
        for row, peers in allowed.items():
            if (a == row and b not in peers) or (b == row and a not in peers):
                raise RuntimeError("Extra GPS signal connection: %s-%s" % (a, b))
    _sam_power_routes = []
    for row, node in expected:
        if j.is_connected(row, node):
            _sam_power_routes.append((row, node))
            j.disconnect(row, node)
    _sam_power_isolated = True
    print("SAM-M8Q signals isolated; power and ADC sense routes unchanged")
elif mode == "restore":
    if not globals().get("_sam_power_isolated", False):
        raise RuntimeError("Missing isolation snapshot; inspect routes manually")
    for row, node in _sam_power_routes:
        j.connect(row, node)
        if not j.is_connected(row, node):
            raise RuntimeError("Could not restore GPS signal route")
    _sam_power_isolated = False
    print("Original SAM-M8Q signal routes restored")

# Average voltages against the module ground to remove common ADC offset.
ground = sum(j.adc_get(3) for _ in range(8)) / 8
vin = sum(j.adc_get(0) for _ in range(8)) / 8 - ground
vcc = sum(j.adc_get(2) for _ in range(8)) / 8 - ground
print("SAM_POWER: VIN=%.3f V, VCC=%.3f V, ground ADC=%.3f V" %
      (vin, vcc, ground))
