"""Optional controls for tests 04-06. Set MODE before executing on Jumperless.

Upload the selected Nano sketch first: its test outputs start released.
MODE is uart, reset, wake_low, wake_high, or release. Existing power and
receive/DDC routes are retained. No Nano output is connected to EXTINT0.
"""
import jumperless as j

mode = globals().get("MODE", "release")
if mode not in ("uart", "reset", "wake_low", "wake_high", "release"):
    raise ValueError("Unknown control mode")
if j.context_get() != "global":
    j.context_toggle()
if j.context_get() != "global":
    raise RuntimeError("Global routing context required")

routes = ((16, j.D7), (48, j.D9), (48, j.A0), (18, j.GPIO_1), (18, j.A1))
allowed = {int(j.D7): {16}, int(j.D9): {48}, int(j.A0): {48},
           int(j.GPIO_1): {18}, int(j.A1): {18}, int(j.D12): set()}
for index in range(j.get_num_bridges()):
    a, b, duplicates = j.get_bridge(index)
    for node, peers in allowed.items():
        if (a == node and b not in peers) or (b == node and a not in peers):
            raise RuntimeError("Conflicting control route: %s-%s" % (a, b))

if mode == "release":
    # Disconnect only this script's explicit control routes.
    for row, node in routes:
        if j.is_connected(row, node):
            j.disconnect(row, node)
    j.gpio_set_dir(j.GPIO_1, j.INPUT)
    print("Control routes released; power and reception unchanged")
elif mode == "uart":
    j.connect(16, j.D7)
    print("RX row 16 -> Nano D7, open-drain UART output")
elif mode == "reset":
    j.connect(48, j.D9)
    j.connect(48, j.A0)
    print("RESET_N row 48 -> D9 open drain and A0 voltage sense")
else:
    # GPIO_1 is a 3.3 V Jumperless output, not a Nano 5 V output.
    j.gpio_set_dir(j.GPIO_1, j.OUTPUT)
    j.gpio_set(j.GPIO_1, mode == "wake_high")
    j.connect(18, j.GPIO_1)
    j.connect(18, j.A1)
    print("EXTINT0 row 18 -> 3.3 V GPIO_1: " + mode)
