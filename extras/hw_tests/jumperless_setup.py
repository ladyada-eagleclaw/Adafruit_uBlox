"""Run on Jumperless V5 MicroPython; retains existing power and other receivers."""
import jumperless as j

# SAM-M8Q prototype straddles the board: GND row 13, VIN row 43.
# Nano V3 remains connected to its own USB. Its TX D7 is deliberately unused.
ROUTES = ((47, j.A4, "SDA"), (46, j.A5, "SCL"),
          (17, j.D8, "GPS TX -> Nano RX"), (14, j.D6, "PPS input"))

if j.context_get() != "global":
    j.context_toggle()
if j.context_get() != "global":
    raise RuntimeError("Persistent global routing context is required")

# Do not disconnect or overwrite another fixture. PA1010D may share the 5 V
# host-side I2C bus at its different address; UART and PPS must be exclusive.
allowed = {int(j.A4): {24, 47}, int(j.A5): {25, 46},
           int(j.D8): {17}, int(j.D6): {14}, int(j.D7): set(),
           int(j.D0): set(), int(j.D1): set()}
for i in range(j.get_num_bridges()):
    a, b, duplicates = j.get_bridge(i)
    for node, peers in allowed.items():
        if (a == node and b not in peers) or (b == node and a not in peers):
            raise RuntimeError("Conflicting Nano route: %s-%s" % (a, b))
for row, node, name in ROUTES:
    j.connect(row, node)
    if not j.is_connected(row, node) or j.get_path_between(row, node) is None:
        raise RuntimeError("Missing route: " + name)
    print("PASS: row %d -> %s (%s)" % (row, node, name))
print("Power rails unchanged. Upload using the Nano's own USB port.")
