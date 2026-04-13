import re
import matplotlib.pyplot as plt

# Storage
velocity = []
position = []
state = []
still = []

# Regex pattern
pattern = re.compile(
    r"V:([-\d\.]+)\s*\|\s*P:([-\d\.]+).*Still:(\d).*State:(\d)"
)

with open("data.txt", "r") as f:
    for line in f:
        match = pattern.search(line)
        if match:
            v, p, s, st = match.groups()
            velocity.append(float(v))
            position.append(float(p))
            still.append(int(s))
            state.append(int(st))

# Time axis
t = list(range(len(velocity)))

# Plot
plt.figure()

plt.plot(t, velocity, label="Velocity")
plt.plot(t, position, label="Position")

# Stillness as scaled signal
plt.plot(t, [s * 0.1 for s in still], label="Still (scaled)")

# State as scaled signal
plt.plot(t, [st * 0.1 for st in state], label="State (scaled)")

plt.legend()
plt.xlabel("Time (samples)")
plt.ylabel("Values")
plt.title("VBT Debug Plot")

plt.show()