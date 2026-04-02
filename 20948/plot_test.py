import pandas as pd
import matplotlib.pyplot as plt

# Load CSV
df = pd.read_csv(
    "test.csv",
    header=None,
    names=["time", "accel", "velocity", "position"],
    on_bad_lines="skip"
)

df = df.dropna()

# Create plot
plt.figure(figsize=(12,6))

plt.plot(df["time"], df["accel"], label="Acceleration")
plt.plot(df["time"], df["velocity"], label="Velocity")
plt.plot(df["time"], df["position"], label="Position")

# Add y = 0 reference line
plt.axhline(0)

plt.legend()
plt.xlabel("Time")
plt.ylabel("Value")
plt.title("VBT IMU Signals")

# Save image
plt.savefig("vbt_plot.png", dpi=300)

plt.show()