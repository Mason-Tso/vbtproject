import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv(
    "test.csv",
    header=None,
    names=["time", "accel", "velocity", "position", "state"],
    on_bad_lines="skip"
)

# Drop any row where time isn't numeric (catches >>> lines and header)
df = df[pd.to_numeric(df["time"], errors="coerce").notna()]
df = df.astype({"time": float, "accel": float, "velocity": float, "position": float})

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