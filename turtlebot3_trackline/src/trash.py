import time
import matplotlib.pyplot as plt

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.last_error = 0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        self.last_error = error

        output = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )
        return output

# Simulated environment
current_temp = 20  # Start temperature
setpoint = 25      # Target temperature
pid = PIDController(kp=2.0, ki=0.1, kd=1.0)

temps = []
times = []

for t in range(100):
    error = setpoint - current_temp
    power = pid.update(error, dt=1.0)

    # Simulated temperature change
    current_temp += 0.1 * power  # Power changes temp
    temps.append(current_temp)
    times.append(t)
    time.sleep(0.01)

# Plot the result
plt.plot(times, temps)
plt.axhline(y=setpoint, color='r', linestyle='--')
plt.title("Temperature Control via PID")
plt.xlabel("Time")
plt.ylabel("Temperature (°C)")
plt.grid(True)
plt.show()