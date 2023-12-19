from scipy.optimize import minimize
from scipy.signal import TransferFunction, step
import matplotlib.pyplot as plt

numerator = [float(i) for i in input("Enter TF Numerator: ").split()]
denominator = [float(i) for i in input("Enter TF Denominator: ").split()]
sys = TransferFunction(numerator, denominator)


def objective_function(params):
    kp, ki, kd = params

    tf = TransferFunction(numerator, [1, kp, ki, kd])

    t, y = step(tf)
    twoPercentage = 0.02 * y[-1]
    i = len(y) - 1
    while (y[-1] - twoPercentage) < y[i] < (y[-1] + twoPercentage):
        i -= 1
    return t[i]


initial_guess = [1.0, 1.0, 1.0]

result = minimize(objective_function, initial_guess, method="nelder-mead")
optimal_gains = result.x
optimal_tf = TransferFunction(numerator, [1, *optimal_gains])


_, y1 = step(sys)
t, y = step(optimal_tf)
factor = y[-1] / y1[-1]
y1 = [i / y1[-1] for i in y1]
y = [i / y[-1] for i in y]

twoPercentage1 = 0.02 * y1[-1]
i = len(y1) - 1
while i > 0 and (y1[-1] - twoPercentage1) < y1[i] < (y1[-1] + twoPercentage1):
    i -= 1
st1 = t[i]
print(f"Original Settling Time: {st1}sec")

twoPercentage = 0.02 * y[-1]
i = len(y) - 1
while i > 0 and (y[-1] - twoPercentage) < y[i] < (y[-1] + twoPercentage):
    i -= 1
st = t[i]
print(f"Optimized Settling Time: {st}sec")


plt.plot(t, y1)
plt.plot(t, y)
plt.title(f"Optimal Step Response (Optimized Settling Time: {st:.4f}sec)")
plt.xlabel("Time")
plt.ylabel("Amplitude")
plt.grid()
plt.show()

print("Optimal PID Controller Gains:", optimal_gains)
