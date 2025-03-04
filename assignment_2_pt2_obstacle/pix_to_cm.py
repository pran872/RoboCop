import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

# Given data: pixel distances (Y-axis) and corresponding real-world cm distances
pixel_values_3 = np.array([0, 37, 95, 178, 335, 480])  # Y-coordinates in pixels
cm_values_3 = np.array([35, 31.5, 25.5, 19.5, 13.5, 9.5])

pixel_values_2 = np.array([0, 7, 71, 160, 293, 480])  # Y-coordinates in pixels
cm_values_2 = np.array([33, 32.5, 26.5, 20.5, 14.5, 8.5])  # Corresponding real-world cm

pixel_values_1 = np.array([0, 50, 119, 220, 355, 480])
cm_values_1 = np.array([26.5, 20.5, 14.5, 8.5, 2.5, 0])

# Define a quadratic function: cm = a * pix^2 + b * pix + c
def quadratic(x, a, b, c):
    return a * x**2 + b * x + c

params_3, _ = curve_fit(quadratic, pixel_values_3, cm_values_3)
x_fit_3 = np.linspace(0, 500, 100)  # Pixel range for fitting
y_fit_3 = quadratic(x_fit_3, *params_3)
print(f"{params_3[0]:.6f}x² + {params_3[1]:.6f}x + {params_3[2]:.6f}")

# Fit the quadratic function to the data
params_2, _ = curve_fit(quadratic, pixel_values_2, cm_values_2)
x_fit_2 = np.linspace(0, 500, 100)  # Pixel range for fitting
y_fit_2 = quadratic(x_fit_2, *params_2)
print(f"{params_2[0]:.6f}x² + {params_2[1]:.6f}x + {params_2[2]:.6f}")

params_1, _ = curve_fit(quadratic, pixel_values_1, cm_values_1)
x_fit_1 = np.linspace(0, 500, 100)  # Pixel range for fitting
y_fit_1 = quadratic(x_fit_1, *params_1)
print(f"{params_1[0]:.6f}x² + {params_1[1]:.6f}x + {params_1[2]:.6f}")

# old_eq = lambda x: 0.000105*x**2 + -0.104068*x + 25.931873
# y_old2 = [old_eq(x) for x in x_fit]
# Plot data points and fitted curve
plt.scatter(pixel_values_3, cm_values_3, color="red")
plt.scatter(pixel_values_2, cm_values_2, color="blue")
plt.scatter(pixel_values_1, cm_values_1, color="orange")
plt.plot(x_fit_3, y_fit_3, label=f"Fitted Curve: {params_3[0]:.6f}x² + {params_3[1]:.6f}x + {params_3[2]:.6f}", color="red")
plt.plot(x_fit_2, y_fit_2, label=f"Fitted Curve: {params_2[0]:.6f}x² + {params_2[1]:.6f}x + {params_2[2]:.6f}", color="blue")
plt.plot(x_fit_1, y_fit_1, label=f"Fitted Curve: {params_1[0]:.6f}x² + {params_1[1]:.6f}x + {params_1[2]:.6f}", color="orange")
plt.xlabel("Pixel Position (Y)")
plt.ylabel("Distance (cm)")
plt.title("Pixel-to-CM Mapping in Y Plane")
plt.legend()
plt.grid(True)
plt.show()



# Return the quadratic equation parameters
params_3


# pix_to_cm = lambda x: 0.000105*x**2 + -0.104068*x + 25.931873
# pix_to_cm = lambda x: 0.000076*x**2 + -0.086957*x + 32.820661
pix_to_cm = lambda x: 0.000101*x**2 + -0.100282*x + 34.739524 

print(pix_to_cm(220))