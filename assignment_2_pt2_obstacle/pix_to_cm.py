import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

# Given data: pixel distances (Y-axis) and corresponding real-world cm distances
pixel_values = np.array([0, 50, 119, 220, 355, 480])  # Y-coordinates in pixels
cm_values = np.array([26.5, 20.5, 14.5, 8.5, 2.5, 0])  # Corresponding real-world cm

# Define a quadratic function: cm = a * pix^2 + b * pix + c
def quadratic(x, a, b, c):
    return a * x**2 + b * x + c

# Fit the quadratic function to the data
params, _ = curve_fit(quadratic, pixel_values, cm_values)

# Generate a smooth curve for visualization
x_fit = np.linspace(0, 500, 100)  # Pixel range for fitting
y_fit = quadratic(x_fit, *params)
print(f"{params[0]:.6f}x² + {params[1]:.6f}x + {params[2]:.6f}")

# Plot data points and fitted curve
plt.scatter(pixel_values, cm_values, color="red", label="Measured Points")
plt.plot(x_fit, y_fit, label=f"Fitted Curve: {params[0]:.6f}x² + {params[1]:.6f}x + {params[2]:.6f}", color="blue")
plt.xlabel("Pixel Position (Y)")
plt.ylabel("Distance (cm)")
plt.title("Pixel-to-CM Mapping in Y Plane")
plt.legend()
plt.grid(True)
plt.show()

# Return the quadratic equation parameters
params
