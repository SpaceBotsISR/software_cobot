import numpy as np
import scipy.optimize as sopt
import matplotlib.pyplot as plt

def generate_data(a, b, noise_std, num_points):
    x = np.random.rand(num_points)
    noise = np.random.normal(0, noise_std, num_points)
    y = a * x + b + noise
    return x, y

def cost_function(params, x, y):
    c, d = params
    residuals = c * x + d - y
    return residuals

def main():
    a = 2.0
    b = 1.0
    noise_std = 0.2
    num_points = 100

    x, y = generate_data(a, b, noise_std, num_points)

    initial_params = np.array([1.0, 0.0])

    result = sopt.least_squares(cost_function, initial_params, args=(x, y))

    optimized_c, optimized_d = result.x

    print("True Parameters:")
    print(f"a: {a}, b: {b}")
    print("\nOptimized Parameters:")
    print(f"c: {optimized_c}, d: {optimized_d}")

    plt.scatter(x, y, label="Data")
    plt.plot(x, optimized_c * x + optimized_d, color='red', label="Fitted Line")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.legend()
    plt.show()

if __name__ == "__main__":
    main()
