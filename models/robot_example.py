from kfplusplus import (
    ExtendedKalmanFilter5x6,
    Vector2, Vector5, Vector6,
    Matrix2x2, Matrix2x5, Matrix5x5
)
import math
import random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches


# Parameters
random.seed(42)
np.random.seed(42)
STATE_DIM = 5  # [x, y, θ, v, ω]
CONTROL_DIM = 6  # [vx, vy, vz, wx, wy, wz]
IMU_MEASUREMENT_DIM = 2  # [θ, ω]
ODOM_MEASUREMENT_DIM = 5  # [x, y, θ, v, ω]
dt = 1.0
steps = 100
ENABLE_OPTIMIZATION = True  # Toggle optimization for Q

# Measurement noise (R) - typically known from sensor specs
R_IMU = 0.1
R_ODOM = 0.1

# Process noise (Q)
Q_VALUE = 1e-5

def plot_covariance_ellipse(ax, mean, cov, n_std=1.0, facecolor='none',  point_color='black', **kwargs):
    """Plot a covariance ellipse centered at mean with covariance matrix cov."""
    # Eigen decomposition of the 2x2 covariance submatrix
    eigvals, eigvecs = np.linalg.eigh(cov)
    order = eigvals.argsort()[::-1]
    eigvals = eigvals[order]
    eigvecs = eigvecs[:,order]

    # angle of ellipse rotation in degrees
    angle = np.degrees(np.arctan2(eigvecs[1,0], eigvecs[0,0]))
    
    # Width and height of ellipse = 2*n_std*sqrt(eigenvalue)
    width, height = 2 * n_std * np.sqrt(eigvals)
    
    ellipse = patches.Ellipse(xy=mean, width=width, height=height,
                              angle=angle, facecolor=facecolor, **kwargs)
    ax.add_patch(ellipse)
    ax.scatter(mean[0], mean[1], color=point_color, s=15, zorder=10)

# Generate IMU and odometry data
def generate_sensor_data(steps, dt):
    ground_truth = []
    imu_measurements = []
    odom_measurements = []
    x, y, theta = 0.0, 0.0, 0.0
    v, omega = 1.0, 0.1

    for i in range(steps):
        t = i * dt
        # Update ground truth state
        x += v * dt * math.cos(theta)
        y += v * dt * math.sin(theta)
        theta += omega * dt
        ground_truth.append([x, y, theta, v, omega])

        # IMU measurements: [θ, ω]
        noisy_theta = theta + random.gauss(0.0, 0.1)
        noisy_omega = omega + random.gauss(0.0, 0.1)
        imu_meas = Vector2()
        imu_meas[0] = noisy_theta
        imu_meas[1] = noisy_omega
        imu_measurements.append(imu_meas)

        # Odometry measurements: [x, y, θ, v, ω]
        noisy_x = x + random.gauss(0.0, 0.1)
        noisy_y = y + random.gauss(0.0, 0.1)
        noisy_v = v + random.gauss(0.0, 0.1)
        noisy_theta = theta + random.gauss(0.0, 0.1)
        noisy_omega = omega + random.gauss(0.0, 0.1)
        odom_meas = Vector5()
        odom_meas[0] = noisy_x
        odom_meas[1] = noisy_y
        odom_meas[2] = noisy_theta
        odom_meas[3] = noisy_v
        odom_meas[4] = noisy_omega
        odom_measurements.append(odom_meas)

    return ground_truth, imu_measurements, odom_measurements

# Generate IMU and odometry data with sinusoidal listing motion
def generate_sensor_data_with_listing(steps, dt):
    ground_truth = []
    imu_measurements = []
    odom_measurements = []
    x, y, theta = 0.0, 0.0, 0.0
    v, omega = 1.0, 0.1
    amplitude = 0.1  # Maximum deviation in radians
    frequency = 0.05  # Oscillation frequency

    for i in range(steps):
        t = i * dt
        # Oscillate heading: listing left then right
        theta = amplitude * math.sin(frequency * t)
        # Update position using the current heading
        x += v * dt * math.cos(theta)
        y += v * dt * math.sin(theta)
        ground_truth.append([x, y, theta, v, omega])

        # IMU measurements: [θ, ω]
        true_omega = amplitude * frequency * math.cos(frequency * t)  # Derivative of theta
        noisy_theta = theta + random.gauss(0.0, 0.1)
        noisy_omega = true_omega + random.gauss(0.0, 0.1)
        imu_meas = Vector2()
        imu_meas[0] = noisy_theta
        imu_meas[1] = noisy_omega
        imu_measurements.append(imu_meas)

        # Odometry measurements: [x, y, θ, v, ω]
        noisy_x = x + random.gauss(0.0, 0.1)
        noisy_y = y + random.gauss(0.0, 0.1)
        noisy_v = v + random.gauss(0.0, 0.1)
        noisy_theta = theta + random.gauss(0.0, 0.1)
        noisy_omega = omega + random.gauss(0.0, 0.1)
        odom_meas = Vector5()
        odom_meas[0] = noisy_x
        odom_meas[1] = noisy_y
        odom_meas[2] = noisy_theta
        odom_meas[3] = noisy_v
        odom_meas[4] = noisy_omega
        odom_measurements.append(odom_meas)

    return ground_truth, imu_measurements, odom_measurements

# State transition model
def state_transition(state, control):
    x, y, theta, v, omega = [state[i] for i in range(STATE_DIM)]
    vx, vy, vz, wx, wy, wz = [control[i] for i in range(CONTROL_DIM)]
    new_state = Vector5()
    new_state[0] = x + v * dt * math.cos(theta)
    new_state[1] = y + v * dt * math.sin(theta)
    new_state[2] = theta + omega * dt
    new_state[3] = vx
    new_state[4] = wz
    return new_state

# Jacobian of the state transition model
def jacobian_transition(state, control):
    theta = state[2]
    v = state[3]
    jacobian = Matrix5x5.identity()
    jacobian[0, 2] = -v * dt * math.sin(theta)
    jacobian[0, 3] = dt * math.cos(theta)
    jacobian[1, 2] = v * dt * math.cos(theta)
    jacobian[1, 3] = dt * math.sin(theta)
    jacobian[2, 4] = dt
    return jacobian

# IMU measurement function
def imu_measurement_function(state):
    meas = Vector2()
    meas[0] = state[2]  # θ
    meas[1] = state[4]  # ω
    return meas

# Jacobian of the IMU measurement function
def imu_jacobian_measurement(state):
    H = Matrix2x5()
    H[0, 2] = 1.0  # d(measurement)/d(θ)
    H[1, 4] = 1.0  # d(measurement)/d(ω)
    return H

# Odometry measurement function
def odom_measurement_function(state):
    meas = Vector5()
    for i in range(STATE_DIM):
        meas[i] = state[i]
    return meas

# Jacobian of the odometry measurement function
def odom_jacobian_measurement(state):
    return Matrix5x5.identity()

# Run EKF with IMU and odometry measurements
def run_ekf(q_value, r_imu, r_odom, cov_value=1.0):
    ekf = ExtendedKalmanFilter5x6()
    initial_state = Vector5()
    initial_state[0] = 0.0
    initial_state[1] = 0.0
    initial_state[2] = 0.0
    initial_state[3] = 1.0
    initial_state[4] = 0.1
    ekf.set_state(initial_state)
    ekf.set_covariance(Matrix5x5.identity() * cov_value)
    ekf.set_process_noise(Matrix5x5.identity() * q_value)

    ekf_x, ekf_y, innovations, covariances = [], [], [], []
    for i in range(steps):
        # Predict step
        control = Vector6()
        control[0] = initial_state[3]  # vx = v
        control[5] = initial_state[4]  # wz = ω
        ekf.predict(state_transition, jacobian_transition, control)

        # Update with IMU measurements
        ekf.update_imu(
            imu_measurements[i],
            Matrix2x2.identity() * r_imu,
            imu_measurement_function,
            imu_jacobian_measurement
        )

        # Update with odometry measurements
        ekf.update_odom(
            odom_measurements[i],
            Matrix5x5.identity() * r_odom,
            odom_measurement_function,
            odom_jacobian_measurement
        )

        state = ekf.get_state()
        ekf_x.append(state[0])
        ekf_y.append(state[1])

        # Compute innovation (measurement residual)
        pred_meas = odom_measurement_function(state)
        innovation = np.sqrt((odom_measurements[i][0] - pred_meas[0])**2 +
                             (odom_measurements[i][1] - pred_meas[1])**2)
        innovations.append(innovation)

        # Store covariance (diagonal elements)
        covariances.append([ekf.get_covariance()[i, i] for i in range(STATE_DIM)])

    return np.array(ekf_x), np.array(ekf_y), np.array(innovations), np.array(covariances)

# Compute MSE
def compute_mse(ekf_x, ekf_y, truth_x, truth_y):
    return np.mean((ekf_x - truth_x) ** 2 + (ekf_y - truth_y) ** 2)

# Optimization for Q
def optimize_q():
    best_q = None
    best_mse = float('inf')
    for q in np.linspace(1e-5, 0.01, 100):
        ekf_x, ekf_y, _, _ = run_ekf(q, R_IMU, R_ODOM)
        mse = compute_mse(ekf_x, ekf_y, truth_x, truth_y)
        if mse < best_mse:
            best_mse = mse
            best_q = q
    return best_q

# Optimization for Initial Covariance
def optimize_initial_covariance():
    best_cov = None
    best_mse = float('inf')
    for cov in np.linspace(0.01, 100.0, 100):  # Sweep initial covariance values
        ekf_x, ekf_y, _, _ = run_ekf(Q_VALUE, R_IMU, R_ODOM, cov)
        mse = compute_mse(ekf_x, ekf_y, truth_x, truth_y)
        if mse < best_mse:
            best_mse = mse
            best_cov = cov
    return best_cov

# Generate data
# ground_truth, imu_measurements, odom_measurements = generate_sensor_data(steps, dt)
ground_truth, imu_measurements, odom_measurements = generate_sensor_data_with_listing(steps, dt)

truth_x = np.array([gt[0] for gt in ground_truth])
truth_y = np.array([gt[1] for gt in ground_truth])

# Optimize Q, R, and Initial Covariance if enabled
if ENABLE_OPTIMIZATION:
    Q_VALUE = optimize_q()
    INITIAL_COVARIANCE = optimize_initial_covariance()
else:
    Q_VALUE = 1e-5  # Default Q value
    INITIAL_COVARIANCE = 1.0  # Default initial covariance

# Run EKF
ekf_x, ekf_y, innovations, covariances = run_ekf(Q_VALUE, R_IMU, R_ODOM, INITIAL_COVARIANCE)

# Plot results
plt.figure()
plt.plot(truth_x, truth_y, label="Ground Truth", linewidth=2, color='blue')
plt.plot(ekf_x, ekf_y, label="EKF Estimate", linestyle='--', color='orange')
plt.xlabel("x")
plt.ylabel("y")
plt.legend()
plt.title("EKF Trajectory vs Ground Truth")
plt.show()

# Plot innovations
plt.figure()
plt.plot(innovations, label="Innovations")
plt.xlabel("Step")
plt.ylabel("Innovation")
plt.title("Measurement Residuals (Innovations)")
plt.legend()
plt.show()

# Plot covariance ellipses every 10 steps
fig, ax = plt.subplots()

ax.plot(truth_x, truth_y, label="Ground Truth", linewidth=2, color='blue')
ax.plot(ekf_x, ekf_y, label=f"EKF (Q={Q_VALUE:.4e}, InitCov={INITIAL_COVARIANCE})", linestyle='--', color='orange')

for idx in range(0, steps, 10):
    # Get the 2x2 submatrix for positions (assumes state order [x, y, theta, v, omega])
    cov_matrix = np.array([[covariances[idx][0], covariances[idx][1]*0.0],   # Off-diagonals not computed here
                            [covariances[idx][1]*0.0, covariances[idx][1]]])
    # Note: If you stored the full 5x5 covariance matrices properly,
    # you should extract the (0,0), (0,1), (1,0), (1,1) components.
    # Here we use the diagonal approximations for demonstration.
    mean = (ekf_x[idx], ekf_y[idx])
    plot_covariance_ellipse(ax, mean, cov_matrix, n_std=1, edgecolor='red', alpha=0.5)

ax.set_xlabel("x")
ax.set_ylabel("y")
ax.legend()
ax.set_title("EKF Trajectory vs Ground Truth with Covariance Ellipses")
plt.show()

