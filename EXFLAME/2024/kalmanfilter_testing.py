import numpy as np
import pandas as pd

# Load measurements from CSV
df = pd.read_csv('Git_clone/data_stationary.csv')

# Initial State (X, Y, Z in mm)
x = np.array([[0], [0], [0]])

# Initial Covariance Matrix
P = np.eye(3) * 1e-3

# State Transition Matrix (Identity for stationary)
F = np.eye(3)

# Measurement Matrix (Identity, as we directly measure position)
H = np.eye(3)

# Process Noise Covariance Matrix (very small values)
Q = np.eye(3) * 1e-5

# Measurement Noise Covariance Matrix (based on your stereo matching)
R = np.array([[1e-2, 0, 0],
              [0, 1e-2, 0],
              [0, 0, 1e-2]])

# Kalman Filter Loop through measurements from CSV
for index, row in df.iterrows():
    z = np.array([row['X in mm'], row['Y in mm'], row['Z in mm']]).reshape(3, 1)

    # Prediction Step
    x = np.dot(F, x)
    P = np.dot(F, np.dot(P, F.T)) + Q

    # Measurement Update
    y = z - np.dot(H, x)
    S = np.dot(H, np.dot(P, H.T)) + R
    K = np.dot(P, np.dot(H.T, np.linalg.inv(S)))
    x = x + np.dot(K, y)
    P = P - np.dot(K, np.dot(H, P))

    print(f"Estimated State at time {row['Time in ms']} ms:\n", x)

# Final Estimated State from Kalman Filter
final_estimated_state = x.flatten()
print("Final Estimated State:\n", final_estimated_state)

# Calculate the average values for X, Y, Z
average_values = df[['X in mm', 'Y in mm', 'Z in mm']].mean()
print("Average Values from CSV:\n", average_values)

# Calculate the difference between the final estimated state and the average values
difference = final_estimated_state - average_values.to_numpy()
print("Difference between Final Estimated State and Average Values:\n", difference)