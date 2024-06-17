import numpy as np
import csv

# Load the .npy file
npy_file = 'save-flight-09.01.2023_15.07.16.npy'  # Replace with the path to your .npy file
loaded_data = np.load(npy_file, allow_pickle=True)  # Load as a dictionary

# Define header for the CSV file
header = ["x", "y", "z", "Rx", "Ry", "Rz"]

# Open the CSV file for writing
csv_file = 'output.csv'

with open(csv_file, 'w', newline='') as csvfile:
    csvwriter = csv.writer(csvfile)

    # Iterate through drones
    for drone_index in range(5):
        drone_data = loaded_data["states"][drone_index]

        # Write the header for the drone's path
        #csvwriter.writerow('\n')  # Write the column headers
        csvwriter.writerow([])
        csvwriter.writerow([f'Drone {drone_index + 1} Path'])
        # csvwriter.writerow(header)  # Write the column headers

        # Write the data for the drone's path
        for i in range(len(drone_data[0])):
            # row = [drone_data[j][i] for j in [0, 1, 2, 7, 8, 6]]
            row = [drone_data[j][i] if not np.isnan(drone_data[j][i]) else '' for j in [0, 2, 1, 7, 8, 6]]   # Flipped 2 and 1 because y and z are flipped in unity
            csvwriter.writerow(row)

print(f"Data for all drones converted to CSV and saved as {csv_file}")
