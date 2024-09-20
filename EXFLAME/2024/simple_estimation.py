import numpy as np
import pandas as pd

# Load measurements from CSV
df = pd.read_csv('Git_clone/data_stationary.csv')

for index, row in df.iterrows():
    x, y, z = [row['X in mm'], row['Y in mm'], row['Z in mm']]
    