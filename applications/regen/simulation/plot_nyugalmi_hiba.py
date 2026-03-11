import pandas as pd
import matplotlib.pyplot as plt

# Read Excel file, rows 5500 to 7500 (Excel is 1-indexed, pandas uses 0-indexing)
# skiprows skips rows 0-5499, nrows reads 2001 rows (5500 to 7500 inclusive)
df = pd.read_excel('nyugalmi_hiba.xlsx', skiprows=5499, nrows=2001, header=None)

# Convert all columns to numeric, coerce errors to NaN
df = df.apply(pd.to_numeric, errors='coerce')

# Drop rows with NaN values
df = df.dropna()

# Store each column into separate arrays
# Assuming no header in the data range, columns are 0-indexed
col_A = df.iloc[:, 0].values  # Column A (index 0)
col_B = df.iloc[:, 1].values  # Column B (index 1)
col_C = df.iloc[:, 2].values  # Column C (index 2)
col_D = df.iloc[:, 3].values  # Column D (index 3)
col_E = df.iloc[:, 4].values  # Column E (index 4)
col_F = df.iloc[:, 5].values  # Column F (index 5)
col_G = df.iloc[:, 6].values  # Column G (index 6)

# Create plot
plt.figure(figsize=(10, 6))
plt.plot(col_A, col_D, label='Column D', marker='o', markersize=2, linestyle='-')
plt.plot(col_A, col_E, label='Column E', marker='s', markersize=2, linestyle='-')
plt.plot(col_A, col_G, label='Column G', marker='^', markersize=2, linestyle='-')

plt.xlabel('Column A')
plt.ylabel('Values')
plt.title('Columns D, E, G vs Column A (Rows 5500-7500)')
plt.legend()
plt.grid(True, alpha=0.3)
plt.tight_layout()

# Display the plot
plt.show()
