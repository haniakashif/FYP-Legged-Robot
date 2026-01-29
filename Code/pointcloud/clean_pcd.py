#!/usr/bin/env python3

import sys

input_file = "/home/syn/ros2_jazzy/my_scan116.953000000.pcd"  # original with inf
output_file = "/home/syn/ros2_jazzy/my_scan_fixed.pcd"

with open(input_file, 'r') as f:
    lines = f.readlines()

# Find header end (DATA ascii line)
data_start = next(i for i, line in enumerate(lines) if line.strip() == "DATA ascii") + 1

# Copy header
header = lines[:data_start]

# Process data: replace "inf inf inf" with "nan nan nan" to keep organized structure
data_lines = []
for line in lines[data_start:]:
    stripped = line.strip()
    if stripped == "inf inf inf":
        data_lines.append("nan nan nan\n")
    else:
        data_lines.append(line)

# Header remains unchanged (POINTS 307200, WIDTH 640, HEIGHT 480)
with open(output_file, 'w') as f:
    f.writelines(header)
    f.writelines(data_lines)

print(f"Fixed PCD saved to {output_file}")
print("Organized structure preserved (640x480), inf replaced with nan.")