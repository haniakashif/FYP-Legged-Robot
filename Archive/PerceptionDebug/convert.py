import os
from pathlib import Path

def convert_py_to_txt(source_dir, output_dir):
    # Create output directory if it doesn't exist
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print(f"Created directory: {output_dir}")

    # Iterate through all files in the source directory
    for file_path in Path(source_dir).glob('*.py'):
        try:
            # Read the python file content
            content = file_path.read_text(encoding='utf-8')

            # Define the new filename
            txt_filename = f"{file_path.stem}.txt"
            target_path = Path(output_dir) / txt_filename

            # Write the content to the text file
            target_path.write_text(content, encoding='utf-8')
            print(f"Converted: {file_path.name} -> {txt_filename}")

        except Exception as e:
            print(f"Failed to convert {file_path.name}: {e}")

# Usage
convert_py_to_txt('/mnt/windows/FYP-Legged-Robot/Archive/PerceptionDebug/', '//mnt/windows/FYP-Legged-Robot/Archive/PerceptionDebug/path')
