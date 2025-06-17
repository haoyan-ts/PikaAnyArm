#!/bin/bash

# Check if an argument was provided
if [ $# -eq 0 ]; then
    echo "Usage: $0 <replacement_path>"
    echo "Example: $0 /home/user"
    exit 1
fi

# Get the replacement string from the first argument
REPLACEMENT="$1"
PATTERN="/home/agilex"

# Define the target directory
TARGET_DIR="piper_ros/piper_description"

# Check if target directory exists
if [ ! -d "$TARGET_DIR" ]; then
  echo "Error: Target directory '$TARGET_DIR' not found!"
  exit 1
fi

# Find all files that contain the pattern and replace it
echo "Searching for URDF files containing '$PATTERN' in '$TARGET_DIR'..."
echo "These files will be modified to use '$REPLACEMENT' instead."
echo ""

# Count of files modified
COUNT=0

# Use grep to find files, then use sed to replace the string in each file
while IFS= read -r file; do
    # Make sure the file exists and is readable
    if [ -f "$file" ] && [ -r "$file" ]; then
        # Create a backup of the original file
        cp "$file" "${file}.bak"
        
        # Replace the pattern in the file
        sed -i "s|$PATTERN|$REPLACEMENT|g" "$file"
        
        # Check if any replacements were made
        if diff -q "${file}.bak" "$file" >/dev/null; then
            # No changes were made, remove the backup
            rm "${file}.bak"
        else
            COUNT=$((COUNT + 1))
            echo "Modified: $file"
        fi
    fi
done < <(grep -l "$PATTERN" --include="*.urdf" --include="*.xacro" -r "$TARGET_DIR")

echo ""
echo "Replacement complete. Modified $COUNT files."
echo "If needed, you can find backups of the modified files with '.bak' extension."
