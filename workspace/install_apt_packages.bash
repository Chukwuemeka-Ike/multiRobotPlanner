#!/bin/bash

# Define the path to the requirements file
REQUIREMENTS_FILE="apt_requirements.txt"

# Check if the requirements file exists
if [ -f "$REQUIREMENTS_FILE" ]; then
    # Read each line from the requirements file and install the packages
    while IFS= read -r package; do
        sudo apt-get install -y "$package"
    done < "$REQUIREMENTS_FILE"
else
    echo "Requirements file not found: $REQUIREMENTS_FILE"
fi
