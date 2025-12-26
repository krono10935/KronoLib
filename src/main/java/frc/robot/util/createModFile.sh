#!/bin/bash

FILE_PATH="/home/lvuser/isPit.txt"

# Ensure the home directory exists
mkdir -p /home/lvuser

# Create the file if it doesn't exist
if [ ! -f "$FILE_PATH" ]; then
    touch "$FILE_PATH"
    chmod 644 "$FILE_PATH"
fi
