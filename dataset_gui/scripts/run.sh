#!/bin/bash

# NOTE: you need to call this with the first parameter in "" if there is a * to select multiple files

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

input_folder="$(pwd)/"

echo -e "${GREEN}Running dataset playback with initial folder $input_folder${NC}"

roslaunch dataset_gui run.launch folder:=$input_folder
