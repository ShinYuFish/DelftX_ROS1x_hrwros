#!/usr/bin/env bash

SIM_LINK="${HOME}/hrwros_gazebo_models"
GILBRETH_MODEL_PATH="$(rospack find hrwros_gazebo)/models"
echo $SIM_LINK

# check existence
if [ -e "${SIM_LINK}" ]; then
	rm $SIM_LINK
fi

# echo "Creating Symbolic Link to path $GILBRETH_MODEL_PATH "
ln -s "$GILBRETH_MODEL_PATH" "$SIM_LINK"

# create env variable
export GAZEBO_MODEL_PATH="$SIM_LINK"

exec "$@"

