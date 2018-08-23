# This configures the environment variables for a vector simulation
# This is necessary to run before starting the simulation 
#

# If there is an onboard PC powered by the system this will run the watchdog
# to make sure it gets gracefully shutdown before the system power cuts out.
export VECTOR_POWERS_PC_ONBOARD=true

#Default Vector RMP_V3 network 
export VECTOR_IP_ADDRESS=10.66.171.5
export VECTOR_IP_PORT_NUM=8080

# Joystick configurations for joystick set VECTOR_JOY_IS_ATTACHED if the joystick
# is physically attached to this PC
export VECTOR_JOY_IS_ATTACHED=true
export VECTOR_JOY_DEV=/dev/input/js1
export VECTOR_JOY_MAPPING=xbox360
export VECTOR_JOY_DEADZONE=0.1

#Used to determine if the system is equipped with saftey lasers 
export VECTOR_HAS_SAFETY_LASERS=false

export DISPATCHER_UI_LOCAL=true


