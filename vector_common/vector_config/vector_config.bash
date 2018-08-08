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
export VECTOR_JOY_IS_ATTACHED=false
export VECTOR_JOY_DEV=/dev/input/js0
export VECTOR_JOY_MAPPING=xbox360
export VECTOR_JOY_DEADZONE=0.1

#Used to determine if the system is equipped with saftey lasers 
export VECTOR_HAS_SAFETY_LASERS=false

# laser configuration (only supports two by default)
export LASER1_XYZ="0.3385 0 0.1439106"
export LASER1_RPY="3.1415 0 0"
export LASER1_MAX_RANGE=20.0
export LASER1_MIN_RANGE=0.05
export LASER1_MAX_ANGLE=2.0
export LASER1_MIN_ANGLE=-2.0

export LASER2_XYZ="-0.3385 0 0.1439106"
export LASER2_RPY="3.1415 0 3.1415"
export LASER2_MAX_RANGE=10.0
export LASER2_MIN_RANGE=0.01
export LASER2_MAX_ANGLE=2.0
export LASER2_MIN_ANGLE=-2.0


