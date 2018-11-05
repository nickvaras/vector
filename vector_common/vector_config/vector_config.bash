# This configures the environment variables for a vector simulation
# This is necessary to run before starting the simulation 
#

# If there is an onboard PC powered by the system this will run the watchdog
# to make sure it gets gracefully shutdown before the system power cuts out.
export VECTOR_POWERS_PC_ONBOARD=true

#Default Vector network 
export VECTOR_IP_ADDRESS=10.66.171.5
export VECTOR_IP_PORT_NUM=8080

# Joystick input types are:
# - ds4  - The Sony DualShock4 Joystick for PS4 (model: CUH-ZCT2U)
# - f710 - The Logitech F710 Wireless Joystick (in XINPUT mode)
# - f310 - The Logitech F310 Wired Joystick (in XINPUT mode)
# - (not supported yet) x3dp - The Logitech Extreme 3D Pro Joystick (not supported yet)   
export VECTOR_JOY_TYPE=f310

# Joystick deadzone (smaller value increases sensitivity to small movements)
export VECTOR_JOY_DEADZONE=0.1

# Joystick configurations for joystick set VECTOR_JOY_IS_ATTACHED if the joystick
# is physically attached to this PC
if [ "$HOSTNAME" = vector1 ]; then
    export VECTOR_JOY_IS_ATTACHED=true
    export VECTOR_JOY_DEV=/dev/input/js0
    
else
    export VECTOR_JOY_IS_ATTACHED=false
    if [ "$VECTOR_JOY_TYPE" = ds4 ]; then
        export VECTOR_JOY_DEV=/dev/input/js1
    else
        export VECTOR_JOY_DEV=/dev/input/js0
    fi
fi

# Used to determine if the system is equipped with saftey lasers 
export VECTOR_HAS_SAFETY_LASERS=true

# Used to determine if the system is equipped with a vlp16
export VECTOR_HAS_VLP16=false

# Used to determine if the system is equipped with a webcam for remote video
export VECTOR_HAS_WEBCAM=false

# Used to determine if the system is equipped with a realsense depth camera
export VECTOR_HAS_REALSENSE=true