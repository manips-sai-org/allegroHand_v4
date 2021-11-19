#!/bin/bash

# uncomment to set nice and RT priority
# # start screen session in detached mode
# # note: this is to get around scripts running in non-interactive subshell
# # screen -dmS $SCREEN_NAME $COMMAND_NAME $ARGUMENTS
# sudo screen -dmS grasp sudo nice -n -39 ./grasp # default nice is 19 (should be 0)

# # wait until process exists
# wait $(pidof grasp)

# # set process to RT priority
# sudo chrt -ap 99 $(pidof grasp)

# # reattach original (interactive) screen session
# sudo screen -r grasp

sudo nice -n -39 ./grasp