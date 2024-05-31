#!/bin/sh

source ../install/setup.bash
# Set Session Name
SESSION="Testing"
SESSIONEXISTS=$(tmux list-sessions | grep $SESSION)

# Only create tmux session if it doesn't already exist
if [ "$SESSIONEXISTS" = "" ]
then
        window="window"
        # Start New Session with our name
        tmux new-session -d -s $SESSION

        # Name first Pane and start zsh
        tmux rename-window -t 0 "$window"

        tmux split-window -t "$SESSION:$window" -t 0
        tmux split-window -t "$SESSION:$window" -t 0 -h
        tmux split-window -t "$SESSION:$window" -t 2 -h
        tmux split-window -t "$SESSION:$window" -t 2 
        tmux split-window -t "$SESSION:$window" -t 3 

        tmux send-keys -t "$SESSION:$window" -t 0 'source ../install/setup.bash' enter
        tmux send-keys -t "$SESSION:$window" -t 0 'sudo docker run -it --rm --privileged --net=host microros/micro-ros-agent:humble serial --dev /dev/ttyACM0' enter

        tmux send-keys -t "$SESSION:$window" -t 1 'sudo MicroXRCEAgent serial --dev /dev/serial0 -b 921600' enter

        tmux send-keys -t "$SESSION:$window" -t 2 'source ../install/setup.bash' enter
        tmux send-keys -t "$SESSION:$window" -t 2 'ros2 run px4_offboard_control offboard_node' enter

        tmux send-keys -t "$SESSION:$window" -t 3 'source ../install/setup.bash' enter
        tmux send-keys -t "$SESSION:$window" -t 3 'ros2 run path_planner local_planner' enter


        tmux send-keys -t "$SESSION:$window" -t 4 'source ../install/setup.bash' enter
        tmux send-keys -t "$SESSION:$window" -t 4 'ros2 run mission_manager mission' enter

        # tmux send-keys -t "$SESSION:$window" -t 5 'source ../install/setup.bash' enter
        # tmux send-keys -t "$SESSION:$window" -t 5 'ros2 run computer_vision dummy_cv' enter

        tmux select-layout tiled

fi
# Attach Session, on the Main window
tmux attach-session -t $SESSION
