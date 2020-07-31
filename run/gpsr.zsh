tmux split-window -v
tmux split-window -h
tmux split-window -v

tmux send-keys -t 0.1 'source ../../devel/setup.zsh && roslaunch pysot_tracker test_track.launch' Enter
sleep 4
tmux send-keys -t 0.2 'source ../../devel/setup.zsh && roslaunch darknet_ros yolo_final.launch' Enter
sleep 8
tmux send-keys -t 0.3 'source ../../devel/setup.zsh && roslaunch nav_ros nav_test.launch' Enter
sleep 5
tmux send-keys -t 0.0 'source ../../devel/setup.zsh && roslaunch robot gpsr_sm.launch' Enter

tmux swap-pane -D

