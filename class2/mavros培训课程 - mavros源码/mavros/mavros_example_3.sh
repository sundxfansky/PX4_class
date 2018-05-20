gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch mavros px4.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; rosrun mavros mavros_example_3; exec bash"' \





