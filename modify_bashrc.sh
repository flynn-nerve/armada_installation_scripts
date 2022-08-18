echo -e "\n# more custom aliases
alias build='cd ~/catkin_ws && catkin build && source devel/setup.bash && cd -'
alias sourcethis='cd ~/catkin_ws && source devel/setup.bash && cd -'
alias ws='cd ~/catkin_ws'

cd ~/catkin_ws
source devel/setup.bash
clear" >> ~/.bashrc
