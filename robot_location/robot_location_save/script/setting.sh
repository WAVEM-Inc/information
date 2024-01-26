#!/bin/bash

# 사용자의 홈 디렉토리 확인
user_home="/home/$USER"
# RobotData 디렉토리 경로 설정
robot_data_dir="$user_home/RobotData"

# 디렉토리가 존재하지 않으면 생성
if [ ! -d "$robot_data_dir" ]; then
    echo 1234 | sudo -S mkdir "$robot_data_dir"
    echo "RobotData 디렉토리가 생성되었습니다."
else
    echo "RobotData 디렉토리는 이미 존재합니다."
fi

