#!/bin/bash
screen -dmS endpoint bash start_ros_tcp_endpoint.sh
screen -dmS h1_publisher bash start_h1_publisher.sh
screen -dmS h1_receiver bash start_h1_receiver.sh
screen -dmS h1_controller bash start_h1_controller.sh