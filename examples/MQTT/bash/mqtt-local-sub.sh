#!/bin/bash
# Script for testing MQTT channel on Linux machine
# mosquitto_sub is using local MQTT broker to communicate to IQRF Arduino gateway

echo "Listening for DPA responses from IQRF network:"
mosquitto_sub -t "dpaGateOut"
