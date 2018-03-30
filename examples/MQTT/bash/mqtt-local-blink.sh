#!/bin/bash
# Script for testing MQTT channel on Linux machine
# mosquitto_pub is using MQTT local broker to communicate to IQRF Arduino gateway

echo "sending requests to pulse red led on node 0"
mosquitto_pub -t "dpaGateIn" -m "{\"ctype\":\"dpa\",\"type\":\"raw\",\"msgid\":\"1\",\"timeout\":1000,\"request\":\"00.00.06.03.ff.ff\",\"request_ts\":\"\",\"confirmation\":\".\",\"confirmation_ts\":\"\",\"response\":\".\",\"response_ts\":\"\"}"
#mosquitto_pub -t "dpaGateIn" -m "{\"ctype\":\"dpa\",\"type\":\"raw\",\"msgid\":\"1\",\"timeout\":1000,\"request\":\"00.00.06.03.ff.ff\"}"
