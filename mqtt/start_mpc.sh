#!/bin/bash
# Startup script for MPC Controller on Mac

echo "=== EcoLead Distributed MPC Controller ==="
echo "Starting MPC controller on Mac..."

# Check if Mosquitto is running
if ! pgrep -x "mosquitto" > /dev/null; then
    echo "Starting Mosquitto MQTT broker..."
    brew services start mosquitto
    sleep 2
fi

# Set default MQTT broker to localhost
export MQTT_BROKER=${MQTT_BROKER:-localhost}
export MQTT_PORT=${MQTT_PORT:-1883}

echo "MQTT Broker: $MQTT_BROKER:$MQTT_PORT"

# Check Python dependencies
python -c "import paho.mqtt.client, casadi, numpy, yaml" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "Error: Missing Python dependencies"
    echo "Install with: pip install paho-mqtt casadi numpy pyyaml"
    exit 1
fi

# Start MPC controller
echo "Starting distributed MPC controller..."
python distributed_mpc_controller.py
