#!/bin/bash

####################################
##           Hall Monitor         ##
## Enthusiastic Service Restarter ##
####################################

SERVICE_NAME="unreliable_service"
DELAY=10

if ! command -v systemctl &> /dev/null; then
    echo "systemctl command not found. Exiting."
    exit 1
fi

while true; do
    if ! systemctl is-active --quiet ${SERVICE_NAME}; then
        echo "${SERVICE_NAME} is not running. Starting service..."
        if ! systemctl start ${SERVICE_NAME} &> /dev/null; then
            echo "Failed to start ${SERVICE_NAME}. Retrying in ${DELAY} seconds..."
        else
            echo "${SERVICE_NAME} started successfully."
        fi
    else
        service_output=$(systemctl status ${SERVICE_NAME} 2>&1)
        if echo "${service_output}" | grep -q "SerialException: write failed"; then
            echo "Error detected in ${SERVICE_NAME}. Restarting service..."
            if ! systemctl restart ${SERVICE_NAME} &> /dev/null; then
                echo "Failed to restart ${SERVICE_NAME}. Retrying in ${DELAY} seconds..."
            else
                echo "${SERVICE_NAME} restarted successfully."
            fi
        fi
    fi
    sleep ${DELAY}
done
