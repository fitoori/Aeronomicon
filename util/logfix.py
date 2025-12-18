#!/usr/bin/env python3

###############################
##  LogFix for WATNE Uplink  ##
###############################

# Condenses old log entries into single-line summaries by month.
# Must be run as root or invoked with sudo.
# WARNING: This script is destructive, the original log entries will be parsed and deleted. 
# Optimized for crontab, try pasting the following line at the bottom of the file after running 'sudo crontab -e' to make it run at 2AM on the first of every month.

# 0 2 1 * * /home/pi/summarize_uplink_log.py

# WARNING AGAIN: Optimized as it is to run automatically in the background, this script will NOT prompt for confirmation before modifying the log file. 
# Dry runs may be performed by running without sudo.


import re
from datetime import datetime, timedelta
import os

def main():
    log_file = os.path.expanduser('~/uplink.log')

    # Regular expressions to match log messages
    timestamp_pattern = re.compile(r'^\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}')
    packet_loss_pattern = re.compile(r'Received response from ping\. Packet loss: (\d+)%')
    connection_failed_pattern = re.compile(r'Connection check failed\. Packet loss: (\d+|Unknown)%')

    months_data = {}
    overall_connected = None
    overall_state_start_time = None

    with open(log_file, 'r') as f:
        lines = f.readlines()

    for i, line in enumerate(lines):
        line = line.strip()
        if not line:
            continue
        # Check if the line starts with a timestamp
        if not timestamp_pattern.match(line):
            # Skip lines without timestamps
            continue
        # Parse the timestamp and message
        try:
            timestamp_str, message = line[:19], line[20:]
            timestamp = datetime.strptime(timestamp_str, '%Y-%m-%d %H:%M:%S')
        except Exception:
            # Skip lines that don't match the expected format
            continue
        # Determine the month key
        month_key = timestamp.strftime('%Y-%m')
        if month_key not in months_data:
            # Initialize data for the month
            months_data[month_key] = {
                'total_packet_loss': 0,
                'packet_loss_count': 0,
                'total_connected_time': 0.0,
                'total_disconnected_time': 0.0,
                'times_gone_offline': 0,
                'state_start_time': None,
                'connected': None  # None means unknown, True or False
            }
        data = months_data[month_key]
        # Determine new connected status based on the message
        new_connected_status = None
        packet_loss_match = packet_loss_pattern.match(message)
        if packet_loss_match:
            packet_loss = int(packet_loss_match.group(1))
            data['total_packet_loss'] += packet_loss
            data['packet_loss_count'] += 1
            new_connected_status = True
        else:
            connection_failed_match = connection_failed_pattern.match(message)
            if connection_failed_match:
                packet_loss_str = connection_failed_match.group(1)
                packet_loss = int(packet_loss_str) if packet_loss_str.isdigit() else 100
                data['total_packet_loss'] += packet_loss
                data['packet_loss_count'] += 1
                new_connected_status = False
            else:
                # Other messages, ignore or handle if needed
                continue
        # If this is the first status update, set state_start_time and connected status
        if data['connected'] is None:
            data['state_start_time'] = timestamp
            data['connected'] = new_connected_status
            if new_connected_status == False:
                data['times_gone_offline'] += 1
        else:
            # If the connected status has changed, compute the duration
            if new_connected_status != data['connected']:
                duration = (timestamp - data['state_start_time']).total_seconds()
                if data['connected']:
                    # Previous state was connected
                    data['total_connected_time'] += duration
                else:
                    # Previous state was disconnected
                    data['total_disconnected_time'] += duration
                # Update state
                data['state_start_time'] = timestamp
                data['connected'] = new_connected_status
                if new_connected_status == False:
                    # Connection just went offline
                    data['times_gone_offline'] +=1
            else:
                # Status remains the same, do not compute duration yet
                pass

    # After processing all lines, for each month, we need to account for the final duration up to the end of the month or last timestamp
    for month_key in sorted(months_data.keys()):
        data = months_data[month_key]
        # If there is an ongoing state, compute duration up to the end of the month
        if data['state_start_time'] is not None:
            # Determine the end time
            year, month = map(int, month_key.split('-'))
            if month == 12:
                next_month = datetime(year+1, 1, 1)
            else:
                next_month = datetime(year, month+1, 1)
            end_time = next_month
            # Calculate the duration from the last state_start_time to the end of the month
            duration = (end_time - data['state_start_time']).total_seconds()
            if data['connected']:
                data['total_connected_time'] += duration
            else:
                data['total_disconnected_time'] += duration

    # Prepare summary lines
    summary_lines = []
    for month_key in sorted(months_data.keys()):
        data = months_data[month_key]
        if data['packet_loss_count'] > 0:
            avg_packet_loss = data['total_packet_loss'] / data['packet_loss_count']
        else:
            avg_packet_loss = None
        total_connected_time = data['total_connected_time']
        total_disconnected_time = data['total_disconnected_time']
        times_gone_offline = data['times_gone_offline']
        # Calculate uptime and downtime percentages
        total_time = total_connected_time + total_disconnected_time
        if total_time > 0:
            uptime_percentage = (total_connected_time / total_time) * 100
            downtime_percentage = (total_disconnected_time / total_time) * 100
        else:
            uptime_percentage = downtime_percentage = 0.0
        if avg_packet_loss is not None:
            summary_line = (
                f"{month_key} - Average Packet Loss: {avg_packet_loss:.2f}% | "
                f"Total Connected Time: {int(total_connected_time)}s ({uptime_percentage:.2f}%) | "
                f"Total Disconnected Time: {int(total_disconnected_time)}s ({downtime_percentage:.2f}%) | "
                f"Times Gone Offline: {times_gone_offline}"
            )
        else:
            summary_line = (
                f"{month_key} - Average Packet Loss: N/A | "
                f"Total Connected Time: {int(total_connected_time)}s ({uptime_percentage:.2f}%) | "
                f"Total Disconnected Time: {int(total_disconnected_time)}s ({downtime_percentage:.2f}%) | "
                f"Times Gone Offline: {times_gone_offline}"
            )
        summary_lines.append(summary_line)

    # Overwrite the log file with the summary
    with open(log_file, 'w') as f:
        for line in summary_lines:
            f.write(line + '\n')

    print("Log file has been summarized and overwritten with the summarized data.")

if __name__ == '__main__':
    main()
