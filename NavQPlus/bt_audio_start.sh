#!/bin/bash
# Bluetooth A2DP Audio Startup Script
# EWA Audio A106Pro - 70:D5:EA:A4:84:1A

MAC_ADDR="70:D5:EA:A4:84:1A"
LOG="/home/marina/bt_audio.log"

echo "[$(date)] Starting BT audio setup..." | tee -a $LOG

# 1. Wait for system services to initialize
sleep 10

# 2. Start bluealsa if not already running
if ! pgrep -x "bluealsa" > /dev/null; then
    echo "[$(date)] Starting bluealsa..." | tee -a $LOG
    sudo bluealsa -p a2dp-sink -p a2dp-source &
    sleep 3
else
    echo "[$(date)] bluealsa already running" | tee -a $LOG
fi

# 3. Connect to BT speaker
echo "[$(date)] Connecting to $MAC_ADDR..." | tee -a $LOG
bluetoothctl connect $MAC_ADDR
sleep 5

# 4. Verify A2DP sink is available
if bluealsa-aplay -l 2>/dev/null | grep -q "A2DP"; then
    echo "[$(date)] A2DP sink confirmed" | tee -a $LOG
else
    echo "[$(date)] WARNING: A2DP sink not found, retrying..." | tee -a $LOG
    bluetoothctl disconnect $MAC_ADDR
    sleep 2
    bluetoothctl connect $MAC_ADDR
    sleep 5
fi

# 5. Test audio
echo "[$(date)] Testing audio output..." | tee -a $LOG
espeak -s 130 -p 45 -a 150 --stdout "Ready" | \
    sox -t raw -r 22050 -e signed -b 16 -c 1 - -t wav -r 48000 -c 2 /tmp/speech.wav
aplay -D bluealsa:DEV=$MAC_ADDR,PROFILE=a2dp /tmp/speech.wav

echo "[$(date)] BT audio setup complete" | tee -a $LOG