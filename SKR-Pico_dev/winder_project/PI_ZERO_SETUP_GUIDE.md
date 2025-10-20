# 🚀 Pi Zero Setup Guide for Winder Project

## WHAT YOU NEED:
- Pi Zero (or Zero W for WiFi!)
- **MicroSD card: 8-32GB** ← Get this!
- USB cable for power
- Your Mac for setup

## STEP 1: Flash SD Card (30 min)

1. Download Raspberry Pi Imager: https://www.raspberrypi.com/software/
2. Insert SD card into Mac
3. Open Imager:
   - Device: "Raspberry Pi Zero"
   - OS: "Raspberry Pi OS Lite (64-bit)"
   - Storage: Your SD card
   - Settings (⚙️):
     * Hostname: "winder"
     * Enable SSH: YES
     * Username: "pi" 
     * Password: (your choice)
     * WiFi: (if Zero W)
     * Timezone: Your timezone
4. Click WRITE (takes ~5 min)

## STEP 2: Enable UART
After flashing, edit config on SD card:
- File: /Volumes/bootfs/config.txt
- Add: enable_uart=1
- Add: dtoverlay=disable-bt

## STEP 3: Boot & SSH
1. Insert SD into Pi Zero
2. Power up (wait 60 sec)
3. On Mac: ssh pi@winder.local
4. You're in! 🎉

## STEP 4: Install Tools
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y python3-pip python3-serial
sudo apt install -y build-essential cmake git
```

## NEXT: I'll create UART protocol + code examples!
