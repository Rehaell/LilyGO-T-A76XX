1.  Make sure usbipd is installed via powershell.

2. To attach the USB Device via WSL perform the following commands
    - usbipd list
    - usbipd bind --busid <busid>
    - usbipd attach --wsl --busid <busid>

3. To detach the USB device perform the following command
    - usbipd attach --wsl --busid <busid>

4. If the USB device is still attached in persistent mode, perform the following command
    -  usbipd unbind -a