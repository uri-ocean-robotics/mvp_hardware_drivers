## MVP related utlitity files and ros node
This repository has all the hardware drivers for the AUVs using mvp framework

- install `i2cbus` library
```
 sudo apt-get install libi2c-dev
 ```

- GPIO setup using wiringPi [PI4]
    - Go to wiringPi github [link](https://github.com/WiringPi/WiringPi?tab=readme-ov-file)
    - Following their installation steps to get the newest wiring Pi.

    - Check user access (if access denied when running the code)
    ```
    ls -l /dev/mem /dev/gpiomem
    ```
    - You should see
    ```
    crw-rw---- 1 root gpio 238, 0 Jan  3 19:24 /dev/gpiomem
    crw-r----- 1 root kmem   1, 1 Jan  3 19:24 /dev/mem
    ```
    - if the first one is `root root`, you do 
    ```
    sudo chown root.gpio /dev/gpiomem
    sudo chmod g+rw /dev/gpiomem
    ```
    - Add your user to the `gpio`, `kmem`, and `dialout` group.
    ```
    sudo usermod -G gpio -a $USER
    sudo usermod -G kmem -a $USER
    sudo usermod -G dialout -a $USER
    ```

    - Add udev rules to save the change
    ```
    sudo nano /etc/udev/rules.d/99-gpiomem.rules

    KERNEL=="gpiomem", GROUP="gpio", MODE="0660"

    sudo usermod -aG gpio $USER

    sudo udevadm control --reload-rules
    sudo udevadm trigger
    ```

- GPIO Permission setup with PI5
    - replace `gpiomem` with `gpiomem0` in the above steps

- New GPIO libary for PI5
 ```sudo apt install python3-rpi-lgpio```
  - Dropweight script GPIO chip in PI5 is `gpiochip4` instead of `gpiochip0`
  
- MUC ATTINY85-20P Setup
    - Add AttinyCore for wire.h
    - Additional manager in preference http://drazzy.com/package_drazzy.com_index.json

### systemd service for Dropweight & Strobe 
The dropweight & strobe functionality is a [script](./mvp_gpio_manager/src/drop_weight.py) that runs as a systemd service.<br>
1. Create a new service :
```sudo nano /etc/systemd/system/dropweight.service```
2. Paste the following into the ```dropweight.service``` file:
```
[Unit]
Description=Dropweight GPIO Controller
DefaultDependencies=no
After=local-fs.target
Before=shutdown.target
Conflicts=shutdown.target
Requires=dev-gpiochip4.device

[Service]
Type=simple
ExecStart=/usr/bin/python3 <path_to_script>
Restart=no
RestartPreventExitStatus=0
RestartSec=2
KillSignal=SIGTERM
TimeoutStopSec=10

SendSIGKILL=no
# Security / sandboxing (optional)
NoNewPrivileges=true
PrivateTmp=true

# Logging
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

3. ```sudo systemctl daemon-reload```
4. ```sudo systemctl enable dropweight.service```
5. ```sudo systemctl start dropweight.service```
6. Verify by ```sudo systemctl status dropweight.service``` 