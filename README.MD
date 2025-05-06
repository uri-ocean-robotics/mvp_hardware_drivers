## MVP related utlitity files and ros node
This repository has all the hardware drivers for the AUVs using mvp framework

- install `i2cbus` library
```
 sudo apt-get install libi2c-dev
 ```

- GPIO setup using wiringPi
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