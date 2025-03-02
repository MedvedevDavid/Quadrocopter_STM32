# Quadrocopter_STM32
The STM32 version of my quadrocopter project.

## Clone the project

After cloning the repositori all the sumodules examle google test need to be cloned with the following command:

```
git submodule init
git submodule update
```

## Enter to container
The code can be compiled from a docker container. To compile the container use:
`docker compose up`

With VS Code docker plugin the container need to be attched.

## Compilation
From the container the SW can be built withe the following command:

`make build`

The CMake command:
`cmake -Bbuild -DCMAKE_TOOLCHAIN_FILE=tools/board_specific/gcc-arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=true`


## Connecting USB devices

### Connect USB to WSL:
https://devblogs.microsoft.com/commandline/connecting-usb-devices-to-wsl/

### Adding the programmer to wsl 
Checking if the programmer is added:

`st-info --probe`

To autodetect the deugger properly 
modification in the cd /etc/udev/rules.d/  --> stm_link.sh

`sudo service udev start`


### Serial port to be working on Linux:
Add user to the dialout group
sudo chmod 777 /dev/ttyACM0


## Usefull functions

Suspend all tasks and ISRs:
```
    __disable_irq();
    vTaskSuspendAll();

    __enable_irq(); 
    xTaskResumeAll();
```

"board/st_nucleo_f4.cfg"