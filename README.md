# Raspberry PI

## Building

Run `make` inside the raspberry pi where this source code is found (inside the workspace).

## Running

### Client-side management of movement through a map

Execute the client side with a python3.9 client with `Djikstra` and `numpy` installed.

By default it uses a map called `map.txt` in the execution folder, and an enlargement safety of 2 blocks in the obstacles found in the map

#### Main manager


Run  `python3 planning/main.py --help` for extra information.

```bash
# It assumes username is pi
export ROBOT_IP="<the robot ip>"
export ROBOT_FOLDER="<the folder of the main executable>"
export ROBOT_EXE="<the main c executable>"
# Add the --dry-run line if you just want to get a description of the attempt of reaching the poitn
python3 planning/main.py --init 700 500 90 --end 700 1800 --speed 80 # --dry-run
```



#### Detection

Graph detection, test with

```bash
python3 planning/detection.py
```

#### Management

Management of the robot externally, test that movement works by running this, which will execute a simple movement procedure.

```bash
python3 planning/manage.py
```


### main

Example

```txt
SPEED=80 sudo -E ./main move 200 100 -100 move 0 0 0 move 20 30 20 2>&1 >/dev/null   | grep -E "RUNNING|Point"
```

Environment meta-parameters

```txt
X_INIT
Y_INIT
THETA_INIT
SPEED
N_TRIES
```

Command line (See main)


### speeds

Keep wheels from touching the floor, this will create a table of different speeds of wheels, in relaiton to the thing.

### tests

Program to run and test if turning left/right and going forward/backward works or not

### calibrate

Program to calibrate the wheels so their response to pulses is to spec




## Connecting
### SSH Connecting to raspberry pi

Assuming raspberry at 192.168.1.59

- We use `ssh-copy-id` to have a permanent link without password (Note, generate an ssh key first with `ssh-keygen -t ed25519`)
    ssh-copy-id -i key.pub   pi@192.168.1.59

- We then use the `remote` extension package in vscode, and connect to the raspberry
- We can now code there
- To back up what we are doing in the raspberry to our local environment,
    rsync -chavzP --stats pi@192.168.1.34:/home/pi/workspace ./pi


### Connect with ethernet cable

https://raspberrypi-guide.github.io/networking/create-direct-ethernet-connection

Basically, loook for the `bridge100` interface. Their IP assigned will be yours+1, based on how it assigns IPs to itself.

When you connect over ethernet cable

1. Enable internet sharing
2. Run `ifconfig` and find `bridge100` network. Check the ip, and network mask; let's say it is `192.168.2.1/24`
3. Scan the network with `nmap -nP -s 192.168.2.1/24`
4. Connect with `pi@192.168.2.3` or something like that

In my case, ping is more reliable than nmap, since it is usually just *.x with x being small

### Configuring WiFi

#### Raspberry system option

https://pimylifeup.com/setting-up-raspberry-pi-wifi/

```bash
sudo nano /etc/wpa_supplicant/wpa_supplicant.conf

```

Then make changes like

```txt
network={
ssid="The SSID of your network (eg. Network name)"
psk="Your Wifi Password"
}
```
#### Linux/Debian option

1. Install NetworkManager (https://computingforgeeks.com/install-and-use-networkmanager-nmcli-on-ubuntu-debian/)

```
sudo apt-get install network-manager

sudo systemctl start NetworkManager.service
sudo systemctl enable NetworkManager.service
```



#### Show connections

nmcli connection show


#### Add connections

According to chatgpt

To connect to Wi-Fi through the terminal in Linux, you can use the `nmcli` command-line tool, which is a command-line interface for the NetworkManager daemon. Here are the steps:

1. Open a terminal window.

2. Type the command `nmcli device wifi list` to see a list of available Wi-Fi networks.

3. Choose the network you want to connect to and note down the SSID and security type.

4. Type the command `nmcli device wifi connect <SSID> password <password> ifname <interface>` to connect to the Wi-Fi network. Replace `<SSID>` with the name of the network you want to connect to, `<password>` with the Wi-Fi password, and `<interface>` with the name of your Wi-Fi interface (e.g. `wlan0` or `wlp2s0`).

5. Once the command is executed successfully, you should be connected to the Wi-Fi network.

Note: If the Wi-Fi network uses a hidden SSID, you will need to add the `hidden yes` option to the `nmcli` command.






## Configuring raspberry,

```
sudo apt-get update && sudo apt-get upgrade
# This is a graphical UI
sudo raspi-config
```




## Transfering a bunch of include statements

Make your own sysroot, then trasfer them.

```bash
RASP=pi@192.168.2.2
rsync -rzLR --safe-links \
      "$RASP:/usr/lib/arm-linux-gnueabihf" \
      "$RASP:/usr/lib/gcc/arm-linux-gnueabihf" \
      "$RASP:/usr/include" \
      "$RASP:/usr/local/include" \
      "$RASP:/usr/local/lib" \
      "$RASP:/lib/arm-linux-gnueabihf" \
      ./sysroot/
```



# Useful information
## Sources

- `data.json` https://github.com/pinout-xyz/Pinout.xyz

## How to do threading

- https://github.com/BPI-SINOVOIP/RPi.GPIO/blob/master/source/event_gpio.c


## Why WiringPi is a bad choice

- WIRING PI DEPRECATION https://web.archive.org/web/20191001170509/http://wiringpi.com/wiringpi-deprecated/
    - "wiringPi was designed to be used by experienced C and RTB BASIC programmers. It is not a newbie learning tool."
- PIN NUMBERING https://web.archive.org/web/20191021032427/https://projects.drogon.net/wiringpi-pin-numbering/


## MCP3008

https://github.com/nagimov/mcp3008hwspi

## DNS magic, call by hostname

- https://unix.stackexchange.com/a/16901

## DRAFT Target from a different machine??

Trying to learn this.

https://freckled.dev/how-to/2019/07/29/create-gcc-crosscompiler/

gcc https://gcc.gnu.org/install/specific.html

Some medium post that I am suspicious of https://medium.com/@haraldfernengel/cross-compiling-c-c-from-macos-to-raspberry-pi-in-2-easy-steps-23f391a8c63


The good tool? https://crosstool-ng.github.io/docs/introduction/ install with `brew install crosstool-ng`?


## Debugging code

Firstly, you have to  use the debug flag -g, you MUST use it both to create the .o objects, as well as the main object.

After that, when your program runs and creates some segfault or something it will create a file called `core`

NOTE: If you run `ulimit -c` and you get `0`, it won't generate a core dump, set it to something. `ulimit -c unlimited` to ALWAYS generate a core dump (unsafe)



Then run your core dump with dbp
```bash
sudo gdb ./main ./core # main is the program, core is the debugging dump
```

To run again the code and see where it fails you can use `run` as an instruction inside gdb when the command line interface opens when running gdb. There are quite a few other options there


### Core pattern

https://sigquit.wordpress.com/2009/03/13/the-core-pattern/

You can set the core pattern, for example

```
echo "/tmp/coredump-%e-%p" > /proc/sys/kernel/core_pattern
```

But to make it permanent, not until the next reboot, you would need to change it in the system reboot config.

In `/etc/sysctl.conf`, you would add:

```txt
# Own core file pattern...
kernel.core_pattern=/tmp/cores/core.%e.%p.%h.%t
```
