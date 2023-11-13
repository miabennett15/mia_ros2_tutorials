# Raspberry Pi Interface Usage

## Boot Overlays

We have currently:

* enabled UART ports 0, 2 and 4,
* enabled Hardware PWM GPIO Pin 19,
* switched off Bluetooth (as it is currently unused).

The following was added to the end of `/boot/config.txt`. The Raspberry Pi 5 symlinks `/boot/config.txt` to `/boot/firmware/config.txt`, so you can still edit the file using:

```bash
sudo nano /boot/config.txt
```

The end of the file should contain:

```text
[all]
dtoverlay=disable-bt
dtoverlay=uart0
dtoverlay=uart2
dtoverlay=uart4
dtoverlay=pwm,pin=19
```

## UART Serial Ports

We have allocated UART ports as follows:

| Device | UART |
| --- | --- |
| IMU | 0 |
| LIDAR | 2 |
| GPS |	4 |


## GPIO Pin Usage

We are currently using the GPIO header as follows:

| Use | Configuration | GPIO Pin | Pin # | Pin # | GPIO Pin | Configuration | Use |
| --- | --- | --- | --- | --- | --- | --- | --- |
|  | - | 3v3 | 1 | 2 | 5v | - |  |
|  |  |  | 3 | 4 | 5v | - |  |
|  |  |  | 5 | 6 | Ground |  |  |
| LiDAR _unused_&dagger; | TXD2 | GPIO 4 | 7 | 8 | GPIO 14 | TXD0 | IMU RX |
|  | - | Ground | 9 | 10 | GPIO 15 | RXD0 | IMU TX |
|  |  |  | 11 | 12 | GPIO 18 | PWM0 |  |
|  |  |  | 13 | 14 | Ground |  | LiDAR Ground |
|  |  |  | 15 | 16 |  |  |  |
|  |  |  | 17 | 18 |  |  |  |
|  |  |  | 19 | 20 | Ground | - |  |
| _Unused_* | RXD3 | GPIO 9 | 21 | 22 |  |  |  |
|  |  |  | 23 | 24 | GPIO 8 | TXD3 | _Unused*_ |
|  | - | Ground | 25 | 26 |  |  |  |
|  |  |  | 27 | 28 |  |  |  |
| LiDAR TX | RXD2 | GPIO 5 | 29 | 30 | Ground | - |  |
|  |  |  | 31 | 32 | GPIO 12 | TXD4 |  |
| GPS TX | RXD4 | GPIO 13 | 33 | 34 | Ground | - |  |
| LiDAR PWM | PWM1 | GPIO 19 | 35 | 36 |  |  |  |
|  |  |  | 37 | 38 |  |  |  |
|  | - | Ground | 39 | 40 |  |  |  |

_Unused_* UART3 has been listed as we may need to use a 4th UART at some point.

LiDAR _unused_&dagger; The LiDAR unit we use cannot receive serial data.
