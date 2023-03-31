# ssl_mbed-brushless

A port of STM32CubeMx brushless project to mbed, using 6TRON ecosystem and MbedOs bare-metal profil.

## Requirements

### Hardware requirements

One of the following boards is required:

- SSL Brushless MK.II V1.0.0 with a STM32F103RBT6
- SSL Brushless MK.II V1.0.1 with a STM32L4A6RGT6

This repository **is not compatible** with the following board:

* SSL Brushless Powerboard rev 3.0.0 with a Trinamic TMC4671-LA

### Software requirements

mbed_ssl-brushless makes use of the following libraries (automatically
imported by `mbed deploy` or `mbed import`):

- [MbedOs 6.17.0](https://github.com/ARMmbed/mbed-os/releases/tag/mbed-os-6.17.0)
- [6TRON Motor lib](https://github.com/catie-aq/6tron_motor)
- [6TRON Motor Sensor lib](https://github.com/catie-aq/6tron_motor_sensor/)
- [6TRON nanopb Mbed port lib](https://github.com/catie-aq/mbed_nanopb/)
- [6TRON PID lib](https://github.com/catie-aq/6tron_pid/)
- [SSL nanopb data protocol definition lib](https://github.com/catie-aq/ssl_data-protocol)



This project required Mbed CLI 1 to build the program.

You will find the official [6TRON setup and installation guide here](https://member.6tron.io/fr/m/ressources/plateforme-logicielle/guide-demarrage-mbed-os/latest/en/).

> For Windows newbies, you can find an alternate guide in french [here](https://github.com/eirbot/mbed-os-template-eirbot).

## Usage

To clone **and** deploy the project in one command, use `mbed import` and skip to the
target enabling instructions:

```shell
mbed import https://github.com/catie-aq/ssl_mbed-brushless.git ssl_mbed-brushless
```

Alternatively:

- Clone to "mbed_ssl-brushless" and enter it:

  ```shell
  git clone https://github.com/catie-aq/ssl_mbed-brushless.git ssl_mbed-brushless
  cd ssl_mbed-brushless
  ```

- Deploy software requirements with:

  ```shell
  mbed deploy
  ```

Choose which board you want to use:

* For SSL Brushless MK.II V1.0.0 board with a STM32F103RBT6:

  ```shell
  mbed target SSL_BRUSHLESS_F103
  ```

* For SSL Brushless MK.II V1.0.1 board with a STM32L4A6RGT6:

  ```shell
  mbed target SSL_BRUSHLESS_L4A6
  ```

Compile the project:

```shell
mbed compile
```

Program the target device with a Segger J-Link debug probe and
[6tron_flash](https://github.com/catie-aq/6tron_flash) tool:

```shell
sixtron_flash
```

Debug on the target device with the probe and Segger [Ozone](https://www.segger.com/products/development-tools/ozone-j-link-debugger) software.
