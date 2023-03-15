# mbed_ssl-brushless
A port of STM32CubeMx brushless project to mbed, using 6TRON ecosystem

## Requirements
### Hardware requirements
The following boards are required:
- *List mbed_ssl-brushless hardware requirements here*

### Software requirements
mbed_ssl-brushless makes use of the following libraries (automatically
imported by `mbed deploy` or `mbed import`):
- *List mbed_ssl-brushless software requirements here*

## Usage
To clone **and** deploy the project in one command, use `mbed import` and skip to the
target enabling instructions:
```shell
mbed import https://gitlab.com/catie_6tron/mbed_ssl-brushless.git mbed_ssl-brushless
```

Alternatively:

- Clone to "mbed_ssl-brushless" and enter it:
  ```shell
  git clone https://gitlab.com/catie_6tron/mbed_ssl-brushless.git mbed_ssl-brushless
  cd mbed_ssl-brushless
  ```

- Deploy software requirements with:
  ```shell
  mbed deploy
  ```

Enable the custom target:
```shell
cp zest-core-stm32l4a6rg/custom_targets.json .
```

Compile the project:
```shell
mbed compile
```

Program the target device with a Segger J-Link debug probe and
[`sixtron_flash`](https://gitlab.com/catie_6tron/6tron-flash) tool:
```shell
sixtron_flash stm32l4a6rg BUILD/ZEST_CORE_STM32L4A6RG/GCC_ARM/mbed_ssl-brushless.elf
```

Debug on the target device with the probe and Segger
[Ozone](https://www.segger.com/products/development-tools/ozone-j-link-debugger)
software.
