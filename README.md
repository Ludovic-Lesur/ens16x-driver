# Description

This repository contains the **ENS16x** air quality sensor driver.

# Dependencies

The driver relies on:

* An external `types.h` header file defining the **standard C types** of the targeted MCU.
* The **embedded utility functions** defined in the [embedded-utils](https://github.com/Ludovic-Lesur/embedded-utils) repository.

Here is the versions compatibility table:

| **ens16x-driver** | **embedded-utils** |
|:---:|:---:|
| [sw5.1](https://github.com/Ludovic-Lesur/ens16x-driver/releases/tag/sw5.1) | >= [sw5.0](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw5.0) |
| [sw5.0](https://github.com/Ludovic-Lesur/ens16x-driver/releases/tag/sw5.0) | >= [sw5.0](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw5.0) |
| [sw4.2](https://github.com/Ludovic-Lesur/ens16x-driver/releases/tag/sw4.2) | >= [sw5.0](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw5.0) |
| [sw4.1](https://github.com/Ludovic-Lesur/ens16x-driver/releases/tag/sw4.1) | >= [sw5.0](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw5.0) |
| [sw4.0](https://github.com/Ludovic-Lesur/ens16x-driver/releases/tag/sw4.0) | >= [sw5.0](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw5.0) |
| [sw3.2](https://github.com/Ludovic-Lesur/ens16x-driver/releases/tag/sw3.2) | >= [sw5.0](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw5.0) |
| [sw3.1](https://github.com/Ludovic-Lesur/ens16x-driver/releases/tag/sw3.1) | >= [sw5.0](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw5.0) |
| [sw3.0](https://github.com/Ludovic-Lesur/ens16x-driver/releases/tag/sw3.0) | >= [sw5.0](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw5.0) |
| [sw2.0](https://github.com/Ludovic-Lesur/ens16x-driver/releases/tag/sw2.0) | >= [sw5.0](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw5.0) |
| [sw1.0](https://github.com/Ludovic-Lesur/ens16x-driver/releases/tag/sw1.0) | >= [sw5.0](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw5.0) |

# Compilation flags

| **Flag name** | **Value** | **Description** |
|:---:|:---:|:---:|
| `ENS16X_DRIVER_DISABLE_FLAGS_FILE` | `defined` / `undefined` | Disable the `ens16x_driver_flags.h` header file inclusion when compilation flags are given in the project settings or by command line. |
| `ENS16X_DRIVER_DISABLE` | `defined` / `undefined` | Disable the ENS16x driver. |
| `ENS16X_DRIVER_I2C_ERROR_BASE_LAST` | `<value>` | Last error base of the low level I2C driver. |
| `ENS16X_DRIVER_DELAY_ERROR_BASE_LAST` | `<value>` | Last error base of the low level delay driver. |
| `ENS16X_DRIVER_DEVICE_ENS161` | `defined` / `undefined` | Select ENS161 chip if defined, ENS160 otherwise. |
