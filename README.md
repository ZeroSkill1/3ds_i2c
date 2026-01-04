# About

An open-source reimplementation of the I2C system module.

# Compiling

Requirements:
- Latest version of devkitPro/devkitARM
- makerom on PATH

The following environment variables are available to configure the build process:

| Variable             | Description                                                                                                                                                                       |
|----------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `DEBUG`              | When set, all optimization is disabled and debug symbols are included in the output ELF. When not set, the ELF will be optimized for size and will not include any debug symbols. |
| `N3DS`               | Build the New3DS-specific variation of the I2C module (with the New3DS bit set in the title ID).                                                                                  |

# Licensing

The project itself is using the Unlicense.

Parts of [libctru](https://github.com/devkitPro/libctru) were taken and modified (function argument order, names, etc.). The license for libctru can be seen here: [LICENSE.libctru.md](/LICENSE.libctru.md)
