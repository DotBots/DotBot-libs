# DotBot Libraries: reusable peripheral drivers and libraries

[![CI][ci-badge]][ci-link]
[![Documentation][doc-badge]][doc-link]
[![License][license-badge]][license-link]

## Overview

This repository contains the source code of low-level drivers and libraries that
can be used to build firmwares for the DotBot robotic platform.

## Related projects

The DotBots ecosystem provides Python
[PyDotBot](https://github.com/DotBots/PyDotBot) to interacts with a nRF DK board
used a gateway.

The source code for ready to use applications is provide by the
[DotBot-firmware](https://github.com/DotBots/DotBot-firmware) repository.

The DotBots hardware design are published on GitHub at
[https://github.com/DotBots/DotBot-hardware](https://github.com/DotBots/DotBot-hardware).

## Building firmwares

The source code of the different applications available in this repository can be built using
[SEGGER Embedded Studio for ARM](https://www.segger.com/downloads/embedded-studio).
In SEGGER embedded studio, use the package manager
(available in menu Tools > Package manager) to install the CMSIS 5 CMSIS-CORE,
CMSIS-DSP and nRF packages.

For details on SEGGER Embedded Studio, read the
[online documentation](https://studio.segger.com/index.htm?https://studio.segger.com/home.htm).

[ci-badge]: https://github.com/DotBots/DotBot-libs/workflows/CI/badge.svg
[ci-link]: https://github.com/DotBots/DotBot-libs/actions?query=workflow%3ACI+branch%3Amain
[doc-badge]: https://readthedocs.org/projects/dotbot-libs/badge/?version=latest
[doc-link]: https://dotbot-libs.readthedocs.io/en/latest
[license-badge]: https://img.shields.io/github/license/DotBots/DotBot-libs
[license-link]: https://github.com/DotBots/dotbot-libs/blob/main/LICENSE.txt
