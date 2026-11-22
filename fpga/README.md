# Basic Vivado Project and Simulation

## Prerequisites
AMD Vivado, Git Bash, GNU Make

### AMD Vivado
[Vivado 2025.2.1](https://www.amd.com/en/support/downloads/adaptive-socs-and-fpgas/development-tools/2025-2.html)

1. Scroll down to 2025.2.1, and download the "Windows Self Extracting Web Installer"
2. Run the installer. When prompted for what boards you want to install, check 7-series. Also, take note of where you installed it to on you computer.

### Git Bash
[Git](https://git-scm.com/install/windows)

1. Download the installer.
2. When prompted, make sure to check the option to install Git Bash. Also, make sure to check the option to add Git to your PATH.

### GNU Make
1. Open a PowerShell terminal (Windows Key -> "Terminal")
2. Install make using winget:

```PowerShell
winget install ezwinports.make
```

## Makefile
In the Makefile, there are currently a few commands: project, sim, and clean. To run them, open this repository in VSCode, and then open a terminal in VSCode (Ctrl + ~).

`make project`: Sets up the Vivado project. This is needed so that Vivado can understand where all the source files live and for it to do its necessary operations.

`make sim`: Runs the testbench (specified in line 3 of ./scripts/sim.tcl) in the terminal.

`make clean`: deletes all log files and the project directory.
