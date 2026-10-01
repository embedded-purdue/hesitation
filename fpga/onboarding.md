# FPGA Team Onboarding

## Download Vivado
[Download link](https://www.amd.com/en/support/downloads/adaptive-socs-and-fpgas/development-tools/2025-2.html)

[Licensing link](https://account.amd.com/en/forms/license/license-form.html)

- Scroll down to "Unified Installer for FPGA & Adaptive SoC Tools Update 1 - 2025.2.1 - Mar 26, 2026" and download the EXE installer.
- Launch the installer. When prompted to download 2026.1.1, just select "Continue" (don't select "Get Latest").
- If you do not have an AMD account, create one and authenticate in the installer. You may just list "Student" as your job function and your name or Purdue for company. I would suggest using a personal email.
- Select "Vitis".
- Under Devices, select "Install Devices for Kria SOMs and Starter Kits", "7 Series", and "SoCs".
- Agree to all terms.
- I suggest leaving the recommended install location.
- Once you have installed, launch Vivado from the Windows Start Menu. It will ask for a license.
- Visit the licensing link above and sign in.
- Check the box next to "Vivado Basic Tier License, Node Locked License", and then click "Generate Node-Locked License".
- Select "Select a host..." -> "Add a host..."
- Add a name (I suggest your PC name), select Windows 64-bit, select Disk Serial Number. For the serial number, open a command prompt (Windows Key -> cmd), and type: `vol C:`. Paste the S/N.
- Download the license file from the email it sends.
- Launch Vivado License Manager (VLM), and link the file by following the prompts.

## Tutorials
Please watch/read these tutorials for the next meeting.

- [Tips for Verilog beginners from a Professional FPGA Engineer](https://www.youtube.com/watch?v=4ynX6o0Zznw)
- [Verilog FSM](https://chipverify.com/verilog/verilog-fsm)
- [SystemVerilog Mini Course - Part 5 - Finite State Machines](https://www.youtube.com/watch?v=hC8jgq2TzpM)
- [SystemVerilog Tutorial](https://chipverify.com/tutorials/systemverilog)
- [SystemVerilog TestBench](https://chipverify.com/systemverilog/systemverilog-simple-testbench)
- [Serial Peripheral Interface || SPI PROTOCOL || explanation with Verilog code and Testbench](https://www.youtube.com/watch?v=hiCm95SieSE)

Keep in mind that SystemVerilog is very similar to Verilog. Most of the time, the only observable difference for us right now is we use the `logic` datatype instead of `wire` and `reg`.

I know it is a lot of content, but keep in mind that this stuff is hard! I essentially need to bring you guys up to speed with 1-2 semesters of digital design courses. Please get through as much of it as you can!

If you have any questions, please message me in the discord thread.
