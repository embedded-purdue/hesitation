ifeq ($(OS),Windows_NT)
    SHELL := C:/Program Files/Git/bin/bash.exe
	VIVADO := C:/AMDDesignTools/2025.2/Vivado/bin/vivado.bat
else
    SHELL := /bin/bash
	VIVADO := vivado
endif

TB ?= counter_4bit_tb

.PHONY: clean project sim wav

CLEAN_OBJS = *.log *.jou dfx_runtime.txt .Xil/ project/

project:
	$(VIVADO) -mode batch -source ./scripts/regen_project.tcl -notrace -log project.log -jou project.jou

sim:
	$(VIVADO) -mode batch -source ./scripts/sim.tcl -tclargs $(TB) -log sim.log -jou sim.jou

wav:
	$(VIVADO) -mode gui -source ./scripts/wav.tcl -tclargs $(TB) -log wav.log -jou wav.jou

clean:
	rm -rf $(CLEAN_OBJS)
