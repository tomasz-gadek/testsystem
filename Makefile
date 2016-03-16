
default:
	@echo ""
	@echo "Please run commands with sudo"
	@echo "-----------------------------"
	@echo " 'compile'  to compile testsystem, requires iowkit.o in the directory"
	@echo " 'run'      to run compiled file"
	@echo " 'clean'    to delete compiled data"
	@echo " 'info'     to get information about installed iowarrior module"
	@echo " 'usb'      list of all connected USB device"
	@echo ""

compile:
	@echo ""
	@echo "Compiling..."
	@gcc testsystem.c -o testsystem -l iowkit -lm -lrt
	@echo ""

run:
	@echo "Trying to run i2c_shtc1_shtw1, make sure you did 'make compile' first"
	@./testsystem
	@echo ""
	@echo ""
	
clean:
	@echo ""
	@echo "Deleting compiled data"
	@rm -f i2c_shtc1_shtw1
	@echo ""

info:
	@echo ""
	@modinfo iowarrior
	@echo ""
usb:
	@echo ""
	@lsusb
	@echo ""
