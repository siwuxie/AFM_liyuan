CC = gcc-linux
C_FLAG=-D_DEBUG_ -Wall
FLAG_7489 = -D_CONST_DELAY_ -D_HT7489_
FLAG_511P = -D_CONST_DELAY_ -D_PM511P_
LINK_FLAG=-lm -lpthread

GENERAL_OBG = main.o dispatch_cmd.o freqScanThread.o scanThread.o \
    forceCurveThread.o laserThread.o motorThread.o closeloop.o restart.o serial_port.o


all : ht7489Server pm511pServer 

ht7489Server : $(GENERAL_OBG) ht7489.o new_dds_DDFS.o new_dds_IO.o 
	$(CC) $(C_FLAG) -o ht7489Server -lm -lpthread $(GENERAL_OBG) ht7489.o new_dds_DDFS.o new_dds_IO.o 

pm511pServer : $(GENERAL_OBG) pm511p.o pm511p_IO.o  new_dds_DDFS.o 
	$(CC) $(C_FLAG) -o pm511pServer -lm -lpthread $(GENERAL_OBG) pm511p.o pm511p_IO.o  new_dds_DDFS.o 

main.o : main.c afm_comm.h work_thread.h hardware.h restart.h
	$(CC) $(C_FLAG) -c main.c

dispatch_cmd.o : dispatch_cmd.c work_thread.h 
	$(CC) $(C_FLAG) -c dispatch_cmd.c

freqScanThread.o : freqScanThread.c work_thread.h hardware.h
	$(CC) $(C_FLAG) -c freqScanThread.c

scanThread.o : scanThread.c work_thread.h hardware.h
	$(CC) $(C_FLAG) -c scanThread.c
	
		
forceCurveThread.o : forceCurveThread.c work_thread.h hardware.h
	$(CC) $(C_FLAG) -c forceCurveThread.c

laserThread.o : laserThread.c work_thread.h hardware.h
	$(CC) $(C_FLAG) -c laserThread.c

motorThread.o : motorThread.c work_thread.h hardware.h
	$(CC) $(C_FLAG) -c motorThread.c

closeloop.o : closeloop.c closeloop.h afm_comm.h hardware.h
	$(CC) $(C_FLAG) -c closeloop.c

ht7489.o : ht7489.c hardware.h
	$(CC) $(C_FLAG) $(FLAG_7489) -c ht7489.c

pm511p.o : pm511p.c hardware.h
	$(CC) $(C_FLAG) $(FLAG_511P) -c pm511p.c

pm511p_IO.o : pm511p_IO.c hardware.h
	$(CC) $(C_FLAG) $(FLAG_511P) -c pm511p_IO.c
	
new_dds_DDFS.o : new_dds_DDFS.c hardware.h
	$(CC) $(C_FLAG) -c new_dds_DDFS.c

new_dds_IO.o : new_dds_IO.c hardware.h
	$(CC) $(C_FLAG) -c new_dds_IO.c

restart.o : restart.c restart.h
	$(CC) $(C_FLAG) -c restart.c

serial_port.o : serial_port.c serial_port.h
	$(CC) $(C_FLAG) -c serial_port.c
	
clean :
	rm -fv *.o;	rm -v ht7489Server pm511pServer

clean.bak :
	rm -fv *.bak