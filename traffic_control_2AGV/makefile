all: traffic_control_2AGV.out

#declare variables
CC:=gcc
exe:=traffic_control_2AGV
obj:=traffic_control_2AGV.o

traffic_control_2AGV: $(obj)
  $(CC) -o $(exe) $(obj)  

.PHONY:clean
clean:
  rm -rf $(obj) $(exe)
