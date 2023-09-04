echo Executing GDB with .gdbinit to connect to OpenOCD.\n

target remote localhost:3333  

monitor reset init 

monitor flash write_image erase final.elf 

monitor reset halt

