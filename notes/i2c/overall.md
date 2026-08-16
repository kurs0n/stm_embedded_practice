i2c have two lines:

SDA -> for data
SCL -> for clock 

master generate a clock


on SDA line there is a start signal and end signal to stop transmission 

first byte is for address and read/write bit (if the master wants to write to slave device or read from him)

next bytes are just data bytes (from slave or from master it depends on read/write bit) (each byte needs to be acknowledged)

after successful transmission with all acknowledged bytes there is a end signal produced by master


clock stretching: (write notes here)

why we need to use configuration open drain for i2c?

because we can have multi master environment and there could be short circuit without pull up resistor in open drain