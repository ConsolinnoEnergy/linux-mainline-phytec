### Build conegx driver with debug output

In the makefile in drivers/gpio/ add the following line:

CFLAGS_gpio-conegx.o := -DDEBUG