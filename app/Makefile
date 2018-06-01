#CC		= gcc
CC		= arm-none-linux-gnueabi-gcc
CFLAGS	= -W -Wall
TARGET	= app

all : $(TARGET)

$(TARGET) : app.o
	$(CC) $(CFLAGS) -static -o $(TARGET) app.o

app.o : app.c
	$(CC) $(CFLAGS) -static -c -o app.o app.c

clean :
	rm -rf *.o app
