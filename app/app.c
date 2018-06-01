#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>

#define DEV_FILE "/dev/stopwatch"
#define MAX_DIGIT 4	// MAX value for fnd_device

int main(void)
{
	int dev_fd;
	dev_fd = open(DEV_FILE, O_WRONLY);
	if(dev_fd<0){
		printf("Open Driver Failured!\n");
		return -1;
	}
	
	write(dev_fd, NULL, 0);
	close(dev_fd);
	
	printf("Success.\n");
	return 0;
}
