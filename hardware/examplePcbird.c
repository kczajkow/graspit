#include "birdclient.h"

main()
{
    int fd = pcbird_connect("localhost", 12345);

    if(fd<=0) {
	perror("connect()");
	return -1;
    }

    pcbird_start_streaming(fd);
    
    for(;;)
    {
	pcbird_pos_t p;

	if(pcbird_data_avail(fd))
	{
	    pcbird_get_streaming_position(fd, &p);
	    
	    printf("[x, y, z] = [%.3f, %.3f, %.3f] ", p.x, p.y, p.z);
	    printf("[a, b, g] = [%.3f, %.3f, %.3f] ", p.a, p.b, p.g);
	    printf("dist = %.3f ts=%d:%d\n", p.distance, p.ts_sec, p.ts_usec);

	}

	usleep(10000);
    }
}
