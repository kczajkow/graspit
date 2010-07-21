
#include <stdio.h>
#include <stdlib.h>
#include <io.h>
#include <sys/types.h>
//#include <inttypes.h>
#include <fcntl.h>
#include <string.h>
//#include <pthread.h>
//#include <unistd.h>
#include <time.h>
//#include <socket.h>
#include <winsock2.h>
//#include <arpa/inet.h>
#include <errno.h>
//#include <poll.h>
//#include <netdb.h>
#define _USE_MATH_DEFINES
#include <math.h>

#include "Pcbird.h"

#define MAX_RANGE ( 36.0 * 0.0254 )

#define SVCMD_POS 1
#define SVCMD_START_STREAMING 2
#define SVCMD_STOP_STREAMING 3
#define SVCMD_END_SESSION 4

int Pcbird::instantRead()
{
	//do {;}
	//while (!pcbird_data_avail(fd));

	//pcbird_get_streaming_position(fd, &p);
	pcbird_get_single_position(fd, &p);

	angles[0] = -1.0 * p.a + 90;
	angles[1] = -1.0 * p.g + 0;
	angles[2] = -1.0 * p.b + 0;
	position[0] = 1.0 * p.x*1000 - 395;
	position[1] = -1.0 * p.y*1000 + 1133;
	position[2] = -1.0 * p.z*1000 + 360;
	float Zang = angles[0] * M_PI / 180;
	float Yang = angles[1] * M_PI / 180;
	float Xang = angles[2] * M_PI / 180;
	//rotationMatrix[0] = cos(Yang)*cos(Zang);
	//rotationMatrix[1] = cos(Yang)*sin(Zang);
	//rotationMatrix[2] = -sin(Yang);
	//rotationMatrix[3] = -cos(Xang)*sin(Zang)+sin(Xang)*sin(Yang)*cos(Zang);
	//rotationMatrix[4] = cos(Xang)*cos(Zang)+sin(Xang)*sin(Yang)*sin(Zang);
	//rotationMatrix[5] = sin(Xang)*cos(Yang);
	//rotationMatrix[6] = sin(Xang)*sin(Zang)+cos(Xang)*sin(Yang)*cos(Zang);
	//rotationMatrix[7] = -sin(Xang)*cos(Zang)+cos(Xang)*sin(Yang)*sin(Zang);
	//rotationMatrix[8] = cos(Xang)*cos(Yang);
	rotationMatrix[0] = cos(Yang)*cos(Zang);
	rotationMatrix[1] = -cos(Xang)*sin(Zang)+sin(Xang)*sin(Yang)*cos(Zang);
	rotationMatrix[2] = sin(Xang)*sin(Zang)+cos(Xang)*sin(Yang)*cos(Zang);
	rotationMatrix[3] = cos(Yang)*sin(Zang);
	rotationMatrix[4] = cos(Xang)*cos(Zang)+sin(Xang)*sin(Yang)*sin(Zang);
	rotationMatrix[5] = -sin(Xang)*cos(Zang)+cos(Xang)*sin(Yang)*sin(Zang);
	rotationMatrix[6] = -sin(Yang);
	rotationMatrix[7] = sin(Xang)*cos(Yang);
	rotationMatrix[8] = cos(Xang)*cos(Yang);

	return 1;
}

Pcbird::Pcbird()
{
	fprintf(stderr,"Obiekt Pcbird utworzony!!!\n");

	fd = pcbird_connect("pcbird", 12345);

	if(fd <= 0)
		fprintf(stderr,"B³¹d po³¹czenia z Pcbird!!!\n");

    //pcbird_start_streaming(fd);
}

Pcbird::~Pcbird()
{
	//pcbird_stop_streaming(fd);
	pcbird_disconnect(fd);
}

int Pcbird::pcbird_connect(const char *addr, unsigned short port)
{
    int s, len;
    struct sockaddr_in remote;
    struct hostent *he;
    
	WSADATA wsa;
    WSAStartup(MAKEWORD(1,1),&wsa);

    he = gethostbyname(addr);
    
    if(!he) return -1;
    
    if ((s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) == -1) 
        return -1;

    remote.sin_family = AF_INET;
    remote.sin_port = htons(port);
    remote.sin_addr.s_addr = *((unsigned long*)he->h_addr);

    len = sizeof(struct sockaddr_in);

    if (connect(s, (struct sockaddr *)&remote, len) == -1) 
        return -1;

    fprintf(stderr, "pcbird_connect(): fd = %d\n", s);
    return s;
}

void Pcbird::pcbird_disconnect(int fd)
{
    char b = SVCMD_END_SESSION;
    send(fd, &b, 1, 0);
    closesocket(fd);
}

int Pcbird::pcbird_start_streaming(int fd)
{
    char b = SVCMD_START_STREAMING;
    return send(fd, &b, 1, 0) == 1 ? 0 : -1;    
}

int Pcbird::pcbird_stop_streaming(int fd)
{
    char b = SVCMD_STOP_STREAMING;
    return send(fd, &b, 1, 0) == 1 ? 0 : -1;    
}

void Pcbird::decode_packet(char *payload, pcbird_pos_t *p)
{
    short xp, yp, zp, ap, bp, gp;
    
    xp = ntohs(*(short *) &payload[0]);
    yp = ntohs(*(short *) &payload[2]);
    zp = ntohs(*(short *) &payload[4]);

    ap = ntohs(*(short *) &payload[6]);
    bp = ntohs(*(short *) &payload[8]);
    gp = ntohs(*(short *) &payload[10]);

    p->x = ((float) xp) * MAX_RANGE / 32768.0 ;
    p->y = ((float) yp) * MAX_RANGE / 32768.0 ;
    p->z = ((float) zp) * MAX_RANGE / 32768.0 ;

    p->a = ((float) ap) * 180.0 / 32768.0;
    p->b = ((float) bp) * 180.0 / 32768.0;
    p->g = ((float) gp) * 180.0 / 32768.0;

    p->distance = sqrt(p->x * p->x + p->y * p->y + p->z * p->z);
    
    p->ts_sec = ntohl(*(int *)&payload[12]);
    p->ts_usec = ntohl(*(int *)&payload[16]);
    
}

int Pcbird::pcbird_get_single_position(int fd, pcbird_pos_t *p)
{
    const char b = SVCMD_POS;
    char rxbuf[20];

    if( send(fd, &b, 1, 0) != 1 ) return -1; 
    if( recv(fd, rxbuf, 20, 0) != 20) return -1; 
    
    decode_packet(rxbuf, p);
    
    return 0;
}

int Pcbird::pcbird_data_avail(int fd)
{
    u_long argp = 0;
    
    ioctlsocket(fd, FIONREAD, &argp);

    if(argp > 0) return 1; 
    
    return 0;
}

int Pcbird::pcbird_get_streaming_position(int fd, pcbird_pos_t *p)
{
    char rxbuf[20];

    if( recv(fd, rxbuf, 20, 0) != 20) return -1; 
    decode_packet(rxbuf, p);
    
    return 0;
}
