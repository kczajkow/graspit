#ifndef _pcbird_h_
#define _pcbird_h_

#include "windows.h"

class Pcbird {
public:
	enum opMode{POS_ANG, POS_MAT};
private:
	opMode currentOpMode;
	double angles[3];
	double position[3];
	double rotationMatrix[9];

	//#include <inttypes.h>
	//for MS Visual Studio
	typedef unsigned __int32 uint32_t;
	typedef unsigned __int8 uint8_t;

	// struktura z pozycja i katami pcbirda
	typedef struct {
		float x, y, z;	// pozycja
		float a, b, g;	// katy (a = azimuth, b = elevation, g = roll)
		float distance;	// odleglosc
		uint32_t ts_sec, ts_usec;	// timestamp
	} pcbird_pos_t;

	int fd;
	pcbird_pos_t p;

	// polaczenie z serwerem PCBird, zwraca deskryptor socketa
	int pcbird_connect(const char *addr, unsigned short port);

	// koniec sesji PCBird
	void pcbird_disconnect(int fd);

	// rozpoczyna streaming pozycji
	int pcbird_start_streaming(int fd);

	// konczy streaming pozycji
	int pcbird_stop_streaming(int fd);

	// pojedynczy odczyt pozycji
	int pcbird_get_single_position(int fd, pcbird_pos_t *p);

	// czy dane do gniazda nadeszly? (uzywane przy streamingu)
	int pcbird_data_avail(int fd);

	// nieblokujacy odczyt pozycji w trybie streaming
	int pcbird_get_streaming_position(int fd, pcbird_pos_t *p);

	void decode_packet(char *payload, pcbird_pos_t *p);
public:
	Pcbird();
	~Pcbird();

	virtual int instantRead();

	virtual void getPosition(double *p){memcpy(p, position, 3*sizeof(double));}
	virtual void getAngles(double *p){memcpy(p, angles, 3*sizeof(double));}
	virtual void getRotationMatrix(double *p){memcpy(p, rotationMatrix, 9*sizeof(double));}
};

#endif
