#ifndef GPSM_H
#define GPSM_H

#include <Arduino.h>
#include <SoftwareSerial.h>

#define GPSM_BUFFER_SIZE 80

class GPSM
{
public:
	SoftwareSerial dbg;
	char buf[GPSM_BUFFER_SIZE];
	char buf_eol; // dummy EOL for string safety

	GPSM();
	void begin();

	void gsmPowerUp();
	void gsmSend(const char *cmd);
	byte gsmRecv(int start_timeout = 1000, int intra_timeout = 50);
	byte gsmSendAndRecv(const char *cmd, int start_timeout = 1000, int intra_timeout = 50);
	char *gsmFind(const char *token);

	boolean gprsConnect(byte *ipaddr, word port);

	enum { EXT_MODE, GSM_MODE, GPS_MODE };
	void serialMode(byte mode);
	inline void extSerial() { serialMode(EXT_MODE); }
	inline void gsmSerial() { serialMode(GSM_MODE); }
	inline void gpsSerial() { serialMode(GPS_MODE); }

private:
	byte _mode;

	boolean isModemReady();
};

#endif
