#ifndef GPRSClient_h
#define GPRSClient_h

#include "SimGSM.h"

enum {
	CIICR_TIMEOUT   = 10,
	CONNECT_TIMEOUT = 20,
	SEND_TIMEOUT    = 30,
	CLOSE_TIMEOUT   = 5
};

#define GPRS_BUFFER_SIZE 64

class GPRSClient: public Client {
public:
	GPRSClient(SimGSM &gsm);

	void setParams(const char *apn, const char *user, const char *pass);

	virtual int connect(IPAddress ip, uint16_t port);
	virtual int connect(const char *host, uint16_t port);
	virtual size_t write(uint8_t);
	virtual size_t write(const uint8_t *buf, size_t size);
	virtual int available();
	virtual int read();
	virtual int read(uint8_t *buf, size_t size);
	virtual int peek();
	virtual void flush();
	virtual void stop();
	virtual uint8_t connected();
	virtual operator bool();
	using Print::write;

private:
	SimGSM &_gsm;
	const char *_apn;
	const char *_user;
	const char *_pass;
	boolean _connected;
	boolean _in_loop;

	size_t _size_left;
	byte _rx_buf[GPRS_BUFFER_SIZE];
	byte _rx_head;
	byte _rx_tail;

	boolean attach();
	boolean isFull();
	void getStatus(char *status, size_t length);
	static size_t callback(byte *buf, size_t length, void *data);
	static inline byte nextIndex(byte i) { return i == GPRS_BUFFER_SIZE - 1 ? 0 : i + 1; }
};

#endif
