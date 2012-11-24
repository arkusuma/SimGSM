#ifndef GPSM_H
#define GPSM_H

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Print.h>
#include <Client.h>
#include <IPAddress.h>

#define GSM_BUFFER_SIZE 80
#define MAX_CALLBACK 3

class GSM {
public:
	typedef size_t (*callback_func)(byte *buf, size_t length, void *data);

	byte buf[GSM_BUFFER_SIZE];
	byte buf_eol; // dummy EOL for string safety
	size_t buf_size;

	GSM();
	void begin(unsigned long);
	void end();

	void powerToggle();

	// Send AT command.  Don't use '\r' as command suffix.
	void send(const char *);

	// Receive until buffer is full or timeout.
	size_t recv();

	// Receive until specified token found or timeout.
	// Returns 1, 2, 3 depending on matched parameter. Or 0 if none found.
	int recvUntil(const char *, const char * = NULL, const char * = NULL);
	int recvUntil(int tries, const char *, const char * = NULL, const char * = NULL);

	// Set timeout for recv() and recvUntil()
	void setTimeout(long = 1000, long = 50);

	size_t sendAndRecv(const char *);
	int sendAndRecvUntil(const char *, const char *, const char * = NULL, const char * = NULL);

	// Must be called frequently to check incoming data
	void loop();
	void setCallback(int slot, const char *match, callback_func func, void *data);

	// Modem status testing functions
	boolean isModemReady();
	boolean isRegistered();
	boolean isAttached();
	boolean getIMEI(char *buf);

	// Emulation of some methods from Serial class
	inline int available() { return Serial.available(); }
	inline int read() { return Serial.read(); }
	inline int peek() { return Serial.peek(); }
	inline void flush() { Serial.flush(); }
	inline size_t write(uint8_t c) { return Serial.write(c); }
	inline size_t write(unsigned long n) { return write((uint8_t)n); }
	inline size_t write(long n) { return write((uint8_t)n); }
	inline size_t write(unsigned int n) { return write((uint8_t)n); }
	inline size_t write(int n) { return write((uint8_t)n); }
	inline size_t write(const char *str) { return Serial.write(str); }
	inline size_t write(const uint8_t *buffer, size_t size) { return Serial.write(buffer, size); }

private:
	unsigned long _first_time;
	unsigned long _intra_time;
	struct {
		callback_func func;
		const char *match;
		byte length;
		void * data;
	} _cb[MAX_CALLBACK];
	size_t _overflow_size;
	byte _overflow_slot;

	void handleCallback();
};

class GPRSClient: public Client {
public:
	GPRSClient();

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

	void loop();

private:
	const char *_apn;
	const char *_user;
	const char *_pass;
	boolean _connected;
	boolean _in_loop;

	size_t _size_left;
	byte _rx_buf[128];
	byte _rx_head;
	byte _rx_tail;

	boolean attach();
	boolean isFull();
	void getStatus(char *status, size_t length);
	static size_t callback(byte *buf, size_t length, void *data);
	inline byte nextIndex(byte i) { return i == sizeof(_rx_buf) - 1 ? 0 : i + 1; }
};

extern GSM gsm;
extern SoftwareSerial gps;
extern SoftwareSerial console;
extern GPRSClient gprsClient;

#endif
