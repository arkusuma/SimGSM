#ifndef GPSM_H
#define GPSM_H

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Print.h>
#include <Client.h>
#include <IPAddress.h>
#include <avr/pgmspace.h>

// DFRobot GPS/GSM/GPRS shield shares its GPS & GSM serial into
// Arduino RX & TX pins (pin 0 & 1).  Only one serial can be used
// at one time, and it is controlled via tri-state buffers.

#define GSM_PIN 3 // Tri-state buffer control pin to enable GSM serial
#define GPS_PIN 4 // Tri-state buffer control pin to enable GPS serial
#define PWR_PIN 5

// GPS can also be used with dedicated serial, but you must wired it
// up manually. There are GPS TXA & GPS RXA soldering pad available
// on SIM548C module.
#define GPS_RX_PIN 8
#define GPS_TX_PIN 9

#define GSM_BUFFER_SIZE 64
#define GSM_MAX_CALLBACK 3

//#define ECHO_ENABLED
#ifdef ECHO_ENABLED
extern HardwareSerial &console;
#endif

// Workaround for http://gcc.gnu.org/bugzilla/show_bug.cgi?id=34734
#ifdef PROGMEM
#undef PROGMEM
#define PROGMEM __attribute__((section(".progmem.data")))
#endif

// Arduino F() macro will create PROGMEM string of type (const __FlashStringHelper *).
// This G() macro will cast it into (const char *) for use with C library.
#define G(s) (const char *)F(s)

enum { EXT_MODE, GSM_MODE, GPS_MODE };

// Functions with name ended with _P should be provided with
// PROGMEM value for its const char * parameter.
class SimGSM {
public:
	SimGSM(HardwareSerial &serial);
	void begin(unsigned long baud);
	void end();

	void powerToggle();

	// Send AT command.  Don't use '\r' as command suffix.
	void send(const char *cmd);
	void send_P(const char *cmd);

	// Receive until buffer is full or timeout.
	size_t recv();

	// Find string inside receive buffer
	char *find_P(const char *needle);

	// Receive until specified token found or timeout.
	// Returns 1, 2, 3 depending on matched parameter. Or 0 if none found.
	int recvUntil_P(const char *s1, const char *s2 = NULL, const char *s3 = NULL);
	int recvUntil_P(int tries, const char *s1, const char *s2 = NULL, const char *s3 = NULL);

	// Utility functions
	inline size_t sendRecv_P(const char *cmd) {
		send_P(cmd);
		return recv();
	}
	inline int sendRecvUntil_P(const char *cmd,
			const char *s1, const char *s2 = NULL, const char *s3 = NULL) {
		send_P(cmd);
		return recvUntil_P(s1, s2, s3);
	}
	inline int sendRecvUntil_P(const char *cmd, int tries,
			const char *s1, const char *s2 = NULL, const char *s3 = NULL) {
		send_P(cmd);
		return recvUntil_P(tries, s1, s2, s3);
	}

	// Set timeout for recv() and recvUntil()
	void setTimeout(long first_time = 1000, long intra_time = 50);

	// Must be called frequently to check incoming data
	void loop();

	typedef size_t (*callback_func)(byte *buf, size_t length, void *data);
	void setCallback_P(int slot, const char *match, callback_func func, void *data);

	void serialMode(int mode);

	// Modem status testing functions
	boolean isModemReady();
	boolean isRegistered();
	boolean isAttached();
	boolean getIMEI(char *buf);

	inline HardwareSerial &serial() { return *_serial; }

private:
	HardwareSerial *_serial;
	byte _buf[GSM_BUFFER_SIZE];
	byte _buf_eol; // dummy EOL for string safety
	size_t _buf_size;
	unsigned long _first_time;
	unsigned long _intra_time;
	size_t _overflow_size;
	byte _overflow_slot;

	struct {
		callback_func func;
		const char *match;
		byte length;
		void *data;
	} _cb[GSM_MAX_CALLBACK];

	void handleCallback();
};

#endif
