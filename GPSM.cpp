#include "GPSM.h"

#define GSM_PIN 3
#define GPS_PIN 4
#define PWR_PIN 5

// GSM RX & TX are connected to Arduino hardware serial (pin 0 & 1),
// accessible via Serial.
//
// #define GSM_RX_PIN 0
// #define GSM_TX_PIN 1

#define GPS_RX_PIN 8
#define GPS_TX_PIN 9

#define CONSOLE_RX_PIN 10
#define CONSOLE_TX_PIN 11

#define ECHO_ENABLED 1

#define G(s) (const char *)F(s)

enum {
	CIICR_TIMEOUT   = 10,
	CONNECT_TIMEOUT = 20,
	SEND_TIMEOUT    = 30,
	CLOSE_TIMEOUT   = 5
};

GSM::GSM() :
	_first_time(1000),
	_intra_time(50),
	_overflow_size(0),
	_overflow_slot(0)
{
	for (int i = 0; i < MAX_CALLBACK; i++)
		_cb[i].func = NULL;
}

void GSM::begin(unsigned long baud) {
	pinMode(GSM_PIN, OUTPUT);
	pinMode(GPS_PIN, OUTPUT);
	pinMode(PWR_PIN, OUTPUT);

	digitalWrite(GSM_PIN, LOW);
	digitalWrite(GPS_PIN, HIGH);

	Serial.begin(baud);
}

void GSM::end() {
	digitalWrite(GSM_PIN, HIGH);
	digitalWrite(GPS_PIN, HIGH);
	Serial.end();
}

void GSM::powerToggle() {
	digitalWrite(PWR_PIN, HIGH);
	delay(2000);
	digitalWrite(PWR_PIN, LOW);
}

void GSM::send(const char *cmd) {
	// Cleanup serial buffer
	while (available())
		recv();

	write(cmd);
	write('\r');

#if ECHO_ENABLED
	console.print(F(":"));
	console.println(cmd);
#endif
}

void GSM::send_P(const char *cmd) {
	// Cleanup serial buffer
	while (available())
		recv();

	Serial.print((__FlashStringHelper *)cmd);
	write('\r');

#if ECHO_ENABLED
	console.print(F(":"));
	console.println((__FlashStringHelper *)cmd);
#endif
}

void GSM::handleCallback() {
	// Handle overflow request first
	size_t pos = 0;
	int i = _overflow_slot;
	if (_overflow_size > 0 && _cb[i].func)
		pos += _cb[i].func(buf, buf_size, _cb[i].data);

	// Handle the rest if we still have available space
	while (pos < buf_size) {
		size_t used = 0;
		for (i = 0; i < MAX_CALLBACK; i++) {
			if (_cb[i].func && !strncmp_P((char *)buf + pos, _cb[i].match, _cb[i].length)) {
				used = _cb[i].func(buf + pos, buf_size - pos, _cb[i].data);
				if (used)
					break;
			}
		}
		pos += used ? used : 1;
	}

	// Callback can request overflow by returning used space
	// beyond buf space
	if (pos > buf_size) {
		_overflow_size = pos - buf_size;
		_overflow_slot = i;
	} else {
		_overflow_size = 0;
	}
}

size_t GSM::recv() {
	unsigned long timeout = millis() + _first_time;
	buf_size = 0;
	while (millis() < timeout) {
		if (available()) {
			buf[buf_size++] = read();
			if (_intra_time > 0)
				timeout = millis() + _intra_time;
			if (buf_size >= GSM_BUFFER_SIZE)
				break;
		}
	}
	buf[buf_size] = 0;
#if ECHO_ENABLED
	if (buf_size > 0)
		console.write((byte *) buf, buf_size);
#endif
	handleCallback();
	return buf_size;
}

int GSM::recvUntil_P(const char *s1, const char *s2, const char *s3) {
	const char *ss[3] = { s1, s2, s3 };
	recv();
	for (int i = 0; i < 3; i++)
		if (ss[i] && strstr_P((char *)buf, ss[i]) != NULL)
			return i + 1;
	return 0;
}

int GSM::recvUntil_P(int tries, const char *s1, const char *s2, const char *s3) {
	int ret = 0;
	for (int i = 0; i < tries && ret == 0; i++)
		ret = recvUntil_P(s1, s2, s3);
	return ret;
}

void GSM::setTimeout(long first_time, long intra_time) {
	_first_time = first_time;
	_intra_time = intra_time;
}

void GSM::loop() {
	if (available())
		recv();
}

void GSM::setCallback_P(int slot, const char *match, callback_func func, void *data) {
	_cb[slot].match = match;
	_cb[slot].length = strlen_P((char *)match);
	_cb[slot].data = data;
	_cb[slot].func = func;
}

boolean GSM::isModemReady() {
	boolean ready = false;
	for (int i = 0; i < 2 && !ready; i++)
		ready = sendRecvUntil_P(G("AT"), G("OK"));
	if (ready)
		sendRecv_P(G("ATE0"));
	return ready;
}

boolean GSM::isRegistered() {
	return sendRecvUntil_P(G("AT+CREG?"), G("+CREG: 0,1"));
}

boolean GSM::isAttached() {
	return sendRecvUntil_P(G("AT+CGATT?"), G("+CGATT: 1"));
}

#define IMEI_LENGTH 14

boolean GSM::getIMEI(char *imei) {
	if (!sendRecvUntil_P(G("AT+GSN"), G("OK")))
		return false;
	int len = 0;
	for (size_t i = 0; i < buf_size && len < IMEI_LENGTH; i++) {
		if ('0' <= buf[i] && buf[i] <= '9')
			imei[len++] = buf[i];
		else if (len > 0)
			break;
	}
	imei[len] = 0;
	return true;
}

/*********************************
 * GPRSClient class implemetation
 *********************************/

GPRSClient::GPRSClient() : _connected(0) {
}

void GPRSClient::setParams(const char *apn, const char *user, const char *pass) {
	_apn = apn;
	_user = user;
	_pass = pass;

	char fmt[25];
	char cmd[40];
	strcpy_P(fmt, G("AT+CGDCONT=1,\"IP\",\"%s\""));
	snprintf(cmd, sizeof(cmd), fmt, _apn);
	gsm.send(cmd);
	gsm.recv();
	gsm.sendRecv_P(G("AT+CIPHEAD=1"));
}

void GPRSClient::getStatus(char *status, size_t length) {
	gsm.sendRecv_P(G("AT+CIPSTATUS"));
	char *p = strstr_P((char *)gsm.buf, G("STATE: "));
	if (p == NULL) {
		status[0] = 0;
	} else {
		p += 7;
		char *eol = strchr(p, '\r');
		if (eol)
			*eol = 0;
		strncpy(status, p, length - 1);
		status[length - 1] = 0;
	}
}

boolean GPRSClient::attach() {
	if (!gsm.isRegistered() || !gsm.isAttached())
		return false;

	char cmd[80];
	getStatus(cmd, sizeof(cmd));

	// If state in "CONNECT OK", "TCP CONNECTING", "UDP CONNECTING"
	// We use "CONNECT" as a shortcut to save space
	if (strstr_P(cmd, G("CONNECT"))) {
		// We shouldn't be here normally, but this can happen when hot resetting device
		_connected = true;
		stop();
		getStatus(cmd, sizeof(cmd));
	}

	if (!strcmp_P(cmd, G("IP CLOSE")))
		return true;

	if (!strcmp_P(cmd, G("PDP DEACT")))
		gsm.sendRecvUntil_P(G("AT+CIPSHUT"), CLOSE_TIMEOUT, G("SHUT OK"), G("ERROR"));

	// State 0: IP INITIAL
	if (!strcmp_P(cmd, G("IP INITIAL"))) {
		char fmt[25];
		strcpy_P(fmt, G("AT+CSTT=\"%s\",\"%s\",\"%s\""));
		snprintf(cmd, sizeof(cmd), fmt, _apn, _user, _pass);
		gsm.send(cmd);
		if (!gsm.recvUntil_P(G("OK")))
			return false;
		getStatus(cmd, sizeof(cmd));
	}

	// State 1: IP START
	if (!strcmp_P(cmd, G("IP START"))) {
		gsm.send_P(G("AT+CIICR"));
		if (gsm.recvUntil_P(CIICR_TIMEOUT, G("OK"), G("ERROR")) != 1)
			return false;
		getStatus(cmd, sizeof(cmd));
	}

	// State 2: IP CONFIG
	while (!strcmp_P(cmd, G("IP CONFIG"))) {
		delay(1000);
		getStatus(cmd, sizeof(cmd));
	}

	// State 3: IP GPRSACT
	if (!strcmp_P(cmd, G("IP GPRSACT"))) {
		if (!gsm.sendRecvUntil_P(G("AT+CIFSR"), G(".")))
			return false;
		getStatus(cmd, sizeof(cmd));
	}

	return !strcmp_P(cmd, G("IP STATUS"));
}

int GPRSClient::connect(IPAddress ip, uint16_t port) {
	if (!attach())
		return 0;

	// State 4: IP STATUS
	char cmd[80];
	char fmt[40];
	strcpy_P(fmt, G("AT+CIPSTART=\"TCP\",\"%d.%d.%d.%d\",\"%d\""));
	snprintf(cmd, sizeof(cmd), fmt, ip[0], ip[1], ip[2], ip[3], port);
	gsm.send(cmd);
	if (!gsm.recvUntil_P(G("OK")))
		return 0;

	_connected = gsm.recvUntil_P(CONNECT_TIMEOUT, G("CONNECT OK"), G("CONNECT FAIL")) == 1;
	if (_connected) {
		_rx_head = _rx_tail = _size_left = 0;
		gsm.setCallback_P(0, G("+IPD"), callback, (void *)this);
	}

	return _connected;
}

int GPRSClient::connect(const char *host, uint16_t port) {
	return false;
}

size_t GPRSClient::write(uint8_t b) {
	return write(&b, 1);
}

size_t GPRSClient::write(const uint8_t *buf, size_t size) {
	if (_connected) {
		char fmt[20];
		char cmd[20];
		strcpy_P(fmt, G("AT+CIPSEND=%u"));
		snprintf(cmd, sizeof(cmd), fmt, size);
		gsm.send(cmd);
		gsm.recvUntil_P(G("> "));
		gsm.write(buf, size);

		if (gsm.recvUntil_P(SEND_TIMEOUT, G("SEND OK"), G("SEND FAIL"), G("ERROR")) == 1)
			return size;

		console.println(F("failed"));
		stop();
	}
	return 0;
}

int GPRSClient::available() {
	if (!_in_loop) {
		_in_loop = 1;
		gsm.loop();
		_in_loop = 0;
	}

	return _rx_tail >= _rx_head ?
	       	_rx_tail - _rx_head :
		_rx_tail - _rx_head + sizeof(_rx_buf);
}

int GPRSClient::read() {
	int c = -1;
	if (available()) {
		c = _rx_buf[_rx_head];
		_rx_head = (_rx_head + 1) % sizeof(_rx_buf);
	}
	return c;
}

int GPRSClient::read(uint8_t *buf, size_t size) {
	size_t i;
	for (i = 0; i < size; i++) {
		int c = read();
		if (c == -1)
			break;
		buf[i] = c;
	}
	return i;
}

int GPRSClient::peek() {
	return available() ? _rx_buf[_rx_head] : -1;
}

void GPRSClient::flush() {
	return;
}

void GPRSClient::stop() {
	if (_connected) {
		_connected = 0;
		gsm.setCallback_P(0, NULL, NULL, NULL);
		gsm.sendRecvUntil_P(G("AT+CIPCLOSE"), CLOSE_TIMEOUT, G("CLOSE OK"), G("ERROR"));
	}
}

uint8_t GPRSClient::connected() {
	return _connected;
}

GPRSClient::operator bool() {
	return true;
}

boolean GPRSClient::isFull() {
	return _rx_head == nextIndex(_rx_tail);
}

size_t GPRSClient::callback(byte *buf, size_t length, void *data) {
	size_t used = 0;
	GPRSClient *client = (GPRSClient *)data;
	if (client->_size_left == 0) {
		char *p = strstr_P((char *)buf, G("+IPD"));
		if (p) {
			p += 4;
			char *end = strchr(p, ':');
			if (end) {
				*end = 0;
				client->_size_left = atoi(p);
				used = (byte *)end - buf + 1;
			}
		}
	}
	while (client->_size_left > 0 && used < length) {
		// Data will be discarded when buffer is full
		if (!client->isFull()) {
			client->_rx_buf[client->_rx_tail] = buf[used++];
			client->_rx_tail = client->nextIndex(client->_rx_tail);
		}
		client->_size_left--;
	}
	return used + client->_size_left;
}

/*
 * Predefined objects
 */

GSM gsm;
SoftwareSerial gps(GPS_RX_PIN, GPS_TX_PIN);
SoftwareSerial console(CONSOLE_RX_PIN, CONSOLE_TX_PIN);
GPRSClient gprsClient;
