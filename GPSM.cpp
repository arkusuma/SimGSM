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

enum {
	CIICR_TIMEOUT   = 10,
	CONNECT_TIMEOUT = 10,
	SEND_TIMEOUT    = 30
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
	console.print(":");
	console.println(cmd);
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
			if (_cb[i].func && !strncmp((char *)buf + pos, _cb[i].match, _cb[i].length)) {
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
	handleCallback();

#if ECHO_ENABLED
	if (buf_size > 0)
		console.write((byte *) buf, buf_size);
#endif

	return buf_size;
}

int GSM::recvUntil(const char *s1, const char *s2, const char *s3) {
	const char *ss[3] = { s1, s2, s3 };
	recv();
	for (int i = 0; i < 3; i++)
		if (ss[i] && strstr((char *)buf, ss[i]) != NULL)
			return i + 1;
	return 0;
}

int GSM::recvUntil(int tries, const char *s1, const char *s2, const char *s3) {
	int ret = 0;
	for (int i = 0; i < tries && ret == 0; i++)
		ret = recvUntil(s1, s2, s3);
	return ret;
}

void GSM::setTimeout(long first_time, long intra_time) {
	_first_time = first_time;
	_intra_time = intra_time;
}

size_t GSM::sendAndRecv(const char *cmd) {
	send(cmd);
	return recv();
}

int GSM::sendAndRecvUntil(const char *cmd, const char *s1, const char *s2, const char *s3) {
	send(cmd);
	return recvUntil(s1, s2, s3);
}

void GSM::loop() {
	if (available())
		recv();
}

void GSM::setCallback(int slot, const char *match, callback_func func, void *data) {
	_cb[slot].match = match;
	_cb[slot].length = strlen(match);
	_cb[slot].data = data;
	_cb[slot].func = func;
}

boolean GSM::isModemReady() {
	boolean ready = false;
	for (int i = 0; i < 2 && !ready; i++)
		ready = sendAndRecvUntil("AT", "OK");
	if (ready)
		sendAndRecv("ATE0");
	return ready;
}

boolean GSM::isRegistered() {
	return sendAndRecvUntil("AT+CREG?", "+CREG: 0,1");
}

boolean GSM::isAttached() {
	return sendAndRecvUntil("AT+CGATT?", "+CGATT: 1");
}

#define IMEI_LENGTH 14

boolean GSM::getIMEI(char *imei) {
	if (!sendAndRecvUntil("AT+GSN", "OK"))
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

	char cmd[40];
	snprintf(cmd, sizeof(cmd), "AT+CGDCONT=1,\"IP\",\"%s\"", _apn);
	gsm.sendAndRecv(cmd);
	gsm.sendAndRecv("AT+CIPHEAD=1");
}

void GPRSClient::getStatus(char *status, size_t length) {
	gsm.sendAndRecv("AT+CIPSTATUS");
	char *p = strstr((char *)gsm.buf, "STATE: ");
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

	stop();

	char cmd[80];
	getStatus(cmd, sizeof(cmd));
	if (!strcmp(cmd, "IP INITIAL")) {
		// State 0: IP INITIAL
		snprintf(cmd, sizeof(cmd), "AT+CSTT=\"%s\",\"%s\",\"%s\"",
				_apn, _user, _pass);
		if (!gsm.sendAndRecvUntil(cmd, "OK"))
			return false;
		getStatus(cmd, sizeof(cmd));
	}

	// State 1: IP START
	if (!strcmp(cmd, "IP START")) {
		gsm.send("AT+CIICR");
		if (gsm.recvUntil(CIICR_TIMEOUT, "OK", "ERROR") != 1)
			return false;
		getStatus(cmd, sizeof(cmd));
	}

	// State 2: IP CONFIG
	while (!strcmp(cmd, "IP CONFIG")) {
		delay(1000);
		getStatus(cmd, sizeof(cmd));
	}

	// State 3: IP GPRSACT
	if (!strcmp(cmd, "IP GPRSACT")) {
		if (!gsm.sendAndRecvUntil("AT+CIFSR", "."))
			return false;
		getStatus(cmd, sizeof(cmd));
	}

	return !strcmp(cmd, "IP STATUS");
}

int GPRSClient::connect(IPAddress ip, uint16_t port) {
	if (!attach()) {
		// Retry once more after shutting down PDP
		gsm.sendAndRecv("AT+CIPSHUT");
		if (!attach())
			return 0;
	}

	// State 4: IP STATUS
	char cmd[80];
	snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"TCP\",\"%d.%d.%d.%d\",\"%d\"",
			ip[0], ip[1], ip[2], ip[3], port);
	if (!gsm.sendAndRecvUntil(cmd, "OK"))
		return 0;

	_connected = gsm.recvUntil(CONNECT_TIMEOUT, "CONNECT OK", "CONNECT FAIL") == 1;
	if (_connected) {
		_rx_head = _rx_tail = _size_left = 0;
		gsm.setCallback(0, "+IPD", callback, (void *)this);
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
		char cmd[40];
		sprintf(cmd, "AT+CIPSEND=%u", size);
		gsm.sendAndRecvUntil(cmd, "> ");
		gsm.write(buf, size);

		if (gsm.recvUntil(SEND_TIMEOUT, "SEND OK", "SEND FAIL", "ERROR") == 1)
			return size;

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
		gsm.setCallback(0, NULL, NULL, NULL);
		gsm.sendAndRecv("AT+CIPCLOSE");
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
		char *p = strstr((char *)buf, "+IPD");
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
