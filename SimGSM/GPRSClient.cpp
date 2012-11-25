#include "GPRSClient.h"

GPRSClient::GPRSClient(SimGSM &gsm) :
	_gsm(gsm),
	_connected(0) {}

void GPRSClient::setParams(const char *apn, const char *user, const char *pass) {
	_apn = apn;
	_user = user;
	_pass = pass;

	char fmt[25];
	char cmd[40];
	strcpy_P(fmt, G("AT+CGDCONT=1,\"IP\",\"%s\""));
	snprintf(cmd, sizeof(cmd), fmt, _apn);
	_gsm.send(cmd);
	_gsm.recv();
	_gsm.sendRecv_P(G("AT+CIPHEAD=1"));
}

void GPRSClient::getStatus(char *status, size_t length) {
	_gsm.sendRecv_P(G("AT+CIPSTATUS"));
	char *p = _gsm.find_P(G("STATE: "));
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
	if (!_gsm.isRegistered() || !_gsm.isAttached())
		return false;

	char cmd[80];
	getStatus(cmd, sizeof(cmd));

	// If state is in "CONNECT OK", "TCP CONNECTING", "UDP CONNECTING", close it.
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
		_gsm.sendRecvUntil_P(G("AT+CIPSHUT"), CLOSE_TIMEOUT, G("SHUT OK"), G("ERROR"));

	// State 0: IP INITIAL
	if (!strcmp_P(cmd, G("IP INITIAL"))) {
		char fmt[25];
		strcpy_P(fmt, G("AT+CSTT=\"%s\",\"%s\",\"%s\""));
		snprintf(cmd, sizeof(cmd), fmt, _apn, _user, _pass);
		_gsm.send(cmd);
		if (!_gsm.recvUntil_P(G("OK")))
			return false;
		getStatus(cmd, sizeof(cmd));
	}

	// State 1: IP START
	if (!strcmp_P(cmd, G("IP START"))) {
		_gsm.send_P(G("AT+CIICR"));
		if (_gsm.recvUntil_P(CIICR_TIMEOUT, G("OK"), G("ERROR")) != 1)
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
		if (!_gsm.sendRecvUntil_P(G("AT+CIFSR"), G(".")))
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
	_gsm.send(cmd);
	if (!_gsm.recvUntil_P(G("OK")))
		return 0;

	_connected = _gsm.recvUntil_P(CONNECT_TIMEOUT, G("CONNECT OK"), G("CONNECT FAIL")) == 1;
	if (_connected) {
		_rx_head = _rx_tail = _size_left = 0;
		_gsm.setCallback_P(0, G("+IPD"), callback, (void *)this);
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
		_gsm.send(cmd);
		_gsm.recvUntil_P(G("> "));
		_gsm.serial().write(buf, size);

		if (_gsm.recvUntil_P(SEND_TIMEOUT, G("SEND OK"), G("SEND FAIL"), G("ERROR")) == 1)
			return size;

		console.println(F("failed"));
		stop();
	}
	return 0;
}

int GPRSClient::available() {
	if (!_in_loop) {
		_in_loop = 1;
		_gsm.loop();
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
		_gsm.setCallback_P(0, NULL, NULL, NULL);
		_gsm.sendRecvUntil_P(G("AT+CIPCLOSE"), CLOSE_TIMEOUT, G("CLOSE OK"), G("ERROR"));
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
			client->_rx_tail = nextIndex(client->_rx_tail);
		}
		client->_size_left--;
	}
	return used + client->_size_left;
}
