#include "SimGSM.h"

SimGSM::SimGSM(HardwareSerial &serial) :
	_serial(&serial),
	_first_time(1000),
	_intra_time(50),
	_overflow_size(0),
	_overflow_slot(0),
	_cb() {}

void SimGSM::begin(unsigned long baud) {
	pinMode(GSM_PIN, OUTPUT);
	pinMode(GPS_PIN, OUTPUT);
	pinMode(PWR_PIN, OUTPUT);

	_serial->begin(baud);
}

void SimGSM::end() {
	_serial->end();
}

void SimGSM::powerToggle() {
	digitalWrite(PWR_PIN, HIGH);
	delay(2000);
	digitalWrite(PWR_PIN, LOW);
}

void SimGSM::send(const char *cmd) {
	// Cleanup serial buffer
	while (_serial->available())
		recv();

	_serial->write(cmd);
	_serial->write('\r');

#ifdef ECHO_ENABLED
	console.print(F(":"));
	console.println(cmd);
#endif
}

void SimGSM::send_P(const char *cmd) {
	// Cleanup serial buffer
	while (_serial->available())
		recv();

	_serial->print((__FlashStringHelper *)cmd);
	_serial->write('\r');

#ifdef ECHO_ENABLED
	console.print(F(":"));
	console.println((__FlashStringHelper *)cmd);
#endif
}

void SimGSM::handleCallback() {
	// Handle overflow request first
	size_t pos = 0;
	int i = _overflow_slot;
	if (_overflow_size > 0 && _cb[i].func)
		pos += _cb[i].func(_buf, _buf_size, _cb[i].data);

	// Handle the rest if we still have available space
	while (pos < _buf_size) {
		size_t used = 0;
		for (i = 0; i < GSM_MAX_CALLBACK; i++) {
			if (_cb[i].func && !strncmp_P((char *)_buf + pos, _cb[i].match, _cb[i].length)) {
				used = _cb[i].func(_buf + pos, _buf_size - pos, _cb[i].data);
				if (used)
					break;
			}
		}
		pos += used ? used : 1;
	}

	// Callback can request overflow by returning used space
	// beyond _buf space
	if (pos > _buf_size) {
		_overflow_size = pos - _buf_size;
		_overflow_slot = i;
	} else {
		_overflow_size = 0;
	}
}

size_t SimGSM::recv() {
	unsigned long timeout = millis() + _first_time;
	_buf_size = 0;
	while (millis() < timeout) {
		if (_serial->available()) {
			_buf[_buf_size++] = _serial->read();
			if (_intra_time > 0)
				timeout = millis() + _intra_time;
			if (_buf_size >= GSM_BUFFER_SIZE)
				break;
		}
	}
	_buf[_buf_size] = 0;
#ifdef ECHO_ENABLED
	if (_buf_size > 0)
		console.write((byte *) _buf, _buf_size);
#endif
	handleCallback();
	return _buf_size;
}

char *SimGSM::find_P(const char *needle) {
	return strstr_P((char *)_buf, needle);
}

int SimGSM::recvUntil_P(const char *s1, const char *s2, const char *s3) {
	const char *ss[3] = { s1, s2, s3 };
	recv();
	for (int i = 0; i < 3; i++)
		if (ss[i] && strstr_P((char *)_buf, ss[i]) != NULL)
			return i + 1;
	return 0;
}

int SimGSM::recvUntil_P(int tries, const char *s1, const char *s2, const char *s3) {
	int ret = 0;
	for (int i = 0; i < tries && ret == 0; i++)
		ret = recvUntil_P(s1, s2, s3);
	return ret;
}

void SimGSM::setTimeout(long first_time, long intra_time) {
	_first_time = first_time;
	_intra_time = intra_time;
}

void SimGSM::loop() {
	if (_serial->available())
		recv();
}

void SimGSM::setCallback_P(int slot, const char *match, callback_func func, void *data) {
	_cb[slot].match = match;
	_cb[slot].length = strlen_P((char *)match);
	_cb[slot].data = data;
	_cb[slot].func = func;
}

void SimGSM::serialMode(int mode) {
	digitalWrite(GSM_PIN, mode == GSM_MODE ? LOW : HIGH);
	digitalWrite(GPS_PIN, mode == GPS_MODE ? LOW : HIGH);
}

boolean SimGSM::isModemReady() {
	boolean ready = false;
	for (int i = 0; i < 2 && !ready; i++)
		ready = sendRecvUntil_P(G("AT"), G("OK"));
	if (ready)
		sendRecv_P(G("ATE0"));
	return ready;
}

boolean SimGSM::isRegistered() {
	return sendRecvUntil_P(G("AT+CREG?"), G("+CREG: 0,1"));
}

boolean SimGSM::isAttached() {
	return sendRecvUntil_P(G("AT+CGATT?"), G("+CGATT: 1"));
}

#define IMEI_LENGTH 14

boolean SimGSM::getIMEI(char *imei) {
	if (!sendRecvUntil_P(G("AT+GSN"), G("OK")))
		return false;
	int len = 0;
	for (size_t i = 0; i < _buf_size && len < IMEI_LENGTH; i++) {
		if ('0' <= _buf[i] && _buf[i] <= '9')
			imei[len++] = _buf[i];
		else if (len > 0)
			break;
	}
	imei[len] = 0;
	return true;
}
