#include "GPSM.h"

#define GSM_PIN 3
#define GPS_PIN 4
#define PWR_PIN 5

#define DBG_RX_PIN 10
#define DBG_TX_PIN 11

GPSM::GPSM() :
	dbg(DBG_RX_PIN, DBG_TX_PIN),
	buf_eol(0),
	_mode(EXT_MODE)
{}

void GPSM::begin()
{
	pinMode(GSM_PIN, OUTPUT);
	pinMode(GPS_PIN, OUTPUT);
	pinMode(PWR_PIN, OUTPUT);
	
	digitalWrite(GSM_PIN, HIGH);
	digitalWrite(GPS_PIN, HIGH);
	digitalWrite(PWR_PIN, LOW);

	Serial.begin(4800);
	dbg.begin(4800);

	gsmPowerUp();
}

// Simcom 548c GSM & GPS serial output are connected to Arduino TX & RX
// via a tri-state buffer. Switching between the two can be done by outputing
// LOW to the one we want to activate (and HIGH to the other).
void GPSM::serialMode(byte mode)
{
	if (_mode != mode)
	{
		_mode = mode;

		// Flush remaining bytes before we change mode
		Serial.flush();
		while (Serial.available())
			Serial.read();

		// Update the tri-state control, use LOW to activate
		digitalWrite(GSM_PIN, mode == GSM_MODE ? LOW : HIGH);
		digitalWrite(GPS_PIN, mode == GPS_MODE ? LOW : HIGH);
	}
}

void GPSM::gsmPowerUp()
{
	dbg.println("Checking GSM power status...");
	boolean ready = isModemReady();
	if (!ready)
	{
		dbg.println("Powering up GSM...");
		digitalWrite(PWR_PIN, HIGH);
		delay(1500);
		digitalWrite(PWR_PIN, LOW);
		delay(1500);
		ready = isModemReady();
	}

	// Disable local echo
	if (ready)
		gsmSendAndRecv("ATE0");

	dbg.println(ready ? "Ready" : "Failed");
}

void GPSM::gsmSend(const char *cmd)
{
	gsmSerial();

	// Cleanup serial buffer
	while (Serial.available())
		dbg.write(Serial.read());

	Serial.write(cmd);
	Serial.write('\r');
	dbg.print(":");
	dbg.println(cmd);
}

byte GPSM::gsmRecv(int start_timeout, int intra_timeout)
{
	gsmSerial();
	byte size = 0;
	unsigned long timeout = millis() + start_timeout;
	while (millis() < timeout && size < GPSM_BUFFER_SIZE)
	{
		if (Serial.available())
		{
			buf[size++] = Serial.read();
			timeout = millis() + intra_timeout;
		}
	}
	if (size < GPSM_BUFFER_SIZE)
		buf[size] = 0;
	if (size > 0)
		dbg.write((byte *) buf, size);
	return size;
}

byte GPSM::gsmSendAndRecv(const char *cmd, int start_timeout, int intra_timeout)
{
	gsmSend(cmd);
	gsmRecv(start_timeout, intra_timeout);
}

char *GPSM::gsmFind(const char *token)
{
	return strstr((char *) buf, token);
}	

boolean GPSM::isModemReady()
{
	char buf[5];
	boolean ready = false;
	for (int i = 0; i < 2 && !ready; i++)
	{
		gsmSendAndRecv("AT");
		ready = gsmFind("OK") != NULL;
	}
	return ready;
}
