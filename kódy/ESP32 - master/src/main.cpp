#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// FreeRTOS Semaphore pro zamezeni konfliktu při přistupování triggrovacích proměnných
SemaphoreHandle_t trigger_mutex = xSemaphoreCreateMutex();
SemaphoreHandle_t lamp_mutex = xSemaphoreCreateMutex();
SemaphoreHandle_t mqtt_mutex = xSemaphoreCreateMutex();
SemaphoreHandle_t i2c_mutex = xSemaphoreCreateMutex();

Adafruit_BME280 bme;
bool IoT = false;

// init globalnich promennych
uint8_t triggerPos[2] = {0, 0};
uint8_t triggerCount = 0;
uint8_t lampCount = 0;
unsigned long triggerNum = 0;

uint8_t low = 5;
uint8_t high = 255;
uint16_t interval = 1000;
uint8_t radius = 24;

typedef struct lampStruct
{
	uint8_t I2C;
	uint8_t X;
	uint8_t Y;
} lampStruct;

#include <lamp.h>
#include <mqtt.h>

//lampMQTT
void lampMQTT(void *parameters)
{
	lampStruct *lampTemp = (lampStruct *)parameters;
	lampStruct lampParam;
	lampParam.I2C = lampTemp->I2C;
	lampParam.X = lampTemp->X;
	lampParam.Y = lampTemp->Y;
	vTaskPrioritySet(NULL, 3);
	while (true)
	{
		if (client.connected())
		{
			mqttPublish("Křemíkové zátiší/Shockleyův park/lamp/" + String(lampParam.I2C) + "/X", String(lampParam.X));
			mqttPublish("Křemíkové zátiší/Shockleyův park/lamp/" + String(lampParam.I2C) + "/Y", String(lampParam.Y));
			break;
		}
		taskYIELD();
	}
	taskYIELD();
	vTaskDelete(NULL);
}

//Task pro ošéfení poloměru
void lampTrigger(void *parameters)
{
	//inicializace
	lampStruct *lampTemp = (lampStruct *)parameters;
	lampStruct lampParam;
	lampParam.I2C = lampTemp->I2C;
	lampParam.X = lampTemp->X;
	lampParam.Y = lampTemp->Y;
	vTaskPrioritySet(NULL, 3);

	unsigned long onMillis = 0;
	unsigned long lastTrigger = 0;
	bool on = false;
	double rad;

	while (true)
	{
		if (xSemaphoreTake(trigger_mutex, 20) == pdTRUE) //požádá o semafor
		{
			if (triggerNum != lastTrigger) //pokud je zaznamenán nový trigger
			{
				rad = sqrt(pow(lampParam.X - triggerPos[0], 2) + pow(lampParam.Y - triggerPos[1], 2)); //výpočet poloměru
				lastTrigger = triggerNum;
				triggerCount--; //dekrementace

				if (triggerCount < 1) //pokud trigger zpracovali všechny tasky uvolníme semafor pro čtení lamp
				{
					xSemaphoreGive(lamp_mutex);
				}
				xSemaphoreGive(trigger_mutex); //uvolníme semafor pro poloměr

				if (rad <= radius) //pokud jsme v poloměru
				{
					if (!on) //pokud je lampa zhasnutá
					{
						writePWM(lampParam.I2C, high); //zapneme lampu
						on = true;
						mqttPublish("Křemíkové zátiší/Shockleyův park/lamp/" + String(lampParam.I2C) + "/light", String(true));
					}
					onMillis = millis(); //nastavíme čas pro výpočet intervalu
				}
				else if (lampParam.I2C > 39 && on)
				{
					writePWM(lampParam.I2C, 10);
					on = false;
					mqttPublish("Křemíkové zátiší/Shockleyův park/lamp/" + String(lampParam.I2C) + "/light", String(false));
				}
			}
			else
			{
				xSemaphoreGive(trigger_mutex); //vrátíme semafor pro poloměr
			}
		}

		//pokud je lampa rozsvícena a zároveň jsme mimo interval
		if (on && (millis() - onMillis) > interval && lampParam.I2C < 40)
		{
			writePWM(lampParam.I2C, low); //vypneme lampu
			on = false;
			mqttPublish("Křemíkové zátiší/Shockleyův park/lamp/" + String(lampParam.I2C) + "/light", String(false));
		}
		taskYIELD();
	}
}

// task kazde lampy
void lamp(void *parameters)
{
	//inicializace
	lampStruct *lampTemp = (lampStruct *)parameters;
	lampStruct lampParam;
	lampParam.I2C = lampTemp->I2C;
	lampParam.X = lampTemp->X;
	lampParam.Y = lampTemp->Y;
	vTaskPrioritySet(NULL, 3);
	uint8_t lastValue = 0;
	uint8_t value = 0;

	// smycka tasku lampy 💡
	while (true)
	{
		if (xSemaphoreTake(lamp_mutex, 20) == pdTRUE) //požádáme o semafor pro čtení lamp
		{
			value = readTouch(lampParam.I2C);
			if (value == 1) //pokud máme dotyk
			{
				//zapíšeme souřadnice triggeru
				triggerPos[0] = lampParam.X;
				triggerPos[1] = lampParam.Y;
				triggerCount = lampCount;
				triggerNum++;
				if (lastValue != value)
				{
					mqttPublish("Křemíkové zátiší/Shockleyův park/lamp/" + String(lampParam.I2C) + "/sensor", String(true));
				}
				delay(100); //tato delay zde nemusí nutně být, ale malinko odlehčí sběrnici :)
			}
			else
			{
				//pokud není dotyk vrátíme semafor, jinak ho nevracíme!!!
				xSemaphoreGive(lamp_mutex);
				if (lastValue != value)
				{
					mqttPublish("Křemíkové zátiší/Shockleyův park/lamp/" + String(lampParam.I2C) + "/sensor", String(false));
				}
			}
			lastValue = value;
		}
		delay(25);
	}
	vTaskDelete(NULL);
}

// task kazde lampy
void lampBlink(void *parameters)
{
	//inicializace
	lampStruct *lampTemp = (lampStruct *)parameters;
	lampStruct lampParam;
	lampParam.I2C = lampTemp->I2C;
	lampParam.X = lampTemp->X;
	lampParam.Y = lampTemp->Y;
	vTaskPrioritySet(NULL, 3);

	//3x zablikání lampy
	for (int i = 0; i < 3; i++)
	{
		writePWM(lampParam.I2C, high);
		delay(1000);
		writePWM(lampParam.I2C, low);
		delay(1000);
	}

	vTaskDelete(NULL);
}

//nastavení lampy
void lampInit(void *parameters)
{
	lampStruct lampParam;
	lampParam.I2C = int(parameters);

	vTaskPrioritySet(NULL, 3);

	uint8_t *p = readPosition(lampParam.I2C);
	lampParam.X = p[0];
	lampParam.Y = p[1];

	//inkrementace počtu lamp
	while (true)
	{
		if (xSemaphoreTake(trigger_mutex, 20) == pdTRUE)
		{
			lampCount++;
			xSemaphoreGive(trigger_mutex);
			break;
		}
	}

	//nastavení módu lampy
	writeMode(lampParam.I2C, 1);
	if (true)
	{
		if (lampParam.I2C < 40)
		{
			writeThreshold(lampParam.I2C, 150);
		}
		else
		{
			writeThreshold(lampParam.I2C, 20);
		}
		writeSpeed(lampParam.I2C, 5);
		autonomusHigh(lampParam.I2C, high);
		autonomusLow(lampParam.I2C, low);
		autonomusInterval(lampParam.I2C, 5000);
		writeFade(lampParam.I2C, true);
	}

	//3x zablikání lampy
	xTaskCreatePinnedToCore(lampBlink, "lampBlink", 1000, (void *)&lampParam, 10, NULL, 1);

	//vytvoření tasků nutných pro správné pracování
	xTaskCreatePinnedToCore(lamp, "lamp", 2000, (void *)&lampParam, 10, NULL, 1);
	xTaskCreatePinnedToCore(lampTrigger, "lampTrigger", 2000, (void *)&lampParam, 10, NULL, 1);
	if (IoT)
	{
		xTaskCreatePinnedToCore(lampMQTT, "lampMQTT", 2000, (void *)&lampParam, 10, NULL, 1);
	}
	vTaskDelete(NULL);
}

// vyhledani lampy na sbernici
void scanner(void *parameters)
{
	for (int i = 4; i < 50; i++)
	{
		if (isLampHere(i))
		{
			//pokud jsme našli lampu vytvoříme pro ni task
			xTaskCreatePinnedToCore(lampInit, "lampInit", 1500, (void *)i, 10, NULL, 1);
			taskYIELD();
		}
		delay(1);
	}
	vTaskDelete(NULL);
}

void sensors(void *parameters)
{
	float temp, press, hum;
	while (true)
	{
		if (xSemaphoreTake(i2c_mutex, 100) == pdTRUE)
		{
			temp = bme.readTemperature();
			press = bme.readPressure();
			hum = bme.readHumidity();
			xSemaphoreGive(i2c_mutex);
			mqttPublish("Křemíkové zátiší/Shockleyův park/temperature", String(temp));
			mqttPublish("Křemíkové zátiší/Shockleyův park/pressure", String(press));
			mqttPublish("Křemíkové zátiší/Shockleyův park/humidity", String(hum));
			delay(2000);
		}
	}
}

void taskIoT(void *parameters)
{
	IoT = true;
	delay(2000);
	WiFi.begin("ThinkSpot", "0123456789");
	while (WiFi.status() != WL_CONNECTED)
	{
		delay(500);
	}
	xTaskCreatePinnedToCore(mqtt, "MQTT", 5000, (void *)1, 3, NULL, 1);
	xTaskCreatePinnedToCore(sensors, "sensor", 2000, (void *)1, 3, NULL, 1);
	vTaskDelete(NULL);
}

void setup()
{
	//inicializace ESP
	delay(500); //zpoždění pro minimalizaci vlivu přechodových jevů

	Wire.begin(22, 23);
	bme.begin(0x76);
	pinMode(22, INPUT);
	pinMode(23, INPUT);

	xTaskCreatePinnedToCore(taskIoT, "IoT", 5000, (void *)1, 3, NULL, 1);
	xTaskCreatePinnedToCore(scanner, "scanner", 2000, (void *)1, 5, NULL, 1);
	vTaskDelete(NULL);
}

void loop()
{
	vTaskDelete(NULL);
}