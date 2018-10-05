#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>

#define SerialTxControl 9   //RS485 управляющий контакт на arduino pin 10
#define RS485Transmit    HIGH
#define RS485Receive     LOW

// Update these with values suitable for your network.
byte mac[]    = {  0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xED };
IPAddress ip(172, 20, 40, 37);
IPAddress server(172, 20, 40, 40);

typedef struct Message
{
	byte DeviceID;		// ID устройства, может принимать значения 1-254
	byte DestinationID;	// Номер устройства-получателя, может принимать значения 0-254 (0 - броадкаст)
	uint8_t PacketID;	// Идентификатор пакета.
	byte ActuatorID;	// ID исполнительного устройства, может принимать значения 1-254
	byte CommandID;		// Название параметра (feedTemperature, returnTemperature, relayState...)
	int DataValue;	// Значение параметра типа int
};

Message dataMessage[4];

// Callback function header
void callback(char* topic, byte* payload, unsigned int length);
void reconnect();

EthernetClient ethClient;
PubSubClient client(server, 1883, callback, ethClient);

void setup(void){
  uint8_t i=0;
  while(i<=4){
	  dataMessage[i].DeviceID = 0;
	  dataMessage[i].DestinationID = 0;
	  dataMessage[i].PacketID = 0;
	  dataMessage[i].ActuatorID = 0;
	  dataMessage[i].CommandID = 0;
	  dataMessage[i].DataValue = 0;
	  i++;
  }
  Serial.begin(38400);

  Serial1.begin(38400);
  pinMode(SerialTxControl, OUTPUT);
  digitalWrite(SerialTxControl, RS485Transmit);
  Serial1.print("Heating controller ready");
  delay(10);
  digitalWrite(SerialTxControl, RS485Receive);

  Ethernet.begin(mac, ip);
  if (client.connect("arduinoClient")) {
    client.publish("outTopic","hello world");
    client.subscribe("inTopic");
  }
}
 
void loop(void){
  char dataTempChar[5];
  if (!client.connect("RelayClient")) {   // Если DHTClient клиент НЕ подключен...
    reconnect();                        // Запускаем функцию переподключения
  }
  digitalWrite(SerialTxControl, RS485Receive);  // читаем данные с порта
  int i=0;
  if(Serial1.available()){
	  while( Serial1.available() ){
		  Serial1.readBytes((uint8_t*)&dataMessage, sizeof(dataMessage));
	  }
	  if( ( 254 == dataMessage[0].DestinationID ) && ( 1 == dataMessage[0].DeviceID ) && ( 1 == dataMessage[0].ActuatorID )){
			dtostrf(dataMessage[0].DataValue/100.00, 5, 2, dataTempChar);
			Serial.println("");
			Serial.print("feedTemp = ");
			Serial.println(dataMessage[0].DataValue/100.00);
			client.publish("/myhome/heating/tfeed", dataTempChar);
		}
	  if( ( 254 == dataMessage[1].DestinationID ) && ( 1 == dataMessage[1].DeviceID ) && ( 2 == dataMessage[1].ActuatorID )){
			dtostrf(dataMessage[1].DataValue/100.00, 5, 2, dataTempChar);
			Serial.println("");
			Serial.print("returnTemp = ");
			Serial.println(dataMessage[1].DataValue/100.00);
			client.publish("/myhome/heating/treturn", dataTempChar);
		}
	  // Выводим что приняли с других устройств
	  Serial.println("######################################");
	  uint8_t i=0;
	  while(i<=4){
		  Serial.print("DeviceID=");
		  Serial.println(dataMessage[i].DeviceID);
		  Serial.print("DestinationID=");
		  Serial.println(dataMessage[i].DestinationID);
		  Serial.print("PacketID=");
		  Serial.println(dataMessage[i].PacketID);
		  Serial.print("ActuatorID=");
		  Serial.println(dataMessage[i].ActuatorID);
		  Serial.print("CommandID=");
		  Serial.println(dataMessage[i].CommandID);
		  Serial.print("DataValue=");
		  Serial.println(dataMessage[i].DataValue/100.00);
		  Serial.println("");
		  i++;
	  }
	  Serial.println("######################################");
  }
  client.loop();
}

// Callback function
void callback(char* topic, byte* payload, unsigned int length) {
  // In order to republish this payload, a copy must be made
  // as the orignal payload buffer will be overwritten whilst
  // constructing the PUBLISH packet.

  // Allocate the correct amount of memory for the payload copy
  byte* p = (byte*)malloc(length);
  // Copy the payload to the new buffer
  memcpy(p,payload,length);
  client.publish("outTopic", p, length);
  // Free the memory
  free(p);
}

// Функция для переподключения соединения с Брокером
void reconnect() {
  // Повторяем, пока не переподключимся...
  while (!client.connected()) {         // Логическое НЕ "!" - Проверяем, если клиент не Законнектен....
    // Попытка подключиться
    if (client.connect("RelayClient")) {  // Если DHTClient клиент подключен...
      Serial.println(F("Connected!"));  // Выводим сообщ., что подключено!
    } else {
      Serial.print(F("Failed connected - ")); // Ощибка соединения
      Serial.println(F("Jdem 5 seconds"));    // Ждём 5 секунд
      delay(5000);
    }
  }
}
		
