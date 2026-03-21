/*
  Productivity Open P1AM Arduino Modbus TCP Client to Click PLC
  This P1AM Example is based on an example found here: https://github.com/AutomationDirect/P1AM-Examples/blob/master/P1AM-100_ModbusTCP_Client_Multiple/P1AM-100_ModbusTCP_Client_Multiple.ino
  This example uses Modbus TCP Client/Server Communication. The P1AM-ETH shield will be the client and sends the input values of a P1-08SIM and P1-04THM to a Click PLC.
  The address is 192.168.1.177 of the Client and 192.168.1.130 is the address of the Server (Click PLC).
*/
#include <Ethernet.h>
#include <P1AM.h>
#include <ArduinoRS485.h>  // ArduinoModbus depends on the ArduinoRS485 library
#include <ArduinoModbus.h>
byte mac[] = { 0x60, 0x52, 0xD0, 0x06, 0xAE, 0xF9 };  // P1AM-ETH have unique MAC IDs on their product label
IPAddress ip(192, 168, 1, 4);                         // IP Address of the P1AM-ETH module.
int HR400002;

EthernetClient clients[2];  // Set up 2 Clients
ModbusTCPClient modbusTCPClient[2] = {
  ModbusTCPClient(clients[0]),
  ModbusTCPClient(clients[1])
};
IPAddress servers[2] = {  //IP Addresses of the Servers
  IPAddress(192, 168, 1, 5),
  IPAddress(0, 0, 0, 0)
};
void setup() {
  modbusTCPClient[0].setTimeout(500);  //Adjust Response Timeout from 30 seconds to 500 ms.
  modbusTCPClient[1].setTimeout(500);  //Adjust Response Timeout from 30 seconds to 500 ms.
  Serial.begin(115200);
  while (!Serial)
    ;  //Wait for Serial port to open. Remove this line if you want it to start automatically
  while (!P1.init())
    ;                       //Wait for module sign-on
  Ethernet.init(5);         //CS pin for P1AM-ETH
  Ethernet.begin(mac, ip);  // start the Ethernet connection
  Serial.println("Productivity Open P1AM Arduino Modbus TCP Client to Click PLC");
  Serial.print("P1AM-ETH at IP:");
  Serial.println(ip);
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {  // Check for Ethernet hardware present
    Serial.println("Ethernet shield is missing. Please try again");
    while (true) {
      delay(1);  // do nothing, no point running without Ethernet hardware
    }
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
  }
}
void loop() {

  int boolpoint = P1.readDiscrete(1, 1);  //reads boolean from slot 1
  if (!modbusTCPClient[0].connected()) {  // client not connected, start the Modbus TCP client
    Serial.print("Attempting to connect to Modbus TCP server at IP:");
    Serial.println(servers[0]);
    if (!modbusTCPClient[0].begin(servers[0])) {
      Serial.println("Modbus TCP Client failed to connect!");
    } else {
      Serial.println("Modbus TCP Client connected");
    }
  }

  if (P1.readDiscrete(1, 1)) {
    Serial.println("ON");
    modbusTCPClient[0].holdingRegisterWrite(0x00, 52);
  } else {
    Serial.println("OFF");
    modbusTCPClient[0].holdingRegisterWrite(0x00, 23);
  }

  for (int i = 1; i <= 8; i++) {            //Run our loop for i = 1 to 8 - Individual input bits
    int boolpoint = P1.readDiscrete(1, i);  //reads boolean from slot 1 for the channel indiciated by our loop variable "i"
    if (!modbusTCPClient[0].coilWrite((0x4000 + (i - 1)), boolpoint)) {
      Serial.print("Failed to write! ");
      Serial.println(modbusTCPClient[0].lastError());
    }
  }
  if (!modbusTCPClient[0].holdingRegisterWrite(0x01, HR400002++)) {
    Serial.print("Failed to write! ");
    Serial.println(modbusTCPClient[0].lastError());
  }

  // Next client communication would be put here.
}