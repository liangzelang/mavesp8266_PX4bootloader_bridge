/****************************************************************************
 *
 * Copyright (c) 2015, 2016 Gus Grubba. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file main.cpp
 * ESP8266 Wifi AP, MavLink UART/UDP Bridge
 *
 * @author Gus Grubba <mavlink@grubba.com>
 * ESP8266 WiFi AP, TCP Brdg
 * @author Liangzelag <liangzelang@gmail.com>
 */

#include "mavesp8266.h"
#include "mavesp8266_parameters.h"
#include "mavesp8266_gcs.h"
#include "mavesp8266_vehicle.h"
#include "mavesp8266_httpd.h"
#include "mavesp8266_component.h"

#include <ESP8266mDNS.h>

#define GPIO02  2
WiFiClient client;
WiFiServer server(8086);

char Reboot_ID1[41]={0xfe,0x21,0x72,0xff,0x00,0x4c,0x00,0x00,0x80,0x3f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xf6,0x00,0x01,0x00,0x00,0x48,0xf0};
char Reboot_ID0[41]={0xfe,0x21,0x45,0xff,0x00,0x4c,0x00,0x00,0x80,0x3f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xf6,0x00,0x00,0x00,0x00,0xd7,0xac};
char ClientData[2]={0};
char SerialData[2]={0};
char ControlData[2]={0};
char BinDate[256]={0};
char flag=0;
char Bin_trans_flag=1;
char Firmware_mode_flag=0;
//function
void wait_ack();
void wait_wifi_ack();
void Write_to_Vehicle(char *binfile,uint16_t length);

//---------------------------------------------------------------------------------
//-- HTTP Update Status
class MavESP8266UpdateImp : public MavESP8266Update {
public:
    MavESP8266UpdateImp ()
        : _isUpdating(false)
    {

    }
    void updateStarted  ()
    {
        _isUpdating = true;
    }
    void updateCompleted()
    {
        //-- TODO
    }
    void updateError    ()
    {
        //-- TODO
    }
    bool isUpdating     () { return _isUpdating; }
private:
    bool _isUpdating;
};



//-- Singletons
IPAddress               subnet;
IPAddress               localIP;
MavESP8266Component     Component;
MavESP8266Parameters    Parameters;
MavESP8266GCS           GCS;
MavESP8266Vehicle       Vehicle;
MavESP8266Httpd         updateServer;
MavESP8266UpdateImp     updateStatus;
MavESP8266Log           Logger;

//---------------------------------------------------------------------------------
//-- Accessors
class MavESP8266WorldImp : public MavESP8266World {
public:
    MavESP8266Parameters*   getParameters   () { return &Parameters;    }
    MavESP8266Component*    getComponent    () { return &Component;     }
    MavESP8266Vehicle*      getVehicle      () { return &Vehicle;       }
    MavESP8266GCS*          getGCS          () { return &GCS;           }
    MavESP8266Log*          getLogger       () { return &Logger;        }
};

MavESP8266WorldImp      World;

MavESP8266World* getWorld()
{
    return &World;
}

//---------------------------------------------------------------------------------
//-- Wait for a DHCPD client
void wait_for_client() {
    DEBUG_LOG("Waiting for a client...\n");
    int wcount = 0;
    uint8 client_count = wifi_softap_get_station_num();
    while (!client_count) {
        #ifdef ENABLE_DEBUG
        Serial1.print(".");
        if(++wcount > 80) {
            wcount = 0;
            Serial1.println();
        }
        #endif
        delay(1000);
        client_count = wifi_softap_get_station_num();
    }
    DEBUG_LOG("Got %d client(s)\n", client_count);
}

//---------------------------------------------------------------------------------
//-- Reset all parameters whenever the reset gpio pin is active
void reset_interrupt(){
    Parameters.resetToDefaults();
    Parameters.saveAllToEeprom();
    ESP.reset();
}

//---------------------------------------------------------------------------------
//-- Set things up
void setup() {
    delay(1000);
    Parameters.begin();
#ifdef ENABLE_DEBUG
    //   We only use it for non debug because GPIO02 is used as a serial
    //   pin (TX) when debugging.
    Serial1.begin(921600);
#else
    //-- Initialized GPIO02 (Used for "Reset To Factory")
    pinMode(GPIO02, INPUT_PULLUP);
    attachInterrupt(GPIO02, reset_interrupt, FALLING);
#endif
    Logger.begin(2048);
    DEBUG_LOG("\nConfiguring access point...\n");
    DEBUG_LOG("Free Sketch Space: %u\n", ESP.getFreeSketchSpace());

    WiFi.disconnect(true);

    if(Parameters.getWifiMode() == WIFI_MODE_STA){
        //-- Connect to an existing network
        WiFi.mode(WIFI_STA);
        WiFi.begin(Parameters.getWifiStaSsid(), Parameters.getWifiStaPassword());
        //-- Wait a minute to connect
        for(int i = 0; i < 120 && WiFi.status() != WL_CONNECTED; i++) {
            #ifdef ENABLE_DEBUG
            Serial.print(".");
            #endif
            delay(500);
        }
        if(WiFi.status() == WL_CONNECTED) {
            localIP = WiFi.localIP();
            WiFi.setAutoReconnect(true);
        } else {
            //-- Fall back to AP mode if no connection could be established
            WiFi.disconnect(true);
            Parameters.setWifiMode(WIFI_MODE_AP);
        }
    }

    if(Parameters.getWifiMode() == WIFI_MODE_AP){
        //-- Start AP
        WiFi.mode(WIFI_AP);
        WiFi.encryptionType(AUTH_WPA2_PSK);
        WiFi.softAP(Parameters.getWifiSsid(), Parameters.getWifiPassword(), Parameters.getWifiChannel());
        localIP = WiFi.softAPIP();
        Serial1.print("localIP: ");
        Serial1.println(localIP);
        subnet = WiFi.subnetMask();
        Serial1.print("NETMASK: ");
        Serial1.println(subnet);
        DEBUG_LOG("Waiting for DHCPD...\n");
        dhcp_status dstat = wifi_station_dhcpc_status();    //此处为启动DHCP  服务器，以自动给其网络下的分配 IP   ，在第一次接收到UDP报文，里面
        while (dstat != DHCP_STARTED) {                     //就得到IP。不再广播   。位于GCS.cpp
            #ifdef ENABLE_DEBUG
            Serial1.print(".");
            #endif
            delay(500);
            dstat = wifi_station_dhcpc_status();
        }
        wait_for_client();                                   //等待有WIFI客户端连接至  ESP8266  上
    }

    //-- Boost power to Max
    WiFi.setOutputPower(20.5);


    //-- MDNS                                               //组播DNS，至于是个什么玩意没太懂
    char mdsnName[256];
    sprintf(mdsnName, "MavEsp8266-%d",localIP[3]);
    MDNS.begin(mdsnName);
    MDNS.addService("http", "tcp", 80);
    //-- Initialize Comm Links
    DEBUG_LOG("Start WiFi Bridge\n");
    Parameters.setLocalIPAddress(localIP);
    IPAddress gcs_ip(localIP);
    //-- I'm getting bogus IP from the DHCP server. Broadcasting for now.
    gcs_ip[3] = 255;
    GCS.begin((MavESP8266Bridge*)&Vehicle, gcs_ip);
    Serial1.println(gcs_ip);
    Vehicle.begin((MavESP8266Bridge*)&GCS);
    //-- Initialize Update Server
    updateServer.begin(&updateStatus);
    server.begin();                                       //begin the TCP server
}

//---------------------------------------------------------------------------------
//-- Main Loop
void loop() {
    if(!updateStatus.isUpdating()) {
        GCS.readMessage();
        delay(0);
        Vehicle.readMessage();

    }
    updateServer.checkUpdates();
    client = server.available();                        //check whether there is one client connecting to the server or not
    if(client)                                          //if there is a client ,got it
    {
      Firmware_mode_flag=1;
    }
    while(Firmware_mode_flag)                           //Enter the loop of TCP mode
    {
      while(!client.available())
      {
        Serial1.print(".");
        delay(500);
      }
      while(flag==0)
       {
         client.readBytes(ClientData, 2);
         Serial1.print(ClientData);                     //Serial1 is just a debug monitor
         client.flush();                                //clean the cache buffer
         if ((ClientData[0]==0x01)&&(ClientData[1]==0x20))
         {                                             //If got the (0x01+0x20) from client
           Serial1.println("The GCS is ready");
           flag=1;
         }
         ClientData[0] = 0;
         ClientData[1] = 0;
       }
       flag=0;

       while(!Serial.availableForWrite());              //Send the Insync signal to Vehicle 0x21+0x20
       Serial.write(0x21);                              //avoid some Bizarre
       while(!Serial.availableForWrite());
       Serial.write(0x20);

       for(uint8_t i=0;i<41;i++ )
       {
         while(!Serial.availableForWrite());              //Send the Reboot signal to Vehicle
         Serial.write(Reboot_ID1[i]);
       }
       delay(300);
       for(uint8_t i=0;i<41;i++ )
       {
         while(!Serial.availableForWrite());
         Serial.write(Reboot_ID0[i]);
       }
       //Serial.flush();                                   //Waits for the transmission of outgoing serial data to complete. (Prior to Arduino 1.0, this instead removed any buffered incoming serial data.)
       delay(1500);                                      //Wait 1.5s for the reset of Vehicl
       while(Serial.available()>0)
            Serial.read();
       while(Serial.read() >= 0);                        //Clean the Rx buffer by reading the buffer zone
       //Serial.flush();                                 // Attention!!!  this function is to wait for the completion of sending  afer arduino 1.0
                                                         // and especially, prior to arduino 1.0 ,this function instead remove the Rx buffer
       while(!Serial.availableForWrite());
       Serial.write(0x21);                               //Send the Insync signal to Vehicle 0x21+0x20
       while(!Serial.availableForWrite());
       Serial.write(0x20);
       wait_ack();                                      //Wait for the Insync echo from Vehicle   0x12+0x10(Insync Ok)  0x12+0x13(Insync Invalid)
       Serial1.println("got the Insync ack");
       delay(1000);

       //...TO DO
       //add the confirm of ERASE function from Vehicle
       client.flush();
       client.write(0x01);                             //ASK the GCS ,whether erase the
       client.write(0x20);
       wait_wifi_ack();
       //...TO DO

       while(Serial.read() >= 0);                       //Waits for the transmission of outgoing serial data to complete.
       while(!Serial.availableForWrite());              //Send the Erase signal to Vehicle 0x23+0x20
       Serial.write(0x23);
       while(!Serial.availableForWrite());
       Serial.write(0x20);
       wait_ack();                                     //Wait for the Insync echo from Vehicle   0x12+0x10(Insync Ok)  0x12+0x13(Insync Invalid)
       Serial1.println("got the Erase ack");

       client.write(0x01);                             //Tell the GCS ,ESP is ready to receive bin data and send it to vehicle
       client.write(0x20);
       wait_wifi_ack();
       Serial1.println("got the wifi start ack");

       client.write(0x01);                             //Tell the GCS ,send  your data .  0x01+0x21 (this is customized)
       client.write(0x21);
       while(Bin_trans_flag==1)                        //Entry the lopp of sending bin file
       {
         while(!client.available());

         client.readBytes(ControlData, 2);             //The first byte is status of trans 0x01 :continual   0x02 : last one
                                                       //The second byte is the length of data this time  ,the range:0-255 ,and especially ,must be A multiple of 4
         client.readBytes(BinDate, ControlData[1]);    //Then read the bin data
        client.flush();                               //Clean the client buffer in order to receive new data next time
         while(Serial.read() >= 0);                    //Before send the data to Vehicle ,clean the Rx buffer
         //Serial1.print(ControlData[0]);
        //Serial1.print(ControlData[1]);
         //Serial1.print(BinDate[0]);
         Write_to_Vehicle(BinDate,ControlData[1]);     //Send the data to Vehicle
         wait_ack();

        if(ControlData[0]==0x01)
         {
           client.write(0x01);                        //tell the GCS ,send  your data  .
           client.write(0x21);                        //0x01  0x21  (customized)
         }
         else if(ControlData[0]==0x02)
         {
           Bin_trans_flag=0;                           //If the first byte equal 0x02  ,after this trans jump out.
         }
       }


       //...TO DO
       //add the confirm of Reboot function
       //..TO DO


       while(Serial.read() >= 0);                    //Before send the command to Vehicle ,clean the Rx buffer
       while(!Serial.availableForWrite());           //Send the Boot signal to the Vehicle  0x30+0x20
       Serial.write(0x30);
       while(!Serial.availableForWrite());
       Serial.write(0x20);
       wait_ack();                                   //Wait for the Boot echo from Vehicle
       Serial1.println("got the Boot ack");

       client.flush();
       client.write(0x02);                           //tell the GCS the trans finish, turn the mode of GCS
       client.write(0x20);
       wait_wifi_ack();
       delay(5000);                                 //Wait for serval seconds to escape the current TCP link , then the sketch won't jump into the TCP transmission
       Firmware_mode_flag=0;                        //Jump out the loop of TCP transmission and to be the WiFi Mavlink Bridge (UDP transmission)
    }
}

// This function is to wait for the echo of vehicle

void wait_ack(){
  while(flag==0){
    if(Serial.available()>0){
      Serial.readBytes(SerialData, 2);
      //Serial1.println(Serial.available());
      Serial1.println(SerialData[0]);
      Serial1.println(SerialData[1]);
        if((SerialData[0]==0x12)&&(SerialData[1]==0x10))
        {
            flag=1;
            SerialData[0]=0;
            SerialData[1]=0;
            //delay(100);
        }
        else
        {
          flag=0;
          SerialData[0]=0;
          SerialData[1]=0;
          //delay(100);
        }
    }
  //  else
  //  {
  //    Serial1.print(".");
      //delay(1000);
  //  }
  }
  flag=0;
}

// This function is write the flash

void Write_to_Vehicle(char *binfile,uint16_t length)
{
  Serial.write(0x27);
  Serial.write(length);
  for(uint16_t i=0; i<length; i++)
  Serial.write(binfile[i]);
  Serial.write(0x20);
  Serial.flush();
}

// This function is wait for the TCP client echo

void wait_wifi_ack()
{
  while(flag==0){
    if(client.available()>0){
      client.readBytes(ClientData, 2);
      Serial1.println(ClientData[0]);
      Serial1.println(ClientData[1]);
      client.flush();
        if((ClientData[0]==0x12)&&(ClientData[1]==0x10))
        {
            flag=1;
            ClientData[0]=0;
            ClientData[1]=0;
            delay(100);
        }
        else
        {
          flag=0;
          SerialData[0]=0;
          SerialData[1]=0;
          delay(100);
        }
    }
    else
    {
      Serial1.print(".");
      delay(1000);
    }
  }
  flag=0;
}
