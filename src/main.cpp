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
WiFiClient client1;
WiFiClient client;									//定义一个TCP client对象
WiFiServer server(8086);						//定义一个TCP server对象，并监听端口8086

bool confirmFlag=0;
bool returnFlag=0;
char Reboot_ID1[41]={0xfe,0x21,0x72,0xff,0x00,0x4c,0x00,0x00,0x80,0x3f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xf6,0x00,0x01,0x00,0x00,0x48,0xf0};
char Reboot_ID0[41]={0xfe,0x21,0x45,0xff,0x00,0x4c,0x00,0x00,0x80,0x3f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xf6,0x00,0x00,0x00,0x00,0xd7,0xac};

//function
void wait_ack();
void wait_eraseConfirm_ack();
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
void reset_interrupt() {
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
	Serial1.begin(115200);
#else
	//-- Initialized GPIO02 (Used for "Reset To Factory")
	pinMode(GPIO02, INPUT_PULLUP);
	attachInterrupt(GPIO02, reset_interrupt, FALLING);
#endif
	Logger.begin(2048);
	DEBUG_LOG("\nConfiguring access point...\n");
	DEBUG_LOG("Free Sketch Space: %u\n", ESP.getFreeSketchSpace());

	WiFi.disconnect(true);
	if(Parameters.getWifiMode() == WIFI_MODE_STA) {
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

	if(Parameters.getWifiMode() == WIFI_MODE_AP) {
	//-- Start AP
		WiFi.mode(WIFI_AP);
		WiFi.encryptionType(AUTH_WPA2_PSK);
		WiFi.softAP(Parameters.getWifiSsid(), Parameters.getWifiPassword(), Parameters.getWifiChannel());
		localIP = WiFi.softAPIP();
		Serial1.print("localIP: ");
		Serial1.println(localIP);

		DEBUG_LOG("Waiting for DHCPD...\n");
		dhcp_status dstat = wifi_station_dhcpc_status();				//此处为启动DHCP  服务器，以自动给其网络下的分配 IP   ，在第一次接收到UDP报文，里面
		while (dstat != DHCP_STARTED) {										//就得到IP。不再广播   。位于GCS.cpp
			#ifdef ENABLE_DEBUG
			Serial1.print(".");
			#endif
			delay(500);
			dstat = wifi_station_dhcpc_status();
		}
		wait_for_client();											//等待有WIFI客户端连接至  ESP8266  上
	}

	//-- Boost power to Max
	WiFi.setOutputPower(20.5);

	//-- MDNS														//组播DNS，至于是个什么玩意没太懂
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
	server.begin();											//启动TCP服务器
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

  //liangzelang below//
	bool flag=0;											//等待回复标志位
	bool firstFlag=0;										//每一帧数据第一次读取标志位
  bool Bin_trans_flag=1;							//bin文件传输标志位
	bool Firmware_mode_flag=0;				//固件升级标志

	char temp=0;											//临时变量，用于存放当前读取的字节数
	char tempPos=0;									//临时变量，用于存放数据存储位置
	char length=0;										//临时变量，用于存放当前帧还需读取的字节长度
  const uint8_t dataRequest[2]={0x31,0x01};
  const uint8_t eraseConfirm[2]={0x31,0x02};
  const uint8_t readyConfirm[2]={0x31,0x03};
  const uint8_t finishConfirm[2]={0x31,0x04};
	char ClientData[2]={0};							//用于存放网络接收缓存数据
	char ControlData[2]={0};						//用于存放每一帧数据前两个字节
	char BinDate[256]={0};							//用于存放每一帧数据中实际bin文件数据

  client = server.available();						//检测是否有TCP客户端连接至服务器

	if(client) {
    //如果有TCP客户端连接上，至 固件升级标志位 为1
    Firmware_mode_flag=1;
    Serial1.println("==>>> Connected to the client");
  }
  while(Firmware_mode_flag) {				//进入固件升级模式
  	while(!client.available()) {					//等待已连接上的客户端的数据
		Serial1.print(".");
		delay(500);
  	}
  	while(flag==0) {
			client.readBytes(ClientData, 2);				//读取来自TCP客户端两字节数据
			//Serial1.println(ClientData);						//Serial 1用作调试串口
			client.flush();											//清除网络接收缓冲
			if ((ClientData[0]==0x01)&&(ClientData[1]==0x20)) {				//如果接收的数据为0x01+0x20，执行upload流程
				Serial1.println("==>>> The GCS is ready,Client ask for upload binary");
				flag=1;
				Bin_trans_flag=1;								//使能烧写程序过程标志位
			} else if((ClientData[0]==0x02)&&(ClientData[1]==0x20)) {	//如果接收的数据为0x02+0x20，执行erase流程
				Serial1.println("==>>> The GCS is ready,Client ask for Erase the chip");
				flag=1;
				Bin_trans_flag=0;								//失能烧写程序过程标志位
			} else if((ClientData[0]==0x0c)&&(ClientData[1]==0x03)) {
        //Firmware_mode_flag=0;
        return;
      }
			ClientData[0] = 0;
			ClientData[1] = 0;
  	}
    flag=0;

		while(!Serial.availableForWrite());			//避免出现奇怪的错误，先发送一次同步信号
		Serial.write(0x21);										//发送同步请求信号至飞控：0x21+0x20
		while(!Serial.availableForWrite());
		Serial.write(0x20);

		for(uint8_t i=0;i<41;i++ ) {							//发送Reboot信号至飞控，使其重启进入Bootloader
			while(!Serial.availableForWrite());
			Serial.write(Reboot_ID1[i]);
		}
		delay(300);
		for(uint8_t i=0;i<41;i++ ) {
			while(!Serial.availableForWrite());
			Serial.write(Reboot_ID0[i]);
		}

		delay(1500);											//延时1.5秒，等待飞控重启。此延时不能超过5秒，因为bootloader的延时为5秒
		while(Serial.available()>0)						//通过读取清缓存
		  Serial.read();
		while(Serial.read() >= 0);						//通过读取串口的接受缓冲区来清除缓存区数据
		//Serial.flush();                                 // Attention!!!  this function is to wait for the completion of sending  afer arduino 1.0
																// and especially, prior to arduino 1.0 ,this function instead remove the Rx buffer
		while(!Serial.availableForWrite());
		Serial.write(0x21);									//发送同步请求信号至飞控：0x21+0x20
		while(!Serial.availableForWrite());
		Serial.write(0x20);
		wait_ack();												//等待飞控的回复  0x12+0x10(Insync Ok)  0x12+0x13(Insync Invalid)
		Serial1.println("got the Insync ack");
		delay(1000);

		//...TO DO
		//add the confirm of ERASE function from Vehicle
		client.flush();												//清除网络接收缓存
		//client.write(0x31);										//向Qt端发送芯片擦除请求: 0x31+0x02(可自定义)
		//client.write(0x02);
    client.write(eraseConfirm, sizeof(eraseConfirm));
    //wait_eraseConfirm_ack();              //等待来自Qt端的回复
    wait_wifi_ack();
    if(returnFlag==1) {
      returnFlag=0;
      return;
    }
    if(confirmFlag==1) {
      confirmFlag=0;                     //重置确认擦除标志位
      continue;          //如果Qt端取消擦除，则重新开始本次循环，等来Qt的指令
    }
		//...TO DO

		while(Serial.read() >= 0);							//通过读取数据清除数据缓存区数据
		while(!Serial.availableForWrite());			//等待串口可写
		Serial.write(0x23);										//发送Erase信号至飞控：0x23+0x20
		while(!Serial.availableForWrite());
		Serial.write(0x20);
		wait_ack();													//等待飞控的同步信号回复：0x12+0x10(Insync Ok)  0x12+0x13(Insync Invalid)
		Serial1.println("got the Erase ack");

		//client.write(0x31);										//发送数据至Qt端，提示ESP已经做好接受bin文件数据的准备:0x31+0x03(可自定义)
		//client.write(0x03);
    client.write(readyConfirm,sizeof(readyConfirm));
		wait_wifi_ack();											//等待Qt端回复 （0x12+0x10）
    if(returnFlag==1) {
      returnFlag=0;
      return;
    }
    if(confirmFlag==1) {
      confirmFlag=0;                     //重置确认擦除标志位
      continue;          //如果Qt端取消擦除，则重新开始本次循环，等来Qt的指令
    }
		Serial1.println("got the wifi start ack");
		if(Bin_trans_flag==1) {
			//client.write(0x31);									//向Qt端发送数据请求：0x31+0x01 (可自定义)
			//client.write(0x01);
      client.write(dataRequest,sizeof(dataRequest));
      //client.write(const uint8_t *buf, size_t size)
		}
		while(Bin_trans_flag==1) {						//进入循环接收、烧写bin文件的循环
			while(!client.available());
			if(firstFlag==0) {										//如果是每一帧的第一次读取
				client.readBytes(ControlData, 2);		//先读取前两个字节，第一个字节（0x01：不是最后一帧数据，0x02：最后一帧数据）
																			      //第二个字节（此帧数据的长度，不含头两个字节，一般为252.值得注意的是该数字必须是4的倍数）
        if((ControlData[0]==0x0c)&&(ControlData[1]==0x03)) return;
        if(ControlData[1]%4!=0) {           //判断长度是否为4的倍数，如果不是，则输出提示，同时接收到来自飞控的无效回复（0x12+0x13）
          Serial1.println("the length is not the multiple of 4,fatal error.");
          return;
        }
				temp=client.readBytes(BinDate, ControlData[1]);	//欲读取ControlData[1]长度的数据，并得到实际读取长度temp
				if(temp<ControlData[1]) {					//如果没有读取完这一帧的数据
					length= ControlData[1]-temp;		//计算还需要读取的字节长度length
					tempPos=temp;								//计算下次读取数据存放在BinData数组的位置
					firstFlag=1;										//置标志位
					continue;										//结束本次循环，从头开始等待数据到来
				}
			} else if (firstFlag==1) {							//如果不是每一帧的第一次读取
        Serial1.println("\n Second time Avaiable ");
				temp=client.readBytes(&BinDate[tempPos], length);			//欲读取length长度的数据，实际读取长度为temp
				if(temp<length) {								//如果没有读完
					length=length-temp;						//计算还需读取的字节长度
					tempPos=tempPos+temp;				//计算下次读取数据存放的位置
					firstFlag=1;										//置标志位，此处没有必要
					continue;										//结束本次循环，从头开始等待数据到来
				}
			}
			firstFlag=0;												//重置标志位firstFlag，使下帧数据能够正常接收
			while(Serial.read() >= 0);						//在向飞控发送数据之前，需清除串口接收缓存
			Write_to_Vehicle(BinDate,ControlData[1]);									//发送数据至飞控
			if((ControlData[0]==0x01)&&(client.available()<=1270)) {				//当前帧不是最后一帧，且网络接收缓存小于五帧数据（一帧为252字节，20*252=5080）
				//client.write(0x31);								//向Qt端发送数据请求：0x01+0x21（可自定义）
				//client.write(0x01);
        client.write(dataRequest, sizeof(dataRequest));
        Serial1.println("\n Send Data Request... ");
			} else if(ControlData[0]==0x02)				//当前帧是最后一帧，不再发送数据请求
				Bin_trans_flag=0;								//重置Bin传输标志位，以便此帧传输结束后，跳出该循环
			wait_ack();												//等待飞控的回复：0x12+0x10
		}

		//...TO DO
		//add the confirm of Reboot function
		//..TO DO

		while(Serial.read() >= 0);							//发送命令至飞控之前，清除串口缓冲区
		while(!Serial.availableForWrite());			//等待串口可写
		Serial.write(0x30);										//发送重启命令：0x30+0x20
		while(!Serial.availableForWrite());
		Serial.write(0x20);
		wait_ack();													//等待飞控回复
		Serial1.println("got the Boot ack");

		client.flush();												//向Qt端发送命令之前，清除网络接收缓存
		//client.write(0x31);										//向Qt端发送结束命令：0x31+0x04(可自定义)
		//client.write(0x04);
    client.write(finishConfirm, sizeof(finishConfirm));
		wait_wifi_ack();											//等待Qt端回复
    if(returnFlag==1) {
      returnFlag=0;
      return;
    }
    if(confirmFlag==1) {
      confirmFlag=0;                     //重置确认擦除标志位
      continue;          //如果Qt端取消擦除，则重新开始本次循环，等来Qt的指令
    }
		//delay(5000);												//等待几秒钟，以使Qt端清理相关现场并断开TCP连接，不让ESP8266再次进入固件升级模式（其实不会出现这个问题）
		//Firmware_mode_flag=0;								//重置固件升级标志位，跳出该循环使其工作在WiFi Mavlink Bridge (UDP transmission)模式
	}
}

void wait_eraseConfirm_ack()
{
  char flag_eraseConfirm=0;
	char eraseConfirmData[2]={0};
	while(flag_eraseConfirm==0) {										//等待回复的大循环
		if(client.available()>0) {								//如果网络接收缓存区有数据
			client.readBytes(eraseConfirmData,2);				//以下与串口等待回复相同
			Serial1.println(eraseConfirmData[0]);
			Serial1.println(eraseConfirmData[1]);
			client.flush();
			if((eraseConfirmData[0]==0x12)&&(eraseConfirmData[1]==0x10)) {
				flag_eraseConfirm=1;
				eraseConfirmData[0]=0;
				eraseConfirmData[1]=0;
				delay(100);
			} else if((eraseConfirmData[0]==0x12)&&(eraseConfirmData[1]==0x14)) {
        flag_eraseConfirm=1;
        confirmFlag=1;
				eraseConfirmData[0]=0;
				eraseConfirmData[1]=0;
				delay(100);
      } else {
				flag_eraseConfirm=0;
				eraseConfirmData[0]=0;
				eraseConfirmData[1]=0;
				delay(100);
			}
		} else {														//如果网络接收缓存区没有数据
			Serial1.print(".");
			delay(1000);
		}
	}
	flag_eraseConfirm=0;														//重置标志位，使下次工作正常
}


// This function is to wait for the echo of vehicle
//此函数是等待飞控的回复
void wait_ack()
{
	char flag_vehicle=0;
	char SerialData[2]={0};									//用于存放串口接收缓存数据
	while(flag_vehicle==0) {								//此循环是一直等待回复
		if(Serial.available()>0) {								//如果串口缓存有数据
			Serial.readBytes(SerialData, 2);				//读取两个字节
			Serial1.println(SerialData[0]);				//（调试用）打印出这两个字节
			Serial1.println(SerialData[1]);
			if((SerialData[0]==0x12)&&(SerialData[1]==0x10)) {		//如果两个字节为0x12+0x10
				flag_vehicle=1;									//置标志位flag为1，跳出循环
				SerialData[0]=0;									//清零接收数组
				SerialData[1]=0;
			} else {													//如果两个字节不是0x12+0x10
				flag_vehicle=0;									//清零接收数组，继续循环
				SerialData[0]=0;
				SerialData[1]=0;
			}
		}
	}
	flag_vehicle=0;												//重置循环标志位flag为0，使下次使用正常
}

// This function is to send the bin data
//此函数是向飞控发送规定格式的bin文件数据
//发送格式：0x27+<length>+<Data>+0x20     (length默认为252字节，必须为4的倍数)
void Write_to_Vehicle(char *binfile,uint16_t length)
{
	Serial.write(0x27);
	Serial.write(length);
	for(uint16_t i=0; i<length; i++)
	 Serial.write(binfile[i]);
	Serial.write(0x20);											//按照格式发送数据
	Serial.flush();													//等待数据发送完成
}

// This function is wait for the TCP client echo
//此函数是等待TCP客户端（Qt端）的回复
void wait_wifi_ack()
{
	char flag_wifi=0;
	char FromClientData[2]={0};
	while(flag_wifi==0) {										//等待回复的大循环
		if(client.available()>0) {								//如果网络接收缓存区有数据
			client.readBytes(FromClientData,2);				//以下与串口等待回复相同
			Serial1.println(FromClientData[0]);
			Serial1.println(FromClientData[1]);
			client.flush();
			if((FromClientData[0]==0x12)&&(FromClientData[1]==0x10)) {
				flag_wifi=1;
				FromClientData[0]=0;
				FromClientData[1]=0;
				delay(100);
			} else if((FromClientData[0]==0x12)&&(FromClientData[1]==0x14)) {
				flag_wifi=1;
        confirmFlag=1;      //continue　重新开始固件升级模式，等待Ｑｔ的命令
				FromClientData[0]=0;
				FromClientData[1]=0;
				delay(100);
			} else if((FromClientData[0]==0x0c)&&(FromClientData[1]==0x03)) {
        returnFlag=1;
        flag_wifi=1;       //return   重新开始程序，等待连接
        FromClientData[0]=0;
				FromClientData[1]=0;
				delay(100);
      }
      else {
				flag_wifi=0;
				FromClientData[0]=0;
				FromClientData[1]=0;
				delay(100);
			}
		} else {														//如果网络接收缓存区没有数据
			Serial1.print(".");
			delay(1000);
		}
	}
	flag_wifi=0;														//重置标志位，使下次工作正常
}
