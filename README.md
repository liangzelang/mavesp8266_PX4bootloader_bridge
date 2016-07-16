# MavESP8266
## ESP8266 WiFi Access Point and MavLink Bridge

[![Join the chat at https://gitter.im/dogmaphobic/mavesp8266](https://badges.gitter.im/dogmaphobic/mavesp8266.svg)](https://gitter.im/dogmaphobic/mavesp8266?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

This was developed using a [NodeMCU v2 Dev Kit](http://www.seeedstudio.com/depot/NodeMCU-v2-Lua-based-ESP8266-development-kit-p-2415.html) as it conveniently provides a secondary UART for debugging. It has been tested with the ESP-01 shipped with the [PixRacer](https://pixhawk.org/modules/pixracer) and it is stable at 921600 baud.

The build enviroment is based on [PlatformIO](http://platformio.org). Follow the instructions found here: http://platformio.org/#!/get-started (only tested on Mac OS) for installing it but skip the ```platform init``` step as this has already been done, modified and it is included in this repository. In summary:

```
sudo pip install -U pip setuptools
sudo pip install -U platformio
git clone --recursive https://github.com/dogmaphobic/mavesp8266.git
cd mavesp8266
platformio run
```

When you run ```platformio run``` for the first time, it will download the toolchains and all necessary libraries automatically.

### Useful commands:

* ```platformio run``` - process/build all targets
* ```platformio run -e esp12e``` - process/build just the ESP12e target (the NodeMcu v2)
* ```platformio run -e esp12e -t upload``` - build and upload firmware to embedded board
* ```platformio run -t clean``` - clean project (remove compiled files)

The resulting image(s) can be found in the directory ```.pioenvs``` created during the build process.

### MavLink Submodule

The ```git clone --recursive``` above not only cloned the MavESP8266 repository but it also installed the dependent [MavLink](https://github.com/mavlink/c_library) sub-module. To upated the module (when needed), use the command:

```git submodule update```

### Wiring it up

User level (as well as wiring) instructions can be found here: https://pixhawk.org/peripherals/8266

* Resetting to Defaults: In case you change the parameters and get locked out of the module, all the parameters can be reset by bringing the GPIO02 pin low (Connect GPIO02 pin to GND pin).

### MavLink Protocol

The MavESP8266 handles its own set of parameters and commands. Look at the [PARAMETERS](PARAMETERS.md) page for more information.

### HTTP Protocol

There are some preliminary URLs that can be used for checking the WiFi Bridge status as well as updating firmware and changing parameters. [You can find it here.](HTTP.md)



###  固件升级模式


#### 整体流程：


-->	TCP客户端（Qt端）连接至TCP Server（ESP8266）

-->	TCP客户端（Qt端）向TCP Server发送执行流程（0x01+0x20:upload模式 ，0x02+0x20:erase模式）

-->	TCP Server（ESP8266） 向飞控发送同步信号、擦除信号

-->	(erase模式： TCP Server向飞控发送重启信号，并向TCP发送结束信号)

-->	upload模式：


LOOP Begin： TCP Server（ESP8266）向TCP客户端（Qt端）发送数据请求

	TCP Server（ESP8266）接收到一帧数据

	TCP Server（ESP8266）按照Bootloader定义的数据格式发送至飞控     

LOOP End： TCP Server（ESP8266）检测到最后一帧数据并发送完成


-->     TCP Server(ESP8266)向飞控发送重启信号，并向TCP客户端（Qt端）发送结束信号


####  通信信号定义：


+ 1、TCP Server(ESP8266)与TCP客户端（Qt端）通信信号定义(C代表TCP客户端，S代表TCP服务器)

  Qt-->ESP8266:

	模式（流程）选择信号：  0x31+0x08（upload模式）   
	　　　　　　　　　　　　0x31+0x09(erase模式)

	回复信号:            0x31+0x10 (OK:程序继续)  
　　　　　　　　　　　　　0x31+0x11(invalid：调回至固件升级模式开始)
　　　　　　　　　　　　　0x31+0x12(结束固件升级模式)

	数据发送信号:          0x01+length+Data(非结束帧)  
	　　　　　　　　　　　　 0x02+length+Data(结束帧)

  ESP8266-->Qt:

   数据请求信号:          0x31+0x01

   擦除确认信号:          0x31+0x02

   提示信号:             0x31+0x03

   结束信号:             0x31+0x04


+ 2、TCP Server（ESP8266）与飞控通信信号定义（S代表ESP8266，V代表飞控）

  ESP8266-->飞控

   同步信号：                      0x21+0x20

   擦除信号：                      0x23+0x20

   烧写信号：                      0x27+length+Data+0x20

   重启信号：                      0x30+0x20

  飞控-->ESP8266

   回复信号：                      0x12+0x10(OK)  0x12+0x13(invalid)


注：1中的信号为自定义，2中的信号为Bootloader中定义的


烧写细节：

    每次烧写252个字节，该字节长度需在0-255之间，且必须为4的倍数。
