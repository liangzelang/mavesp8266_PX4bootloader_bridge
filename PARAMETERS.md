# MavESP8266
## Mavlink Paramaters and Messages supported by the MavESP8266 Firmware

### Parameters

#### MAVLINK_MSG_ID_PARAM_REQUEST_LIST

If this message is sent to *All Components* (```MAV_COMP_ID_ALL```), or specifically to the MavESP8266 component ID, it will return its list of parameters. Messages sent to ```MAV_COMP_ID_ALL``` will be forwarded to the UAS as well.

#### MAVLINK_MSG_ID_PARAM_REQUEST_READ

If this message is sent to *All Components*, or specifically to the MavESP8266 component ID, it will return the requested parameter (either by ID or by Index)

#### MAVLINK_MSG_ID_PARAM_SET

If this message is sent to *All Components*, or specifically to the MavESP8266 component ID, it will set the new value for the specified parameter. Note that this only sets the value for the current session. It does not write the values to EEPROM.

### Available Parameters

| Parameter ID  | Parameter Type | Description |
| ------------- | -------------- | ----------- |
| SW_VER  | MAV_PARAM_TYPE_UINT32 | Firmware Version (Read Only) |
| DEBUG_ENABLED | MAV_PARAM_TYPE_INT8  | Enable Debug Messages (1) |
| COMP_ID | MAV_PARAM_TYPE_UINT8 | Component ID (2) |
| WIFI_CHANNEL  | MAV_PARAM_TYPE_UINT32 | AP WiFi Channel (default to 11) |
| WIFI_UDP_HPORT | MAV_PARAM_TYPE_UINT16 | GCS UDP Port (default to 14550) |
| WIFI_UDP_CPORT | MAV_PARAM_TYPE_UINT16 | Local UDP Port (default to 14555)  |
| WIFI_SSID1 | MAV_PARAM_TYPE_UINT32 | WiFi AP SSID (3) |
| WIFI_SSID2 | MAV_PARAM_TYPE_UINT32 | |
| WIFI_SSID3 | MAV_PARAM_TYPE_UINT32 | |
| WIFI_SSID4 | MAV_PARAM_TYPE_UINT32 | |
| WIFI_PASSWORD1 | MAV_PARAM_TYPE_UINT32 | WiFi AP Password (3) |
| WIFI_PASSWORD2 | MAV_PARAM_TYPE_UINT32 | |
| WIFI_PASSWORD3 | MAV_PARAM_TYPE_UINT32 | |
| WIFI_PASSWORD4 | MAV_PARAM_TYPE_UINT32 | |
| UART_BAUDRATE | MAV_PARAM_TYPE_UINT32 | UAS UART Link Baud Rate (default to 921600) |

#### Notes

* (1) If debug is enabled, debug messages are sent using ```MAVLINK_MSG_ID_STATUSTEXT``` with a proper ```MAV_SEVERITY_DEBUG``` type. Other messages types, ```MAV_SEVERITY_NOTICE``` for example, are sent regardless.
* (2) The default component ID used by MavESP8266 is ```MAV_COMP_ID_UDP_BRIDGE``` (240).
* (3) MavLink parameter messages only support a 32-Bit parameter (be it a float, an uint32_t, etc.) In other to fit a 16-character SSID and a 16-character Password, 4 paramaters are used for each. The 32-Bit storage is used to contain 4 bytes for the string.

### MAVLINK_MSG_ID_COMMAND_LONG

In addition to parameters, MavESP8266 also supports a few commands, which will be handled _**if addressed to its component ID**_:

#### MAV_CMD_PREFLIGHT_STORAGE

* If ```param1``` == 0 It will load all parameters from EEPROM overwriting any changes.
* If ```param1``` == 1 It will save all current parameters to EEPROM. 
* If ```param1``` == 2 It will reset all parameters to the original default values. Note that it will not store them to EEPROM. You must request that separately if that's what you want to do.

#### MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN

* If ```param2``` == 1 It will cause the ESP8266 module to reboot. This is necessary if you want parameters changes to take effect. Out of the above, the only parameter that takes effect immediatly upon changing is **DEBUG_ENABLED**. All other values will only take effect at boot time.