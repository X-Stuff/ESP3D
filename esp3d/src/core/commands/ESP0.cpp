/*
 ESP100.cpp - ESP3D command class

 Copyright (c) 2014 Luc Lebosse. All rights reserved.

 This code is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This code is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with This code; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#include "../../include/esp3d_config.h"
#include "../esp3d_commands.h"
#include "../esp3d_settings.h"
const char* help[] = {
    "[ESP] (id) - display this help",
#if defined(WIFI_FEATURE)
    "[ESP100](SSID) - display/set STA SSID",
    "[ESP101](Password) - set STA password",
#endif  // WIFI_FEATURE
#if defined(WIFI_FEATURE)
    "[ESP102](Mode) - display/set STA IP mode (DHCP/STATIC)",
    "[ESP103](IP=xxxx MSK=xxxx GW=xxxx) - display/set STA IP/Mask/GW",
    "[ESP104](State) - display/set sta fallback mode which can be BT, SETUP, "
    "OFF",
    "[ESP105](SSID) - display/set AP SSID",
    "[ESP106](Password) - set AP password",
    "[ESP107](IP) - display/set AP IP",
    "[ESP108](Chanel) - display/set AP chanel",
#endif  // WIFI_FEATURE
#if defined(WIFI_FEATURE) || defined(BLUETOOTH_FEATURE) || defined(ETH_FEATURE)
    "[ESP110](State) - display/set radio state which can be BT, WIFI-STA, "
    "WIFI-AP, WIFI-SETUP, ETH-STA, OFF",
#endif  // WIFI_FEATURE || BLUETOOTH_FEATURE || ETH_FEATURE
#if defined(WIFI_FEATURE) || defined(ETH_FEATURE)
    "[ESP111](OUTPUT=PRINT)(ALL) display current IP and network informations",
#endif  // WIFI_FEATURE || ETH_FEATURE
#if defined(WIFI_FEATURE) || defined(ETH_FEATURE) || defined(BT_FEATURE)
    "[ESP112](Hostname) - display/set Hostname",
    "[ESP114](State) - display/set boot Network state which can be ON, OFF",
    "[ESP115](State) - display/set immediate Network state which can be ON, "
    "OFF",
#endif  // WIFI_FEATURE || ETH_FEATURE || BT_FEATURE
#if defined(ETH_FEATURE)
    "[ESP116](Mode) - display/set ETH STA IP mode (DHCP/STATIC)",
    "[ESP117](IP=xxxx MSK=xxxx GW=xxxx) - display/set ETH STA IP/Mask/GW",
    "[ESP118](State) - display/set eth sta fallback mode which can be BT, Off",
#endif  // ETH_FEATURE
#if defined(HTTP_FEATURE)
    "[ESP120](State) - display/set HTTP state which can be ON, OFF",
    "[ESP121](Port) - display/set HTTP port ",
#endif  // HTTP_FEATURE
#if defined(TELNET_FEATURE)
    "[ESP130](State) - display/set Telnet state which can be ON, OFF",
    "[ESP131](Port) - display/set Telnet port",
#endif  // TELNET_FEATURE
#if defined(TIMESTAMP_FEATURE)
    "[ESP140](SYNC) (srv1=xxxx) (srv2=xxxx) (srv3=xxxx) (tzone=xxx) "
    "(time=YYYY-MM-DDTHH:mm:ss)(ntp=YES/NO) (SYNC) (NOW)- sync/display/set "
    "current time/time servers",
#endif  // TIMESTAMP_FEATURE
    "[ESP150](delay=time) (verbose=ON/OFF)- display/set boot delay in ms / "
    "Verbose boot",
#if defined(WS_DATA_FEATURE)
    "[ESP160](State) - display/set WebSocket state which can be ON, OFF, CLOSE",
    "[ESP161](Port) - display/set WebSocket port",
#endif  // WS_DATA_FEATURE
#if defined(CAMERA_DEVICE)
    "[ESP170](json) (label=value) - display/set Camera commands",
    "[ESP171] (path=<target path>) (filename=<target filename>) Save frame to "
    "target path and filename",
#endif  // CAMERA_DEVICE
#if defined(FTP_FEATURE)
    "[ESP180](State) - display/set FTP state which can be ON, OFF",
    "[ESP181](ctrl=xxxx) (active=xxxx) (passive=xxxx) - display/set FTP ports",
#endif  // FTP_FEATURE
#if defined(WEBDAV_FEATURE)
    "[ESP190](State) - display/set WebDav state which can be ON, OFF",
    "[ESP191](Port) - display/set WebDav port",
#endif  // WEBDAV_FEATURE
#if defined(SD_DEVICE)
    "[ESP200] (json) (RELEASE) (REFRESH)- display/set SD Card Status",
#endif  // SD_DEVICE
#ifdef DIRECT_PIN_FEATURE
    "[ESP201](P=xxx) (V=xxx) (PULLUP=YES RAW=YES ANALOG=NO ANALOG_RANGE=255) - "
    "read / set  pin value",
#endif  // DIRECT_PIN_FEATURE
#if defined(SD_DEVICE)
#if SD_DEVICE != ESP_SDIO
    "[ESP202] SPEED=(factor) - display / set  SD Card  SD card Speed factor (1 "
    "2 4 6 8 16 32)",
#endif  // SD_DEVICE != ESP_SDIO
#endif  // SD_DEVICE
#ifdef SENSOR_DEVICE
    "[ESP210](type=NONE/xxx) (interval=xxxx) - display and read/set SENSOR "
    "info",
#endif  // SENSOR_DEVICE
#if defined(PRINTER_HAS_DISPLAY)
    "[ESP212](text) - display (text) to printer screen status",
#endif  // PRINTER_HAS_DISPLAY
#if defined(DISPLAY_DEVICE)
    "[ESP214](text) - display (text) to ESP screen status",
#if defined(DISPLAY_TOUCH_DRIVER)
    "[ESP215](CALIBRATE) - display state / start touch calibration",
#endif  // DISPLAY_TOUCH_DRIVER
#endif  // DISPLAY_DEVICE
    "[ESP220] - Show used pins",
#ifdef BUZZER_DEVICE
    "[ESP250]F=(frequency) D=(duration) - play sound on buzzer",
#endif  // BUZZER_DEVICE
    "[ESP290](delay in ms) - do a pause",
#if defined(ESP_LUA_INTERPRETER_FEATURE)
    "[ESP300]<filename> - execute Lua script",
    "[ESP301]action=<PAUSE/RESUME/ABORT> - query and control ESP300 execution",
#endif  // ESP_LUA_INTERPRETER_FEATURE
    "[ESP400] - display ESP3D settings in JSON",
    "[ESP401]P=(position) T=(type) V=(value) - Set specific setting",
#ifdef SD_UPDATE_FEATURE
    "[ESP402](State) - display/set check update at boot from SD which can be "
    "ON, OFF",
#endif  // SD_UPDATE_FEATURE
#if defined(WIFI_FEATURE)
    "[ESP410]display available AP list (limited to 30) in plain/JSON",
#endif  // WIFI_FEATURE
    "[ESP420]display ESP3D current status in plain/JSON",
    "[ESP444](Cmd) - set ESP3D state (RESET/RESTART)",
#ifdef MDNS_FEATURE
    "[ESP450]display ESP3D list on network",
#endif  // MDNS_FEATURE
#if defined(AUTHENTICATION_FEATURE)
    "[ESP500]display authentication level",
    "[ESP510](timeout) - set/display session time out",
    "[ESP550](password) - change admin password",
    "[ESP555](password) - change user password",
#endif  // AUTHENTICATION_FEATURE
#if defined(NOTIFICATION_FEATURE)
    "[ESP600](message) - send notification",
    "[ESP610]type=(NONE/PUSHOVER/EMAIL/LINE/TELEGRAM/IFTTT/HOMEASSISTANT/WHATSAPP) "
    "(T1=xxx) (T2=xxx) "
    "(TS=xxx) - display/set Notification settings",
    "[ESP620]URL=http://XXXXXX  - send GET notification",
#endif  // NOTIFICATION_FEATURE
#if defined(GCODE_HOST_FEATURE)
    "[ESP700](filename) - read ESP Filesystem file",
    "[ESP701]action=(PAUSE/RESUME/ABORT) - query and control ESP700 stream",
#endif  // GCODE_HOST_FEATURE
#if defined(FILESYSTEM_FEATURE)
    "[ESP710]FORMATFS - Format ESP Filesystem",
#endif  // FILESYSTEM_FEATURE
#if defined(SD_DEVICE)
    "[ESP715]FORMATSD - Format SD Filesystem",
#endif  // SD_DEVICE
#if defined(FILESYSTEM_FEATURE)
    "[ESP720](path) - List ESP Filesystem",
    "[ESP730](Action)=(path) - rmdir / remove / mkdir / exists / create on ESP "
    "FileSystem (path)",
#endif  // FILESYSTEM_FEATURE
#if defined(SD_DEVICE)
    "[ESP740](path)  - List SD Filesystem",
    "[ESP750](Action)=(path) - rmdir / remove / mkdir / exists / create on SD "
    "(path)",
#endif  // SD_DEVICE
#if defined(GLOBAL_FILESYSTEM_FEATURE)
    "[ESP780](path)  - List Global Filesystem",
    "[ESP790](Action)=(path) - rmdir / remove / mkdir / exists / create on "
    "Global Filesystem (path)",
#endif  // GLOBAL_FILESYSTEM_FEATURE
    "[ESP800](time=YYYY-MM-DDTHH:mm:ss)(version=3.0.0-a11)(setup=0/1) - "
    "display FW Informations /set time",
#if COMMUNICATION_PROTOCOL != SOCKET_SERIAL
    "[ESP900](ENABLE/DISABLE) - display/set serial state",
    "[ESP901]<BAUD RATE> - display/set serial baud rate",
#endif  // COMMUNICATION_PROTOCOL != SOCKET_SERIAL
#if defined(USB_SERIAL_FEATURE)
    "[ESP902]<BAUD RATE> - display/set USB Serial baud rate",
#endif  // defined(USB_SERIAL_FEATURE)
#ifdef BUZZER_DEVICE
    "[ESP910](ENABLE/DISABLE) - display/set buzzer state",
#endif  // BUZZER_DEVICE
#if defined(ESP_SERIAL_BRIDGE_OUTPUT)
    "[ESP930](ENABLE/DISABLE/CLOSE) - display/set serial bridge state",
    "[ESP931]<BAUD RATE> - display/set serial bridge baud rate",
#endif  // defined(ESP_SERIAL_BRIDGE_OUTPUT)
#if defined(USB_SERIAL_FEATURE)
    "[ESP950]<SERIAL/USB> - display/set client output",
#endif  // defined(USB_SERIAL_FEATURE)
#if defined(ARDUINO_ARCH_ESP32) &&                             \
    (CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32S2 || \
     CONFIG_IDF_TARGET_ESP32C3)
    "[ESP999](QUIETBOOT) [pwd=<admin/user password>] - set quiet boot mode",
#endif  // ARDUINO_ARCH_ESP32
};
const uint cmdlist[] = {
    0,
#if defined(WIFI_FEATURE)
    100, 101,
#endif  // WIFI_FEATURE
#if defined(WIFI_FEATURE) 
    102, 103,
    104,
    105, 106, 107, 108,
#endif  // WIFI_FEATURE
#if defined(WIFI_FEATURE) || defined(BLUETOOTH_FEATURE) || defined(ETH_FEATURE)
    110,
#endif  // WIFI_FEATURE || BLUETOOTH_FEATURE || ETH_FEATURE
#if defined(WIFI_FEATURE) || defined(ETH_FEATURE)
    111,
#endif  // WIFI_FEATURE || ETH_FEATURE
#if defined(WIFI_FEATURE) || defined(ETH_FEATURE) || defined(BT_FEATURE)
    112, 114, 115,
#endif  // WIFI_FEATURE || ETH_FEATURE || BT_FEATURE
#if defined(ETH_FEATURE) 
    116, 117, 118,
#endif  // ETH_FEATURE
#if defined(HTTP_FEATURE)
    120, 121,
#endif  // HTTP_FEATURE
#if defined(TELNET_FEATURE)
    130, 131,
#endif  // TELNET_FEATURE
#if defined(TIMESTAMP_FEATURE)
    140,
#endif  // TIMESTAMP_FEATURE
    150,
#if defined(WS_DATA_FEATURE)
    160, 161,
#endif  // WS_DATA_FEATURE
#if defined(CAMERA_DEVICE)
    170, 171,
#endif  // CAMERA_DEVICE
#if defined(FTP_FEATURE)
    180, 181,
#endif  // FTP_FEATURE
#if defined(WEBDAV_FEATURE)
    190, 191,
#endif  // WEBDAV_FEATURE
#if defined(SD_DEVICE)
    200,
#endif  // SD_DEVICE
#ifdef DIRECT_PIN_FEATURE
    201,
#endif  // DIRECT_PIN_FEATURE
#if defined(SD_DEVICE) && SD_DEVICE != ESP_SDIO
    202,
#endif  // SD_DEVICE
#ifdef SENSOR_DEVICE
    210,
#endif  // SENSOR_DEVICE
#if defined(PRINTER_HAS_DISPLAY)
    212,
#endif  // PRINTER_HAS_DISPLAY
#if defined(DISPLAY_DEVICE)
    214,
#if defined(DISPLAY_TOUCH_DRIVER)
    215,
#endif  // DISPLAY_TOUCH_DRIVER
#endif  // DISPLAY_DEVICE
    220,
#ifdef BUZZER_DEVICE
    250,
#endif  // BUZZER_DEVICE
    290,
#if defined(ESP_LUA_INTERPRETER_FEATURE)
    300, 301,
#endif  // ESP_LUA_INTERPRETER_FEATURE
     400, 401,
#ifdef SD_UPDATE_FEATURE
    402,
#endif  // SD_UPDATE_FEATURE
#if defined(WIFI_FEATURE)
    410,
#endif  // WIFI_FEATURE
    420, 444,
#ifdef MDNS_FEATURE
    450,
#endif  // MDNS_FEATURE
#if defined(AUTHENTICATION_FEATURE)
    500, 510, 550, 555,
#endif  // AUTHENTICATION_FEATURE
#if defined(NOTIFICATION_FEATURE)
    600, 610, 620,
#endif  // NOTIFICATION_FEATURE
#if defined(GCODE_HOST_FEATURE)
    700, 701,
#endif  // GCODE_HOST_FEATURE
#if defined(FILESYSTEM_FEATURE)
    710,
#endif  // FILESYSTEM_FEATURE
#if defined(SD_DEVICE)
    715,
#endif  // SD_DEVICE
#if defined(FILESYSTEM_FEATURE)
    720, 730,
#endif  // FILESYSTEM_FEATURE
#if defined(SD_DEVICE)
    740, 750,
#endif  // SD_DEVICE
#if defined(GLOBAL_FILESYSTEM_FEATURE)
    780, 790,
#endif  // GLOBAL_FILESYSTEM_FEATURE
    800,
#if COMMUNICATION_PROTOCOL != SOCKET_SERIAL
    900, 901,
#endif  // COMMUNICATION_PROTOCOL != SOCKET_SERIAL
#if defined(USB_SERIAL_FEATURE)
    902,
#endif  // defined(USB_SERIAL_FEATURE)
#ifdef BUZZER_DEVICE
    910,
#endif  // BUZZER_DEVICE
#if defined(ESP_SERIAL_BRIDGE_OUTPUT)
    930, 931,
#endif  // defined(ESP_SERIAL_BRIDGE_OUTPUT)
#if defined(USB_SERIAL_FEATURE)
    950,
#endif  // defined(USB_SERIAL_FEATURE)
#if defined(ARDUINO_ARCH_ESP32) &&                             \
    (CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32S2 || \
     CONFIG_IDF_TARGET_ESP32C3)
    999,
#endif  // ARDUINO_ARCH_ESP32 && CONFIG_IDF_TARGET_ESP32S3 ||
        // CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32C3
};

// ESP3D Help
//[ESP0] or [ESP]<command>
void ESP3DCommands::ESP0(int cmd_params_pos, ESP3DMessage* msg) {
  ESP3DClientType target = msg->origin;
  ESP3DRequest requestId = msg->request_id;
  msg->target = target;
  msg->origin = ESP3DClientType::command;
  String tmpstr;
  const uint cmdNb = sizeof(help) / sizeof(char*);
  const uint cmdlistNb = sizeof(cmdlist) / sizeof(uint);
  bool json = hasTag(msg, cmd_params_pos, "json");
  if (cmdNb != cmdlistNb) {
    esp3d_log("Help corrupted: %d vs %d", cmdNb, cmdlistNb);
    msg->type = ESP3DMessageType::unique;
    if (!dispatch(msg, "Help corrupted")) {
      esp3d_log_e("Error sending command to clients");
    }
    return;
  }
  tmpstr = get_clean_param(msg, cmd_params_pos);
  if (tmpstr.length() == 0) {
    // use msg for first answer
    if (json) {
      tmpstr = "{\"cmd\":\"0\",\"status\":\"ok\",\"data\":[";
    } else {
      tmpstr = "[List of ESP3D commands]\n";
    }
    msg->type = ESP3DMessageType::head;
    if (!dispatch(msg, tmpstr.c_str())) {
      esp3d_log_e("Error sending command to clients");
      return;
    }
    yield();
    for (uint i = 0; i < cmdNb; i++) {
      if (json) {
        tmpstr = "{\"id\":\"";
        tmpstr += String(cmdlist[i]);
        tmpstr += "\",\"help\":\"";
        tmpstr += help[i];
        tmpstr += "\"}";
        if (i < cmdNb - 1 && (cmdNb - 1) > 0) {
          tmpstr += ",";
        }
      } else {
        tmpstr = help[i];
        tmpstr += "\n";
      }

      if (!dispatch(tmpstr.c_str(), target, requestId,
                    ESP3DMessageType::core)) {
        esp3d_log_e("Error sending answer to clients");
        return;
      }
      esp3d_log("Help sent: %d", i);

      yield();
    }

    if (json) {
      if (!dispatch("]}", target, requestId, ESP3DMessageType::tail)) {
        esp3d_log_e("Error sending answer to clients");
        return;
      }
    } else {
      if (!dispatch("ok\n", target, requestId, ESP3DMessageType::tail)) {
        esp3d_log_e("Error sending answer to clients");
        return;
      }
    }
  } else {
    uint cmdval = atoi(tmpstr.c_str());
    for (uint i = 0; i < cmdNb; i++) {
      if (cmdlist[i] == cmdval) {
        if (json) {
          tmpstr = "{\"cmd\":\"0\",\"status\":\"ok\",\"data\":{\"id\":\"";
          tmpstr += String(cmdval);
          tmpstr += "\",\"help\":\"";
          tmpstr += help[i];
          tmpstr += "\"}}";
        } else {
          tmpstr = help[i];
          tmpstr += "\n";
        }
        msg->type = ESP3DMessageType::unique;
        if (!dispatch(msg, tmpstr.c_str())) {
          return;
        }
        return;
      }
    }
    if (json) {
      tmpstr =
          "{\"cmd\":\"0\",\"status\":\"error\",\"data\":\"This command is not "
          "supported: ";
      tmpstr += String(cmdval);
      tmpstr += "\"}";
    } else {
      tmpstr = "This command is not supported: ";
      tmpstr += String(cmdval);
      tmpstr += "\n";
    }
    msg->type = ESP3DMessageType::unique;
    if (!dispatch(msg, tmpstr.c_str())) {
      return;
    }
  }
}
