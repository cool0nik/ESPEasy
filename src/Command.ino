
#include "src/Commands/Common.h"
#ifdef USES_BLYNK
# include "src/Commands/Blynk.h"
# include "src/Commands/Blynk_c015.h"
#endif // ifdef USES_BLYNK
#include "src/Commands/Diagnostic.h"
#include "src/Commands/HTTP.h"
#include "src/Commands/i2c.h"
#ifdef USES_MQTT
# include "src/Commands/MQTT.h"
#endif // USES_MQTT
#include "src/Commands/Networks.h"
#include "src/Commands/Notifications.h"
#include "src/Commands/RTC.h"
#include "src/Commands/Rules.h"
#include "src/Commands/SDCARD.h"
#include "src/Commands/Settings.h"
#include "src/Commands/System.h"
#include "src/Commands/Tasks.h"
#include "src/Commands/Time.h"
#include "src/Commands/Timer.h"
#include "src/Commands/UPD.h"
#include "src/Commands/wd.h"
#include "src/Commands/WiFi.h"

#include "ESPEasy_common.h"


bool checkNrArguments(const char *cmd, const char *Line, int nrArguments) {
  if (nrArguments < 0) { return true; }

  // 0 arguments means argument on pos1 is valid (the command) and argpos 2 should not be there.
  if (HasArgv(Line, nrArguments + 2)) {
    if (loglevelActiveFor(LOG_LEVEL_ERROR)) {
      String log;
      log.reserve(128);
      log += F("Too many arguments: cmd=");
      log += cmd;

      if (nrArguments < 1) {
        log += Line;
      } else {
        // Check for one more argument than allowed, since we apparently have one.
        for (int i = 0; i <= nrArguments; ++i) {
          if (i < nrArguments) {
            log += F(" arg");
          } else {
            log += F(" extraArg");
          }
          log += String(i + 1);
          log += '=';
          log += parseString(Line, i + 2);
        }
      }
      log += F(" lineLength=");
      log += strlen(Line);
      addLog(LOG_LEVEL_ERROR, log);
      log = F("Line: _");
      log += Line;
      log += '_';
      addLog(LOG_LEVEL_ERROR, log);
    }
    return false;
  }
  return true;
}

/*********************************************************************************************\
* Registers command
\*********************************************************************************************/
String doExecuteCommand(const char *cmd, struct EventStruct *event, const char *line)
{
  String cmd_lc;

  cmd_lc = cmd;
  cmd_lc.toLowerCase();

  if (loglevelActiveFor(LOG_LEVEL_INFO)) {
    String log = F("Command: ");
    log += cmd_lc;
    addLog( LOG_LEVEL_INFO, log);
#ifndef BUILD_NO_DEBUG
    addLog(LOG_LEVEL_DEBUG, line); // for debug purposes add the whole line.
    String parameters;
    parameters.reserve(64);
    parameters += F("Par1: ");
    parameters += event->Par1;
    parameters += F(" Par2: ");
    parameters += event->Par2;
    parameters += F(" Par3: ");
    parameters += event->Par3;
    parameters += F(" Par4: ");
    parameters += event->Par4;
    parameters += F(" Par5: ");
    parameters += event->Par5;
    addLog(LOG_LEVEL_DEBUG, parameters);
#endif // ifndef BUILD_NO_DEBUG
  }

  // Simple macro to match command to function call.
  #define COMMAND_CASE(S, C, NARGS) \
  if (strcmp_P(cmd_lc.c_str(),      \
               PSTR(S)) ==          \
      0) { if (!checkNrArguments(cmd, line, NARGS)) { return return_incorrect_nr_arguments(); } else  return C (event, line); }

  // FIXME TD-er: Should we execute command when number of arguments is wrong?

  // FIXME TD-er: must determine nr arguments where NARGS is set to -1

  switch (cmd_lc[0]) {
    case 'a': {
      COMMAND_CASE("accessinfo", Command_AccessInfo_Ls, 0); // Network Command
      break;
    }
    case 'b': {
      COMMAND_CASE("background", Command_Background,     1); // Diagnostic.h
    #ifdef USES_C012
      COMMAND_CASE(  "blynkget", Command_Blynk_Get,     -1);
    #endif // ifdef USES_C012
    #ifdef USES_C015
      COMMAND_CASE(  "blynkset", Command_Blynk_Set,     -1);
    #endif // ifdef USES_C015
      COMMAND_CASE(     "build", Command_Settings_Build, 1); // Settings.h
      break;
    }
    case 'c': {
      COMMAND_CASE("clearaccessblock", Command_AccessInfo_Clear,   0); // Network Command
      COMMAND_CASE(     "clearrtcram", Command_RTC_Clear,          0); // RTC.h
      COMMAND_CASE(          "config", Command_Task_RemoteConfig, -1); // Tasks.h
      break;
    }
    case 'd': {
      COMMAND_CASE(    "debug", Command_Debug,            1); // Diagnostic.h
      COMMAND_CASE("deepsleep", Command_System_deepSleep, 1); // System.h
      COMMAND_CASE(    "delay", Command_Delay,            1); // Timers.h
      COMMAND_CASE(      "dns", Command_DNS,              1); // Network Command
      COMMAND_CASE(      "dst", Command_DST,              1); // Time.h
      break;
    }
    case 'e': {
      COMMAND_CASE(       "erase", Command_WiFi_Erase,     0); // WiFi.h
      COMMAND_CASE(       "event", Command_Rules_Events,  -1); // Rule.h
      COMMAND_CASE("executerules", Command_Rules_Execute, -1); // Rule.h
      break;
    }
    case 'g': {
      COMMAND_CASE("gateway", Command_Gateway, 1); // Network Command
      break;
    }
    case 'i': {
      COMMAND_CASE("i2cscanner", Command_i2c_Scanner, -1); // i2c.h
      COMMAND_CASE(        "ip", Command_IP,           1); // Network Command
      break;
    }
    case 'j': {
      COMMAND_CASE("jsonportstatus", Command_JSONPortStatus, -1); // Diagnostic.h
    }
    case 'l': {
      COMMAND_CASE(          "let", Command_Rules_Let,     2);    // Rules.h
      COMMAND_CASE(         "load", Command_Settings_Load, 0);    // Settings.h
      COMMAND_CASE(     "logentry", Command_logentry,      2);    // Diagnostic.h
      COMMAND_CASE("logportstatus", Command_logPortStatus, 0);    // Diagnostic.h
      COMMAND_CASE(       "lowmem", Command_Lowmem,        0);    // Diagnostic.h
      break;
    }
    case 'm': {
      COMMAND_CASE(        "malloc", Command_Malloc,            1); // Diagnostic.h
      COMMAND_CASE(       "meminfo", Command_MemInfo,           0); // Diagnostic.h
      COMMAND_CASE( "meminfodetail", Command_MemInfo_detail,    0); // Diagnostic.h
#ifdef USES_MQTT
      COMMAND_CASE(  "messagedelay", Command_MQTT_messageDelay, 1); // MQTT.h
      COMMAND_CASE("mqttretainflag", Command_MQTT_Retain,       1); // MQTT.h
#endif // USES_MQTT
      break;
    }
    case 'n': {
      COMMAND_CASE(   "name", Command_Settings_Name,        1); // Settings.h
      COMMAND_CASE("nosleep", Command_System_NoSleep,       1); // System.h
      COMMAND_CASE( "notify", Command_Notifications_Notify, 2); // Notifications.h
      COMMAND_CASE("ntphost", Command_NTPHost,              1); // Time.h
      break;
    }
    case 'p': {
      COMMAND_CASE("password", Command_Settings_Password, 1); // Settings.h
#ifdef USES_MQTT
      COMMAND_CASE( "publish", Command_MQTT_Publish,      2); // MQTT.h
#endif // USES_MQTT
      break;
    }
    case 'r': {
      COMMAND_CASE(                "reboot", Command_System_Reboot,              0); // System.h
      COMMAND_CASE(                 "reset", Command_Settings_Reset,             0); // Settings.h
      COMMAND_CASE("resetflashwritecounter", Command_RTC_resetFlashWriteCounter, 0); // RTC.h
      COMMAND_CASE(               "restart", Command_System_Restart,             0); // System.h
      COMMAND_CASE(                 "rules", Command_Rules_UseRules,             1); // Rule.h
      break;
    }
    case 's': {
      COMMAND_CASE(       "save", Command_Settings_Save,   0); // Settings.h
    #ifdef USES_MQTT
	    COMMAND_CASE(  "subscribe", Command_MQTT_Subscribe,  1);  // MQTT.h  
    #endif // USES_MQTT
        #ifdef FEATURE_SD
      COMMAND_CASE(     "sdcard", Command_SD_LS,           0); // SDCARDS.h
      COMMAND_CASE(   "sdremove", Command_SD_Remove,       1); // SDCARDS.h
        #endif // ifdef FEATURE_SD
      COMMAND_CASE(     "sendto", Command_UPD_SendTo,      2); // UDP.h    // FIXME TD-er: These send commands, can we determine the nr of
                                                               // arguments?
      COMMAND_CASE( "sendtohttp", Command_HTTP_SendToHTTP, 3); // HTTP.h
      COMMAND_CASE(  "sendtoudp", Command_UDP_SendToUPD,   3); // UDP.h
      COMMAND_CASE("serialfloat", Command_SerialFloat,     0); // Diagnostic.h
      COMMAND_CASE(   "settings", Command_Settings_Print,  0); // Settings.h
      COMMAND_CASE(     "subnet", Command_Subnet,          1); // Network Command
      COMMAND_CASE(    "sysload", Command_SysLoad,         0); // Diagnostic.h
      break;
    }
    case 't': {
      COMMAND_CASE(         "taskclear", Command_Task_Clear,          1); // Tasks.h
      COMMAND_CASE(      "taskclearall", Command_Task_ClearAll,       0); // Tasks.h
      COMMAND_CASE(           "taskrun", Command_Task_Run,            1); // Tasks.h
      COMMAND_CASE(      "taskvalueset", Command_Task_ValueSet,       3); // Tasks.h
      COMMAND_CASE(   "taskvaluetoggle", Command_Task_ValueToggle,    2); // Tasks.h
      COMMAND_CASE("taskvaluesetandrun", Command_Task_ValueSetAndRun, 3); // Tasks.h
      COMMAND_CASE(        "timerpause", Command_Timer_Pause,         1); // Timers.h
      COMMAND_CASE(       "timerresume", Command_Timer_Resume,        1); // Timers.h
      COMMAND_CASE(          "timerset", Command_Timer_Set,           2); // Timers.h
      COMMAND_CASE(          "timezone", Command_TimeZone,            1); // Time.h
      break;
    }
    case 'u': {
      COMMAND_CASE("udpport", Command_UDP_Port,      1); // UDP.h
      COMMAND_CASE("udptest", Command_UDP_Test,      2); // UDP.h
      COMMAND_CASE(   "unit", Command_Settings_Unit, 1); // Settings.h
      COMMAND_CASE( "usentp", Command_useNTP,        1); // Time.h
      break;
    }
    case 'w': {
      COMMAND_CASE(      "wdconfig", Command_WD_Config,       3); // WD.h
      COMMAND_CASE(        "wdread", Command_WD_Read,         2); // WD.h
      COMMAND_CASE(    "wifiapmode", Command_Wifi_APMode,     0); // WiFi.h
      COMMAND_CASE(   "wificonnect", Command_Wifi_Connect,    0); // WiFi.h
      COMMAND_CASE("wifidisconnect", Command_Wifi_Disconnect, 0); // WiFi.h
      COMMAND_CASE(       "wifikey", Command_Wifi_Key,        1); // WiFi.h
      COMMAND_CASE(      "wifikey2", Command_Wifi_Key2,       1); // WiFi.h
      COMMAND_CASE(      "wifimode", Command_Wifi_Mode,       1); // WiFi.h
      COMMAND_CASE(      "wifiscan", Command_Wifi_Scan,       0); // WiFi.h
      COMMAND_CASE(      "wifissid", Command_Wifi_SSID,       1); // WiFi.h
      COMMAND_CASE(     "wifissid2", Command_Wifi_SSID2,      1); // WiFi.h
      COMMAND_CASE(   "wifistamode", Command_Wifi_STAMode,    0); // WiFi.h
      break;
    }
    default:
      break;
  }

  if (loglevelActiveFor(LOG_LEVEL_INFO)) {
    String errorUnknown = F("Command unknown: \"");
    errorUnknown += cmd_lc;
    errorUnknown += '\"';
    addLog(LOG_LEVEL_INFO, errorUnknown);
  }
  return F("\nUnknown command!");

  #undef COMMAND_CASE
}

void ExecuteCommand(byte source, const char *Line)
{
  checkRAM(F("ExecuteCommand"));
  String cmd;

  if (!GetArgv(Line, cmd, 1)) {
    return;
  }
  struct EventStruct TempEvent;

  // FIXME TD-er: Not sure what happens now, but TaskIndex cannot be set here
  // since commands can originate from anywhere.
  TempEvent.Source = source;

  // Split the arguments into Par1...5 of the event.
  // Do not split it in doExecuteCommand, since that one will be called from the scheduler with pre-set events.
  // FIXME TD-er: Why call this for all commands? The CalculateParam function is quite heavy.
  parseCommandString(&TempEvent, Line);

  String status = doExecuteCommand(cmd.c_str(), &TempEvent, Line);
  delay(0);
  SendStatus(source, status);
  delay(0);

  /*
     } else {
      // Schedule to run async
      schedule_command_timer(cmd.c_str(), &TempEvent, Line);
     }
   */
}

#ifdef FEATURE_SD
void printDirectory(File dir, int numTabs)
{
  while (true) {
    File entry = dir.openNextFile();

    if (!entry) {
      // no more files
      break;
    }

    for (uint8_t i = 0; i < numTabs; i++) {
      serialPrint("\t");
    }
    serialPrint(entry.name());

    if (entry.isDirectory()) {
      serialPrintln("/");
      printDirectory(entry, numTabs + 1);
    } else {
      // files have sizes, directories do not
      serialPrint("\t\t");
      serialPrintln(String(entry.size(), DEC));
    }
    entry.close();
  }
}

#endif // ifdef FEATURE_SD
