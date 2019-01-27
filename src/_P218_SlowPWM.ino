#define USES_P218
#ifdef USES_P218
//#######################################################################################################
//#################################### Plugin 218: PWM Power Control ####################################
//#######################################################################################################
#define PLUGIN_218
#define PLUGIN_ID_218        218
#define PLUGIN_NAME_218      "Regulator - Slow/Soft PWM [TESTING]"

#define PLUGIN_VALUENAME1_218 "Output"

#define PLUGINT_MAX_PERIOD_218 30000
#define PLUGINT_MIN_PERIOD_218 4

// A plugin has implement the Bresenham PWM fo slow devices (heaters and so on)

/*
  Main reason to have slow PWN.... Sometimes we need to works with AC.
  You need to get a part of period/search zero/... to make able power regulation on AC.... 
  or (for slow device/heaters) control of series of periods (triac)
  
  So I use 1 sec as base frequency for PWM in this plugin. You can switch it to 1/10 sec as you wish.
  Of course you should understand the consequences.

  I use it with heater/regulator. It is incomparably better than _P021_Level to make temperature stable.
  The plugin take float from another plugin [0-1] (power factor) and use to control power [0-100]%
  Values less 0 use as 0 (off). Values greater 1 assume as 1 (100% power).

  I use it as follows:
  DS18b20 (P004_Dallas) temperature -> 
  "Rule" calculetes power and put it to P033_Dummy (e.g. TaskValueSet,3,1,(22-%eventvalue%)/16 )
  This power factor is used by this plugin.

  So I have P (proporcional) power regulator.
  This can work offline.

  You can implement any robust power controller by using different calculation or/and external calculations of power factor.
*/

static_assert(VARS_PER_TASK >= 3, "VARS_PER_TASK should be greater or equal 3");
static_assert(PLUGIN_CONFIGVAR_MAX >= 4, "PLUGIN_CONFIGVAR_MAX should be greater or equal 4");

// compiler fails on c++ style code. will try C style

// I want to use type inference but compiler fails.
// check that the types have not changed
static_assert(std::is_same<std::remove_all_extents<decltype(Settings.TaskDevicePluginConfigFloat)>::type, float>::value, "Needs to update types in plugin");
static_assert(std::is_same<std::remove_all_extents<decltype(Settings.TaskDevicePluginConfig)>::type, int16_t>::value, "Needs to update types in plugin");
static_assert(sizeof(float) >= 2 * sizeof(int16_t), "plugin use memory allocatied for float as two int16");

// to avoid the “strict aliasing” warning that appears when make this by using “define”
// compiler produce bad code if all "#define" replace by "template inline"
template<typename T>
inline int16_t * GET_HIGH_HALF_FLOAT_AS_INT_POINTER_218(T event, int16_t id)
{
  return reinterpret_cast<int16_t*>(&(UserVar[event->BaseVarIndex + id]));
}

#define GET_LOW_HALF_FLOAT_AS_INT_POINTER_218(event, id)  (&(GET_HIGH_HALF_FLOAT_AS_INT_POINTER_218(event, id)[1]))

// config int
#define PLUGIN_CONTROLLED_ID_218       0
#define PLUGIN_CONTROLLED_VAR_ID_218   1
#define PLUGIN_PERIOD_218              2

// flag id (checkbox)
// #define FAST_PWM_ID_218                0
#define INVERT_OUTPTUT_218             1

// user var
#define PLUGIN_SHOW_STATE_218          0
#define PLUGIN_PACKED_INT12_218        1 // state + error pwm
#define PLUGIN_PACKED_INT34_218        2 // step  + previous value pwm

#define GET_PWM_CUR_SATE_POINTER_218(event)           (GET_HIGH_HALF_FLOAT_AS_INT_POINTER_218(event, PLUGIN_PACKED_INT12_218))
#define GET_PWM_ERROR_POINTER_218(event)              (GET_LOW_HALF_FLOAT_AS_INT_POINTER_218(event, PLUGIN_PACKED_INT12_218))
#define GET_PWM_STEP_POINTER_218(event)               (GET_HIGH_HALF_FLOAT_AS_INT_POINTER_218(event, PLUGIN_PACKED_INT34_218))
#define GET_PWM_PREVIOUS_VALUE_POINTER_218(event)     (GET_LOW_HALF_FLOAT_AS_INT_POINTER_218(event, PLUGIN_PACKED_INT34_218))
#define GET_PWM_CUR_SATE_SHOW_218(event)              (UserVar[event->BaseVarIndex + PLUGIN_SHOW_STATE_218])

#define GET_PLUGIN_FLAG_218(event, mask)              (Settings.TaskDevicePluginConfigLong[event->TaskIndex][mask] != 0)
#define SET_PLUGIN_FLAG_218(event, mask, data)        (Settings.TaskDevicePluginConfigLong[event->TaskIndex][mask] = data)

boolean Plugin_218(byte function, struct EventStruct *event, String& string)
{
  boolean success = false;

  switch (function)
  {
    case PLUGIN_DEVICE_ADD:
    {
        //This case defines the device characteristics, edit appropriately

        Device[++deviceCount].Number = PLUGIN_ID_218;
        Device[deviceCount].Type = DEVICE_TYPE_SINGLE;  // how the device is connected
        Device[deviceCount].VType = SENSOR_TYPE_SWITCH; // SENSOR_TYPE_NONE; //type of value the plugin will return, used only for Domoticz
        Device[deviceCount].Ports = 0;
        Device[deviceCount].PullUpOption = false;
        Device[deviceCount].InverseLogicOption = false;
        Device[deviceCount].FormulaOption = false;
        Device[deviceCount].ValueCount = 1;             //number of output variables. The value should match the number of keys PLUGIN_VALUENAME1_xxx
        Device[deviceCount].SendDataOption = true;
        Device[deviceCount].TimerOption = false;
        break;
    }

    case PLUGIN_GET_DEVICENAME:
    {
      //return the device name
      string = F(PLUGIN_NAME_218);
      break;
    }

    case PLUGIN_GET_DEVICEVALUENAMES:
    {
      strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[0], PSTR(PLUGIN_VALUENAME1_218));
      break;
    }

    case PLUGIN_GET_DEVICEGPIONAMES:
    {
      event->String1 = formatGpioName_output(F("PWM"));
      break;
    }

    case PLUGIN_WEBFORM_LOAD:
    {
      addHtml(F("<TR><TD>Check Task:<TD>"));
      addTaskSelect(F("p218_task"), PCONFIG(PLUGIN_CONTROLLED_ID_218));

      LoadTaskSettings(PCONFIG(PLUGIN_CONTROLLED_ID_218)); // we need to load the values from another task for selection!
      addHtml(F("<TR><TD>Check Value:<TD>"));
      addTaskValueSelect(F("p218_value"), PCONFIG(PLUGIN_CONTROLLED_VAR_ID_218), PCONFIG(PLUGIN_CONTROLLED_ID_218));

      addFormNumericBox(F("Set Period"),
        F("p218_period"), 
        p218_normolize_value(PCONFIG(PLUGIN_PERIOD_218), PLUGINT_MIN_PERIOD_218, PLUGINT_MAX_PERIOD_218),
        PLUGINT_MIN_PERIOD_218,
        PLUGINT_MAX_PERIOD_218
      );

      addFormCheckBox(F("Invert output"), F("p218_invert"), GET_PLUGIN_FLAG_218(event, INVERT_OUTPTUT_218));
#ifdef FAST_PWM_ID_218
      addFormCheckBox(F("use 1/10s period"),  F("p218_tenth"), GET_PLUGIN_FLAG_218(event, FAST_PWM_ID_218));
#endif      
      LoadTaskSettings(event->TaskIndex);
      success = true;
      break;
    }

    case PLUGIN_WEBFORM_SAVE:
    {
      PCONFIG(PLUGIN_CONTROLLED_ID_218) = getFormItemInt(F("p218_task"));
      PCONFIG(PLUGIN_CONTROLLED_VAR_ID_218) = getFormItemInt(F("p218_value"));
      PCONFIG(PLUGIN_PERIOD_218) = p218_normolize_value(getFormItemInt(F("p218_period")), PLUGINT_MIN_PERIOD_218, PLUGINT_MAX_PERIOD_218);

      SET_PLUGIN_FLAG_218(event, INVERT_OUTPTUT_218, isFormItemChecked(F("p218_invert")));
#ifdef FAST_PWM_ID_218
      SET_PLUGIN_FLAG_218(event, FAST_PWM_ID_218, isFormItemChecked(F("p218_tenth")));
#endif
      p218_init_settings(event);
      
      success = true;
      break;

    }
    case PLUGIN_INIT:
    {
      pinMode(Settings.TaskDevicePin1[event->TaskIndex], OUTPUT);
      success = true;
      break;

    }

	  case PLUGIN_EXIT:
	  {
	    digitalWrite(Settings.TaskDevicePin1[event->TaskIndex], GET_PLUGIN_FLAG_218(event, INVERT_OUTPTUT_218) ? 1 : 0);
      success = true;
	    break;

	  }
#ifdef FAST_PWM_ID_218
    case PLUGIN_TEN_PER_SECOND:
    {
      if (GET_PLUGIN_FLAG_218(event, FAST_PWM_ID_218)) {
        p218_next_step(event);
      }
      success = true;
      break;
    }
#endif

    case PLUGIN_ONCE_A_SECOND:
    {
#ifdef FAST_PWM_ID_218
      if (!GET_PLUGIN_FLAG_218(event, FAST_PWM_ID_218))
#endif
      {
        p218_next_step(event);
      }
      success = true;
      break;
    }
  }   // switch
  return success;

}     //function

template<typename T>
inline T p218_normolize_value(T t, int16_t min, int16_t max)
{
  if (t < min)
    return min;
  if (t > max)
    return max;
  return t;
}

void p218_init_settings(struct EventStruct *event)
{
  GET_PWM_CUR_SATE_SHOW_218(event) = GET_PLUGIN_FLAG_218(event, INVERT_OUTPTUT_218) ? 1.0 : 0.0;
  *GET_PWM_CUR_SATE_POINTER_218(event) = 0;
  *GET_PWM_ERROR_POINTER_218(event) = PCONFIG(PLUGIN_PERIOD_218) / 2;
  *GET_PWM_STEP_POINTER_218(event) = 0;
}

void p218_next_step(struct EventStruct *event)
{
  int16_t  period = PCONFIG(PLUGIN_PERIOD_218);
  int16_t  value;
  {
    int16_t* prev = GET_PWM_PREVIOUS_VALUE_POINTER_218(event);
    int16_t BaseVarIndex = PCONFIG(PLUGIN_CONTROLLED_ID_218) * VARS_PER_TASK + PCONFIG(PLUGIN_CONTROLLED_VAR_ID_218);
    float float_value = UserVar[BaseVarIndex];
    if (float_value == float_value) { //  check if value is NaN, use previous value
      value =  p218_normolize_value(float_value * period, 0 , period); // min/max power
      *prev = value; 
    } else {
      value = *prev;
    }
  }
  int16_t* error = GET_PWM_ERROR_POINTER_218(event);
  int16_t* stepNumber = GET_PWM_STEP_POINTER_218(event);
  int16_t* curState = GET_PWM_CUR_SATE_POINTER_218(event);

  int16_t result;
  *error -= value;
  if ( *error < 0 ) {
    *error += period;
    result = 1;
  } else {
    result = 0;
  }
  if ( ++(*stepNumber) >= period) {
    *stepNumber = 0;
    *error = (period) / 2;
  }
  
  if (*curState != result) {
    *curState = result;

    if (GET_PLUGIN_FLAG_218(event, INVERT_OUTPTUT_218)) {
      result = 1 - result;
    }

    digitalWrite(Settings.TaskDevicePin1[event->TaskIndex], result);
    GET_PWM_CUR_SATE_SHOW_218(event) = result;
    sendData(event);
  }
}
/*
typedef struct bresenham_struct {
  uint8_t size;
  uint8_t value;
  int16_t error;
  uint8_t stepNumber;
} bresenham_struct;

void bresenham_init(struct bresenham_struct *st, uint8_t size) {
  st->size = size;
}

void bresenham_setValue(struct bresenham_struct *st, uint8_t val) {
  st->stepNumber = 0;
  st->value = val;
  st->error = st->size/2;
}

bool bresenham_getNext(struct bresenham_struct *st) {
  bool result;
  st->error -= st->value;
  if ( st->error < 0 ) {
    st->error += st->size;
    result = true;
  } else {
    result = false;
  }
  if ( ++st->stepNumber >= st->size) {
    st->stepNumber = 0;
    st->error = st->size/2;
  }
  return result;
}
*/
#endif
