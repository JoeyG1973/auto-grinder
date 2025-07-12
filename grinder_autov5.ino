#include <FastAccelStepper.h>
#include <Bounce2.h>
#include <LiquidCrystal_I2C.h>
#include <SmoothProgress.h>
#define BAR_STYLES_IN_PROGMEM
#include <BarStyleV2.h>
#include <avr/wdt.h> 


#define X_LIMIT_SWITCH_LEFT_PIN 51
#define X_LIMIT_SWITCH_RIGHT_PIN 53
#define X_STEP_PIN 6
#define X_DIR_PIN 5
#define X_ENA_PIN 4
#define X_MAX_SPEED 15000 // in hertz/steps
#define X_POTPIN A0
#define X_ENABLE_SWITCH_PIN 3
#define X_ACCEL 100000


#define Y_STEP_PIN 8
#define Y_DIR_PIN 9
#define Y_ENA_PIN 10
#define Y_MAX_SPEED 25000 // in hertz/steps
#define Y_POTPIN A4
#define Y_MAX_STEP_OVER 24950
#define Y_ENABLE_SWITCH_PIN 2
#define Y_LIMIT_SWITCH_FRONT_PIN 50
#define Y_LIMIT_SWITCH_REAR_PIN 52
#define Y_ACCEL 200000

///////////////////////////////////////
// buttons, switches, sensors objects //
///////////////////////////////////////
Bounce2::Button X_limit_switch_left = Bounce2::Button();
Bounce2::Button X_limit_switch_right = Bounce2::Button();
Bounce2::Button X_toggle_switch = Bounce2::Button();
Bounce2::Button Y_toggle_switch = Bounce2::Button();
Bounce2::Button Y_limit_switch_front = Bounce2::Button();
Bounce2::Button Y_limit_switch_rear = Bounce2::Button();

////////////////////////////
// stepper engine objects //
////////////////////////////
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *Xstepper = NULL;
FastAccelStepper *Ystepper = NULL;

/////////////////
// global vars //
/////////////////
bool X_dir_forward = true;
uint32_t X_pot_map = 0;
uint32_t Y_pot_map = 0;
uint32_t X_pot_EMA_S = 0;
uint32_t Y_pot_EMA_S = 0;
int32_t Y_step_with_dir = 0;
int barX = 0, barY = 0;
char buffer[16];
bool startup = true;
bool Y_limit_switch_front_trip = false, Y_limit_switch_rear_trip = false;
bool X_limit_switch_left_trip = false, X_limit_switch_right_trip = false;

////////////////////////
// lcd global objects //
////////////////////////
LiquidCrystal_I2C lcd(0x27, 20, 4);
LCD dispA(lcd, inPROGMEM(barStyleV2));
SmoothProgressBar spb1(dispA, 4, 9, 3, 0);
SmoothProgressBar spb2(dispA, 4, 19, 3, 1);

void setup()
{ 
  ///////////////////////////////////////////
  // button, switch, sensor, pot, etc init //
  ///////////////////////////////////////////
  X_limit_switch_left.attach( X_LIMIT_SWITCH_LEFT_PIN, INPUT );
  X_limit_switch_left.interval(5);
  X_limit_switch_left.setPressedState(HIGH);

  X_limit_switch_right.attach( X_LIMIT_SWITCH_RIGHT_PIN, INPUT );
  X_limit_switch_right.interval(5);
  X_limit_switch_right.setPressedState(HIGH);

  X_toggle_switch.attach( X_ENABLE_SWITCH_PIN, INPUT_PULLUP);
  X_toggle_switch.interval(5);
  X_toggle_switch.setPressedState(LOW);

  Y_toggle_switch.attach( Y_ENABLE_SWITCH_PIN, INPUT_PULLUP );
  Y_toggle_switch.interval(5);
  Y_toggle_switch.setPressedState(LOW);

  Y_limit_switch_front.attach( Y_LIMIT_SWITCH_FRONT_PIN, INPUT );
  Y_limit_switch_front.interval(5);
  Y_limit_switch_front.setPressedState(HIGH);

  Y_limit_switch_rear.attach( Y_LIMIT_SWITCH_REAR_PIN, INPUT );
  Y_limit_switch_rear.interval(5);
  Y_limit_switch_rear.setPressedState(HIGH);

  X_pot_EMA_S = analogRead(X_POTPIN);
  Y_pot_EMA_S = analogRead(Y_POTPIN);

  ////////////////////////
  // Steppers init code //
  ////////////////////////
  engine.init();
  Xstepper = engine.stepperConnectToPin(X_STEP_PIN);
  Xstepper->setDirectionPin(X_DIR_PIN);
  Xstepper->setEnablePin(X_ENA_PIN, false);
  Xstepper->setAutoEnable(false);
  Xstepper->setSpeedInHz(0);
  Xstepper->setAcceleration(X_ACCEL);

  Ystepper = engine.stepperConnectToPin(Y_STEP_PIN);
  Ystepper->setDirectionPin(Y_DIR_PIN);
  Ystepper->setEnablePin(Y_ENA_PIN, false);
  Ystepper->setAutoEnable(false);
  Ystepper->setSpeedInHz(Y_MAX_SPEED);
  Ystepper->setAcceleration(Y_ACCEL);

  //////////////
  // lcd init //
  //////////////
  lcd.init();
  lcd.clear();
  dispA.begin();

  for (int i = 0; i < 3; i++) {
    lcd.backlight();
    delay(250);
    lcd.noBacklight();
    delay(250);
  }
  lcd.backlight();
  lcd.home();
  lcd.setCursor(1, 0);
  lcd.print("X Axis:");
  lcd.setCursor(3, 1);
  lcd.print("OFF");
  lcd.setCursor(11, 0);
  lcd.print("Y Axis:");
  lcd.setCursor(13, 1);
  lcd.print("OFF");
  lcd.setCursor(0, 2);
  lcd.print("D: ");
  lcd.setCursor(3, 2);
  lcd.print("Right");
  lcd.setCursor(10, 2);
  lcd.print("D: ");
  lcd.setCursor(13, 2);
  lcd.print("Fwd");

  //////////////
  // Watchdog //
  //////////////
  wdt_enable(WDTO_250MS);
}


void loop()
{
  ///////////////////
  // switch update //
  ///////////////////
  Y_toggle_switch.update();
  X_toggle_switch.update();
  Y_limit_switch_front.update();
  Y_limit_switch_rear.update();
  X_limit_switch_left.update();
  X_limit_switch_right.update();

  /////////
  // LCD //
  /////////
  sprintf(buffer, "%3d%%", barX);
  lcd.setCursor(4, 3);
  lcd.print(buffer);
  sprintf(buffer, "%3d%%", barY);
  lcd.setCursor(14, 3);
  lcd.print(buffer);
  spb1.showProgressPct(barX);
  spb2.showProgressPct(barY);

  ///////////
  // X pot //
  ///////////
  X_pot_EMA_S = potEMA(X_POTPIN, X_pot_EMA_S);
  X_pot_map = map(X_pot_EMA_S, 0, 1019, 0, X_MAX_SPEED);
//  Serial.println(X_pot_map);
  barX = map(X_pot_EMA_S, 0, 1019, 0, 100);

  ///////////
  // Y pot //
  ///////////
  Y_pot_EMA_S = potEMA(Y_POTPIN, Y_pot_EMA_S);
  Y_pot_map = map(Y_pot_EMA_S, 0, 1019, 0, Y_MAX_STEP_OVER);
  barY = map(Y_pot_EMA_S, 0, 1019, 0, 100);
  if (Y_step_with_dir < 0) {
    Y_step_with_dir = int(Y_pot_map) * -1;
  } else {
    Y_step_with_dir = int(Y_pot_map);
  }

  //////////////
  // Y toggle //
  //////////////

  if ( Y_toggle_switch.isPressed() ) {
    Ystepper->enableOutputs();
    lcd.setCursor(13, 1);
    lcd.print("ON ");
  } else {
    Ystepper->disableOutputs();
    lcd.setCursor(13, 1);
    lcd.print("OFF");
  }

  //////////////
  // X toggle //
  //////////////
  // don't enable X unless the pot is at 0
  if ( X_toggle_switch.isPressed() && X_pot_EMA_S == 0 ) {
    Xstepper->enableOutputs();
    lcd.setCursor(3, 1);
    lcd.print("ON ");
  } else if (X_toggle_switch.isPressed() == false) {
    Xstepper->disableOutputs();
    lcd.setCursor(3, 1);
    lcd.print("OFF");
  }

  //////////////
  // Y Limit  //
  //////////////
  if (Y_limit_switch_front.pressed() && Y_limit_switch_front_trip == false) {
    Y_step_with_dir = Y_step_with_dir * -1;
    Y_limit_switch_front_trip = true;
    Y_limit_switch_rear_trip = false;
    lcd.setCursor(13, 2);
    lcd.print("Rev");
  }
  if (Y_limit_switch_rear.pressed() && Y_limit_switch_rear_trip == false) {

    Y_step_with_dir = Y_step_with_dir * -1;
    Y_limit_switch_rear_trip = true;
    Y_limit_switch_front_trip = false;
    lcd.setCursor(13, 2);
    lcd.print("Fwd");
  }

  /////////////
  // X limit //
  /////////////
  
  if (X_limit_switch_left.pressed() && X_limit_switch_right_trip == false) {
    if (X_toggle_switch.isPressed() && X_pot_EMA_S > 0) {
      Xstepper->runBackward();
    }
    X_dir_forward = true;
    X_limit_switch_left_trip = false;
    X_limit_switch_right_trip = true;
    lcd.setCursor(3, 2);
    lcd.print("Right");
    if (Y_toggle_switch.isPressed() && Y_pot_EMA_S > 0) {
      Ystepper->move(Y_step_with_dir);
    }
  }
  if (X_limit_switch_right.pressed() && X_limit_switch_left_trip == false)  {
    if (X_toggle_switch.isPressed() && X_pot_EMA_S > 0) {
      Xstepper->runForward();
    }
    X_dir_forward = false;
    X_limit_switch_right_trip = false;
    X_limit_switch_left_trip = true;
    lcd.setCursor(3, 2);
    lcd.print("Left ");
    if (Y_toggle_switch.isPressed() && Y_pot_EMA_S > 0) {
      Ystepper->move(Y_step_with_dir);
    }
  }


  /////////////////////
  // X Movement Stop //
  /////////////////////
  // this is needed otherwise the stepper moves after
  // the pot is turned up then down to zero again with
  // the toggle enabled.  I think there is a better
  // way to do this by combining it with the
  // X Movement Start and not have this code run each
  // cycle and constantly send Xstepper->stopMove()
  if (X_pot_EMA_S == 0 && startup == false) {
    Xstepper->stopMove();
    startup = true;
  } else if ( X_pot_EMA_S > 0 && startup == true) {
    startup = false;
  }

  //////////////////////
  // X Movement Start //
  //////////////////////

  if (X_toggle_switch.isPressed() && X_pot_EMA_S > 0) {
    Xstepper->setSpeedInHz(X_pot_map);
    switch (X_dir_forward) {
      case false:
        Xstepper->runForward();
        break;
      case true:
        Xstepper->runBackward();
        break;
    }
    startup = false;

  }

  wdt_reset();
  delay(1);

}


uint32_t potEMA(int sensorPin, uint32_t EMA_S) {
  float EMA_a = 0.3;
  int sensorValue = analogRead(sensorPin);
  EMA_S = (EMA_a * sensorValue) + ((1 - EMA_a) * EMA_S);
  return EMA_S;
}
