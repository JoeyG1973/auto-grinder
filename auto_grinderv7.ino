#include <FastAccelStepper.h>
#include <Bounce2.h>
#include <LiquidCrystal_I2C.h>
#include <SmoothProgress.h>
#define BAR_STYLES_IN_PROGMEM
#include <BarStyleV2.h>

#define X_LIMIT_SWITCH_LEFT_PIN 18
#define X_LIMIT_SWITCH_RIGHT_PIN 19
#define X_STEP_PIN 6
#define X_DIR_PIN 5
#define X_ENA_PIN 4
#define X_MAX_SPEED 7500 // halved from 15000
#define X_POTPIN A0
#define X_ENABLE_SWITCH_PIN 3
#define X_ACCEL 100000

#define Y_STEP_PIN 8
#define Y_DIR_PIN 9
#define Y_ENA_PIN 10
#define Y_MAX_SPEED 25000 // in hertz/steps
#define Y_POTPIN A4
#define Y_MAX_STEP_OVER 5000
#define Y_ENABLE_SWITCH_PIN 2
#define Y_LIMIT_SWITCH_FRONT_PIN 50
#define Y_LIMIT_SWITCH_REAR_PIN 52
#define Y_ACCEL 200000

// Debounced switches
Bounce2::Button X_toggle_switch = Bounce2::Button();
Bounce2::Button Y_toggle_switch = Bounce2::Button();
Bounce2::Button Y_limit_switch_front = Bounce2::Button();
Bounce2::Button Y_limit_switch_rear = Bounce2::Button();

// Steppers
FastAccelStepperEngine engine;
FastAccelStepper *Xstepper = nullptr;
FastAccelStepper *Ystepper = nullptr;

// LCD & progress bars
LiquidCrystal_I2C lcd(0x27, 20, 4);
LCD dispA(lcd, inPROGMEM(barStyleV2));
SmoothProgressBar spb1(dispA, 4, 9, 3, 0);
SmoothProgressBar spb2(dispA, 4, 19, 3, 1);

// State
volatile bool xLimitLeftHit = false;
volatile bool xLimitRightHit = false;
bool X_dir_forward = true;
uint32_t X_pot_EMA_S = 0;
uint32_t Y_pot_EMA_S = 0;
int barX = 0, barY = 0;
int32_t Y_step_with_dir = 0;
bool startup = true;

char buffer[16];

// Interrupt handlers
void onXLimitLeft()
{
  xLimitLeftHit = true;
}

void onXLimitRight()
{
  xLimitRightHit = true;
}

uint32_t potEMA(int sensorPin, uint32_t EMA_S)
{
  const float EMA_a = 0.3f;
  int sensorValue = analogRead(sensorPin);
  return (uint32_t)((EMA_a * sensorValue) + ((1 - EMA_a) * EMA_S));
}

void setup()
{
  // Debounce switches
  X_toggle_switch.attach(X_ENABLE_SWITCH_PIN, INPUT_PULLUP);
  X_toggle_switch.interval(5);
  X_toggle_switch.setPressedState(LOW);

  Y_toggle_switch.attach(Y_ENABLE_SWITCH_PIN, INPUT_PULLUP);
  Y_toggle_switch.interval(5);
  Y_toggle_switch.setPressedState(LOW);

  Y_limit_switch_front.attach(Y_LIMIT_SWITCH_FRONT_PIN, INPUT);
  Y_limit_switch_front.interval(5);
  Y_limit_switch_front.setPressedState(HIGH);

  Y_limit_switch_rear.attach(Y_LIMIT_SWITCH_REAR_PIN, INPUT);
  Y_limit_switch_rear.interval(5);
  Y_limit_switch_rear.setPressedState(HIGH);

  // Initial EMA
  X_pot_EMA_S = analogRead(X_POTPIN);
  Y_pot_EMA_S = analogRead(Y_POTPIN);

  // Steppers
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

  // LCD init
  lcd.init();
  lcd.clear();
  dispA.begin();
  for (int i = 0; i < 3; i++)
  {
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

  // Attach hardware interrupts
  attachInterrupt(digitalPinToInterrupt(X_LIMIT_SWITCH_LEFT_PIN), onXLimitLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(X_LIMIT_SWITCH_RIGHT_PIN), onXLimitRight, RISING);
}

void loop()
{
  // 1) Update inputs
  X_toggle_switch.update();
  Y_toggle_switch.update();
  Y_limit_switch_front.update();
  Y_limit_switch_rear.update();

  // 2) Priority: X limit hits
  if (xLimitLeftHit)
  {
    xLimitLeftHit = false;
    uint32_t speed = map(X_pot_EMA_S, 0, 1019, 0, X_MAX_SPEED);
    Xstepper->setSpeedInHz(speed);
    Xstepper->runBackward();
    X_dir_forward = true;
    lcd.setCursor(3, 2);
    lcd.print("Right");

    // Move Y stepper when X limit is hit
    if (Y_toggle_switch.isPressed()) // && Y_pot_EMA_S > 0)
    {
      Ystepper->move(Y_step_with_dir);
    }
    return;
  }
  if (xLimitRightHit)
  {
    xLimitRightHit = false;
    uint32_t speed = map(X_pot_EMA_S, 0, 1019, 0, X_MAX_SPEED);
    Xstepper->setSpeedInHz(speed);
    Xstepper->runForward();
    X_dir_forward = false;
    lcd.setCursor(3, 2);
    lcd.print("Left ");

    // Move Y stepper when X limit is hit
    if (Y_toggle_switch.isPressed()) // && Y_pot_EMA_S > 0)
    {
      Ystepper->move(Y_step_with_dir);
    }
    return;
  }

  // 3) Update LCD progress bars
  sprintf(buffer, "%3d%%", barX);
  lcd.setCursor(4, 3);
  lcd.print(buffer);
  sprintf(buffer, "%3d%%", barY);
  lcd.setCursor(14, 3);
  lcd.print(buffer);
  spb1.showProgressPct(barX);
  spb2.showProgressPct(barY);

  // 4) Read pots & map speeds
  X_pot_EMA_S = potEMA(X_POTPIN, X_pot_EMA_S);
  uint32_t X_pot_map = map(X_pot_EMA_S, 0, 1019, 0, X_MAX_SPEED);
  barX = map(X_pot_EMA_S, 0, 1019, 0, 100);

  Y_pot_EMA_S = potEMA(Y_POTPIN, Y_pot_EMA_S);
  uint32_t Y_pot_map = map(Y_pot_EMA_S, 0, 1019, 0, Y_MAX_STEP_OVER);
  barY = map(Y_pot_EMA_S, 0, 1019, 0, 100);
  Y_step_with_dir = (Y_step_with_dir < 0) ? -int(Y_pot_map) : int(Y_pot_map);

  // 5) Y-axis toggle & movement
  if (Y_toggle_switch.isPressed())
  {
    Ystepper->enableOutputs();
    lcd.setCursor(13, 1);
    lcd.print("ON ");
    if (Y_limit_switch_front.pressed())
    {
      Y_step_with_dir = -Y_step_with_dir;
      lcd.setCursor(13, 2);
      lcd.print("Rev");
    }
    if (Y_limit_switch_rear.pressed())
    {
      Y_step_with_dir = -Y_step_with_dir;
      lcd.setCursor(13, 2);
      lcd.print("Fwd");
    }
  }
  else
  {
    Ystepper->disableOutputs();
    lcd.setCursor(13, 1);
    lcd.print("OFF");
  }

  // 6) X-axis toggle & normal movement
  if (!X_toggle_switch.isPressed())
  {
    Xstepper->disableOutputs();
    lcd.setCursor(3, 1);
    lcd.print("OFF");
  }
  else if (X_pot_EMA_S == 0)
  {
    Xstepper->stopMove();
    startup = true;
  }
  else
  {
    lcd.setCursor(3, 1);
    lcd.print("ON ");
    Xstepper->enableOutputs();
    Xstepper->setSpeedInHz(X_pot_map);
    if (X_dir_forward)
      Xstepper->runBackward();
    else
      Xstepper->runForward();
    startup = false;
  }

  delay(1);
}
