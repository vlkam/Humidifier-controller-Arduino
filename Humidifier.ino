
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <Wire.h> 

#define WATERFLOWSENSOR_PUMP_PIN		2
#define DRAIN_VALVE_PIN					3

#define WATER_INLET_VALVE				5
#define WATERFLOWSENSOR_WATER_INLET_PIN	6
#define ONE_WIRE_BUS					7
#define WATER_LEVEL_HIGH_SENSOR_PIN		8
#define PUMP_PIN						9
#define HEATING_WATER_PIN				10
#define WATER_LEVEL_LOW_SENSOR_PIN		11

#define BTN_START_PIN					13
#define BTN_STOP_PIN					A1
#define BTN_DRAIN_PIN					12
#define BTN_MODE_PIN					A3
#define BTN_MODE_LEFT_PIN				A5
#define BTN_MODE_RIGHT_PIN				A2

#define TEMPERATURE_PRECISION	9
#define PUMP_STARTING_TIME      3000 // in milliseconds
#define DRAINING_PAUSE			(12 * 60 * 60 ) // in seconds
#define BUTTON_DELAY			100  // in milliseconds
#define LCD_OFF_DELAY			0  // 


enum Modes { Off, On, Error};
enum enStepsModeOn { Starting, StartingPump, PumpWorking, Draining,Stoping };
Modes MainMode;
enStepsModeOn StepsModeOn;


class LCD_display {
	LiquidCrystal_I2C *lcd; //(0x27, 16, 2);  // set the LCD address to 0x27 for a 16 chars and 2 line display
	uint32_t lastupdate = 0;

	public:

	LCD_display(){
		lcd = new LiquidCrystal_I2C(0x27, 16, 2);
	}

	void init(){
		lcd->init();
		lcd->backlight();
	}

	void loop(){
		if (LCD_OFF_DELAY > 0){
			uint32_t CurTime = millis();

			if ((CurTime - lastupdate) > (LCD_OFF_DELAY * 1000)){
				lcd->noBacklight();
			}
		}
	}

	void print(int x, int y, String str){
		String str1 = str + "                 .";
		lcd->setCursor(x, y);
		lcd->print(str1.substring(0,16));
		lcd->backlight();
		lastupdate = millis();
		Serial.println(str);
	}

	void print(int x, int y, char *str){
		print(x, y, String(str));
	}

	void print(char *str1, char *str2){
		print(0, 0, str1);
		print(0, 1, str2);
	}
};

LCD_display lcd_display;


class HardwareDevice {
protected:
	int pin;
public:
	bool debug;
protected:
	HardwareDevice(int pin_){
		debug = false;
		pin = pin_;
	}
};

class MyTimer {

	unsigned long LastCall;

public:

	unsigned long Pause;

	bool IsElapsed(){
		unsigned long CurrentMillis = millis();
		if (CurrentMillis < LastCall){
			LastCall = 0;
			return true;
		}
		if (CurrentMillis - LastCall > Pause){
			LastCall = CurrentMillis;
			return true;
		}
		return false;
	}

	MyTimer(unsigned long Pause_){
		LastCall = 0;
		Pause = Pause_;
	}
};

// Water flow sensor (WFS)
class WaterFlowSensor : public HardwareDevice {

private:
	int last_status;
	int impulses_counter;
	MyTimer *calculation_gap;

public:
	int impulses_per_second;
	bool isWorking;

	WaterFlowSensor(int pin_) : HardwareDevice(pin_){
		isWorking = false;
		pinMode(pin, INPUT);
		last_status = digitalRead(pin);

		calculation_gap = new MyTimer(1000);
		impulses_counter = 0;
		impulses_per_second = 100;

	}

	void loop(){
		
		//
		int current_status = digitalRead(pin);
		if (current_status != last_status){
			if (current_status == HIGH){
				impulses_counter++;
				isWorking = true;
			}
			last_status = current_status;
		}
		
		//
		if (calculation_gap->IsElapsed()){
			impulses_per_second = impulses_counter;
			impulses_counter = 0;
			
			isWorking = impulses_per_second > 0 ? true : false;

			//if (impulses_per_second > 0){
			//	Serial.println(String("WFS sensor is working!  ")+impulses_per_second);
			//}
		}

	}
};

WaterFlowSensor wfs_pump(WATERFLOWSENSOR_PUMP_PIN);
WaterFlowSensor wfs_water_inlet(WATERFLOWSENSOR_WATER_INLET_PIN);

class Pump : public HardwareDevice {
private:
	unsigned long last_time_counter;
public:
	bool isWorking;
	unsigned long time_start;
	unsigned long working_time_in_sec;
	unsigned long draining_time_interval_in_sec;

	void Start(){
		digitalWrite(pin,LOW);
		time_start = millis();
		isWorking = true;
	}

	void Stop(){
		digitalWrite(pin, HIGH);
		time_start = 0;
		isWorking = false;
	}

	void loop(){
		
		// working time count
		unsigned long current_millis = millis();
		if (isWorking && last_time_counter == 0) last_time_counter = current_millis;
		if (isWorking && current_millis - last_time_counter >= 1000){
			working_time_in_sec++;
			draining_time_interval_in_sec++;
			last_time_counter = current_millis;
		}
	}

	Pump(int pin_) : HardwareDevice(pin_){
		isWorking = false;
		pinMode(pin, OUTPUT);
		digitalWrite(pin, LOW);
		last_time_counter = 0;
		working_time_in_sec = 0;
		draining_time_interval_in_sec = 0;
	}
};

Pump MainPump(PUMP_PIN);

MyTimer WaterDrainageGap(4 * 60 * 60 * 1000); // 4 hours

class WaterLevelSensor : public HardwareDevice {
public:
	bool isEnabled;
	bool invert;

	WaterLevelSensor(int pin_,bool invert_) : HardwareDevice(pin_){
		pinMode(pin, INPUT);
		invert = invert_;
		loop();
	}

	void loop(){
		bool stat = digitalRead(pin) == HIGH ? true : false;
		if (invert){
			stat = stat == HIGH ? LOW : HIGH;
		}
		isEnabled = stat;
	}
};

WaterLevelSensor water_level_sensor_high(WATER_LEVEL_HIGH_SENSOR_PIN,true);
WaterLevelSensor water_level_sensor_low(WATER_LEVEL_LOW_SENSOR_PIN,false);

class Valve : public HardwareDevice {
public:
	bool isOpen;
	
	Valve(int pin_) : HardwareDevice(pin_){
		pinMode(pin, OUTPUT);
		isOpen = false;
		Close();
	}

	void Close(){
		digitalWrite(pin, HIGH);
		isOpen = false;
	}

	void Open(){
		digitalWrite(pin, LOW);
		isOpen = true;
	}

};

Valve DrainValve(DRAIN_VALVE_PIN);
Valve WaterInletValve(WATER_INLET_VALVE);

class TemperatureSensor {
	DallasTemperature *sensors;
	MyTimer *debug_timer;
	MyTimer *update_timer;
public:

	DeviceAddress deviceAddress;
	float value;

	TemperatureSensor(DallasTemperature* sensors_, DeviceAddress adr_){
		sensors = sensors_;
		for (int i = 0; i < 8; i++) deviceAddress[i] = adr_[i];
		sensors->setResolution(deviceAddress, TEMPERATURE_PRECISION);
		debug_timer = new MyTimer(1000);
		update_timer = new MyTimer(300);
		value = GetValue();
	}

private:
	float GetValue(){
		return sensors->getTempC(deviceAddress);
	}

public:
	void printAddress()
	{
		for (uint8_t i = 0; i < 8; i++)
		{
			if (deviceAddress[i] < 16) Serial.print("0");
			Serial.print(deviceAddress[i], HEX);
			if (i<7) Serial.print("-");
		}
	}

	void printTemperature()
	{
		float tempC = sensors->getTempC(deviceAddress);
		Serial.print("Temp C: ");
		Serial.print(tempC);
		Serial.println();
	}

	void loop(){
		if (debug_timer->IsElapsed()){
			//printTemperature();
		}
		if (update_timer->IsElapsed()){
			value = GetValue();
		}
	}

	bool IsEqivalent(DeviceAddress adr_){
		for (uint8_t i = 0; i < 8; i++){
			if (deviceAddress[i] != adr_[i]){
				return false;
			}
		}
		return true;
	}

};

TemperatureSensor *water_temperature_sensor;


class ArrayOfTemperatureSensors{
	MyTimer *timer;
	OneWire *oneWire;//(ONE_WIRE_BUS);
	public:

		DallasTemperature *sensors;// (&oneWire);

		ArrayOfTemperatureSensors(){
			timer = new MyTimer(500);
			oneWire = new OneWire(ONE_WIRE_BUS);
			sensors = new DallasTemperature(oneWire);
		}

		void loop(){
			if (timer->IsElapsed()){
				sensors->requestTemperatures();
			}
		}

		void Initialization(){
			
			// Temperature sensors section
			sensors->begin();
			sensors->requestTemperatures();

			DeviceAddress adr = { 0x28, 0x23, 0x96, 0x35, 0x05, 0x00, 0x00, 0x26 };
			water_temperature_sensor = new TemperatureSensor(sensors, adr);

			Serial.println("Locating Dallas devices...");
			Serial.println(String("Found ") + sensors->getDeviceCount() + " devices.");
			for (int i = 0; i < sensors->getDeviceCount(); i++){
				DeviceAddress adr;
				if (sensors->getAddress(adr, i)){

					if (water_temperature_sensor->IsEqivalent(adr)) continue;

					Serial.print("Found a new device with address ");
					TemperatureSensor *temp = new TemperatureSensor(sensors, adr);
					temp->printAddress();
					Serial.println();
				}
			}
			// report parasite power requirements
			Serial.print("Parasite power is: ");
			if (sensors->isParasitePowerMode()) Serial.println("ON");  else Serial.println("OFF");

			// Testing device
			Serial.print("Water temperature : ");
			water_temperature_sensor->printAddress();
			Serial.print(" temperature : ");
			Serial.println(water_temperature_sensor->value);
		}
};
ArrayOfTemperatureSensors sensors_array;


class PushButton : public HardwareDevice {
private:
	bool isPressedStart;
	int time_pressed;
public:
	bool IsPressed;

	PushButton(int pin_) : HardwareDevice(pin_){

		pinMode(pin, INPUT);

		isPressedStart = false;
		IsPressed = false;
	}

	void EventProcessed(){
		isPressedStart = false;
		IsPressed = false;
	}

	void loop(){

		bool status = digitalRead(pin);

		if (status == true){
			//Serial.print("pressed");
			if (isPressedStart == false){
				isPressedStart = true;
				time_pressed = millis();
			} 
		}
		else 
		{
			if (isPressedStart == true && millis() - time_pressed > BUTTON_DELAY){
				IsPressed = true;
				isPressedStart = false;
				if (debug){
					//Serial.println("Pressed button "+pin);
				}
				//Serial.println(String("Press key ") + pin + "   " + analogRead(pin));
			}
		}

	}

};

PushButton btnStart(BTN_START_PIN);
PushButton btnStop(BTN_STOP_PIN);
PushButton btnDrain(BTN_DRAIN_PIN);
PushButton btnMode(BTN_MODE_PIN);
PushButton btnLeft(BTN_MODE_LEFT_PIN);
PushButton btnRight(BTN_MODE_RIGHT_PIN);

class WaterHeater : public HardwareDevice {
public:
	bool isEnabled;
	WaterHeater(int pin_) : HardwareDevice(pin_){
		pinMode(pin, OUTPUT);
		Stop();
	}

	void loop(){

	}

	void Start(){
		digitalWrite(pin, LOW);
		isEnabled = true;
	}

	void Stop(){
		isEnabled = false;
		digitalWrite(pin, HIGH);
	}
};

WaterHeater water_heater(HEATING_WATER_PIN);

class Setting {
	int Flag;
public:
	bool HeatingWater;
	int TargetTempOfWater;

	Setting(){}

	void SetDefaultValues(){
		HeatingWater = false;
		TargetTempOfWater = 25;
	}

	void Save(){
		int addr = 0;
		EEPROM.put(addr,*this);
	}

	void Load(){
		int addr = 0;
		EEPROM.get(addr, *this);
		if (Flag != 0xFAFD){
			Serial.println("Save EEPROM");
			Flag = 0xFAFD;
			SetDefaultValues();
			Save();
		}

	}
};

Setting setting;

#define MAX_POSITIONS 3
class Menu {
	int8_t CurrentPosition;
public:
	Menu(){
		CurrentPosition = 0;
	}

	enum BtnLeftAndRigth {Left,Right};

	void loop(){
		switch (CurrentPosition){
		case 0:
			break;
		case 1:
			//ShowCurrentPosition();
			break;
		case 2:
			break;

		}
	}

	void SetNextPosition(){
		CurrentPosition++;
		if (CurrentPosition > MAX_POSITIONS - 1) CurrentPosition = 0;
		ShowCurrentPosition();
	}

	void ShowCurrentPosition(){
		switch (CurrentPosition){
		case 0:
			lcd_display.print(0, 1, String("Water temp: ") + String(water_temperature_sensor->value));
			break;
		case 1:
			lcd_display.print(0, 1, String("Heat water: ") + String(setting.HeatingWater ? "yes":"no"));
			break;
		case 2:
			lcd_display.print(0, 1, String("Targ wtr temp ") + String(setting.TargetTempOfWater));
			break;

		}
	}

	void EditCurrentPosition(BtnLeftAndRigth btn){
		switch (CurrentPosition){
		case 0:
			break;
		case 1:
			setting.HeatingWater = setting.HeatingWater ? false : true;
			setting.Save();
			break;
		case 2:
			if (btn == Left){
				setting.TargetTempOfWater--;
			}
			else {
				setting.TargetTempOfWater++;
			}
			setting.Save();
			break;

		}
		ShowCurrentPosition();
	}
};

Menu menu;

void setup()
{

	Serial.begin(9600);
  
	lcd_display.init();

	setting.Load();
	
	Serial.print("Target temperature : ");
	Serial.println(setting.TargetTempOfWater);

	btnStop.IsPressed = true;


	MainMode = Off;

	sensors_array.Initialization();

	//DeviceAddress adr = { 0x28, 0xAB, 0xF2, 0xEB, 0x03, 0x00, 0x00, 0xA6 };
	//water_temperature_sensor = new TemperatureSensor(sensors_array.sensors, adr);
	
	//setup_temperature_sensor();
}

void loop()
{
	
	lcd_display.loop();
	menu.loop();
	
	// Buttons
	btnStart.loop();
	btnStop.loop();
	btnDrain.loop();
	btnMode.loop();
	btnLeft.loop();
	btnRight.loop();

	if (btnStart.IsPressed){
		Serial.println("Pressed the Start button");
		btnStart.EventProcessed();
		MainMode = On;
		StepsModeOn = Starting;
		lcd_display.print("Starting...","");
	}

	if (btnStop.IsPressed){
		Serial.println("Pressed the Stop button");
		lcd_display.print("Humidifier ready","");
		MainMode = Off;
		btnStop.EventProcessed();
	}

	if (btnDrain.IsPressed){
		Serial.println("Pressed the Draining button");
		btnDrain.EventProcessed();
	}

	if (btnMode.IsPressed){
		menu.SetNextPosition();
		Serial.println("Pressed the Select mode button");
		btnMode.EventProcessed();
	}

	if (btnLeft.IsPressed){
		menu.EditCurrentPosition(menu.Left);
		Serial.println("Pressed the Mode left button");
		btnLeft.EventProcessed();
	}

	if (btnRight.IsPressed){
		menu.EditCurrentPosition(menu.Right);
		Serial.println("Pressed the Mode right button");
		btnRight.EventProcessed();
	}

	water_level_sensor_high.loop();
	water_level_sensor_low.loop();
	wfs_pump.loop();
	wfs_water_inlet.loop();
	water_heater.loop();
	MainPump.loop();
	
	sensors_array.loop();
	water_temperature_sensor->loop();

	// Closing the water inlet valve if the level of water is high
	if (water_level_sensor_high.isEnabled && WaterInletValve.isOpen){
		Serial.println("Full, water valve close");
		WaterInletValve.Close();
	}

	// self diagnostic  
	if (water_level_sensor_high.isEnabled && !water_level_sensor_low.isEnabled){
		lcd_display.print("Water level sens","error");
		MainMode == Off;
	}

	// water heater off
	if (water_heater.isEnabled && setting.TargetTempOfWater <= water_temperature_sensor->value) // temperature ok
	{
		Serial.println("Water heater off. Temperature target of water reached");
		water_heater.Stop();
	}
	if (water_heater.isEnabled && !water_level_sensor_high.isEnabled) // the water's level is low
	{
		Serial.println("Water heater off. The water level is low");
		water_heater.Stop();
	}
	if (water_heater.isEnabled && !setting.HeatingWater) // the water's level is low
	{
		Serial.println("Water heater off.");
		water_heater.Stop();
	}

	// water heater on
	if (MainMode == On && 
		setting.HeatingWater && 
		water_level_sensor_high.isEnabled && 
		water_level_sensor_low.isEnabled &&
		setting.TargetTempOfWater > water_temperature_sensor->value &&
		!water_heater.isEnabled)
	{
		Serial.println("Water heater on.");
		water_heater.Start();
	}

	// starting humidifier
	if (MainMode == On && StepsModeOn == Starting){
		if (water_level_sensor_high.isEnabled)
		{
			Serial.println("StartingPump");
			StepsModeOn = StartingPump;
			MainPump.Start();
		}
		else
		{
			if (!WaterInletValve.isOpen){
				WaterInletValve.Open();
				lcd_display.print(0, 1, "Fill the water");
			}
		}
	}

	// humidifier is starting
	if (MainMode == On && StepsModeOn == StartingPump){
		StepsModeOn = PumpWorking;
		lcd_display.print("Humidifier work", "");

		/*
		if (!MainPump.isWorking){
			MainPump.Start();
			lcd_display.print(0, 1, "Pump is stating");
		}
		else {
			if (millis() - MainPump.time_start > PUMP_STARTING_TIME){
				if (wfs_pump.isWorking){
					StepsModeOn = PumpWorking;
					lcd_display.print("Humidifier work","");
				}
				else {
					MainMode = Error;
					lcd_display.print("Error start pump","");
				}
			}
		}
		*/
	}

	// draining start
	if (MainMode == On && StepsModeOn == PumpWorking){
		if (MainPump.draining_time_interval_in_sec > DRAINING_PAUSE){
			StepsModeOn = Draining;
			DrainValve.Open();
			lcd_display.print(0, 1, "draining...");
			WaterInletValve.Close();
			MainPump.draining_time_interval_in_sec = 0;
		}
	}

	// draining in progress
	if (MainMode == On && StepsModeOn == Draining){
		
		if (!water_level_sensor_low.isEnabled){
			DrainValve.Close();
			StepsModeOn = PumpWorking;
		}
	}

	if (MainMode == Off){
		MainPump.Stop();
		DrainValve.Close();
		WaterInletValve.Close();
		water_heater.Stop();
	}

	// Closing the water inlet valve
	if (water_level_sensor_high.isEnabled && WaterInletValve.isOpen){
		WaterInletValve.Close();
	}

	// fill the water if the level of water is low
	if (MainMode == On && StepsModeOn == PumpWorking){
		if (!water_level_sensor_high.isEnabled && !WaterInletValve.isOpen){
			lcd_display.print(0, 1, "Fill the water");
			WaterInletValve.Open();
		}
	}

	// Test low water level
	if (MainMode == On && StepsModeOn == PumpWorking){
		if (!water_level_sensor_low.isEnabled && MainPump.isWorking){
			MainPump.Stop();
			lcd_display.print(0,1,"Water lvl to low!");
			Serial.println("Test water level:stop pump");
		}
		if (water_level_sensor_low.isEnabled && !MainPump.isWorking){
			MainPump.Start();
			lcd_display.print(0, 1, "");
			Serial.println("Test water level:start pump");
		}
	}
}
