/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/*
The sample initializes the ADC, gets and converts the ADC values to a battery voltage in a loop and prints them. 
The following code shows the initialization of the ADC by getting the ADC device handle from the Icarus device tree
and setting up the ADC channel

struct device* adc_dev = device_get_binding("ADC_0");
adc_channel_setup(adc_dev, &m_1st_channel_cfg);

with m_1st_channel_cfg being a structure of type struct adc_channel_cfg:

static const struct adc_channel_cfg m_1st_channel_cfg = {
  .gain = ADC_GAIN,
  .reference = ADC_REFERENCE,
  .acquisition_time = ADC_ACQUISITION_TIME,
  .channel_id = ADC_1ST_CHANNEL_ID,
  .input_positive = ADC_1ST_CHANNEL_INPUT,
};

In this sample, the ADC_REFERENCE is set to ADC_REF_INTERNAL which is 0.6V, ADC_GAIN is set up as 1/6 so the 
resulting input voltage range is: 0.6/(1/6) = 3.6V and for the Icarus the value is read from ADC_1ST_CHANNEL_INPUT 
which is actually NRF_SAADC_INPUT_AIN0 (Analog input pin 0)

To read the value from the ADC the sample calls:

adc_read(adc_dev, &sequence)
with sequence being a structure of type struct adc_sequence:

const struct adc_sequence sequence = {
  .channels = BIT(ADC_1ST_CHANNEL_ID),
  .buffer = m_sample_buffer,
  .buffer_size = sizeof(m_sample_buffer),
  .resolution = ADC_RESOLUTION,
};

This structure sets up the buffer where the values are stored and sets up the ADC read resolution ADC_RESOLUTION which is set to 10 bit in this sample (values from 0 to 1023).

After this call the buffer contains a sample value that can be used to compute the battery voltage value. 

*/

/*
B1 Temperature - Humidity bme280
B2 Temperature - Humidity - Movement bme280 - MH-SR602 MINI
B3 Temperature - Humidity - AirQuality bme680
B4 Door reed interruptor magnético
*/

//zephyr 2.20200

//#define LOG

#define B_1
//#define B_2
//#define B_3
//#define B_4

#if defined B_1 || defined B_2
 	#define BME280_CHIP true
#endif

#ifdef B_3
 	#define BME680_CHIP true
#endif

//#define CCS811_CHIP true

//#define Battery_Power true
#define Battery_Percent true

#include <zephyr/types.h>
#include <stddef.h>
#include <sys/printk.h>
#include <sys/util.h>
#include <drivers/sensor.h> 
#include <drivers/gpio.h>

#include <bluetooth/bluetooth.h>

#include <device.h>
#include <drivers/i2c.h>
#include <stdio.h>

#include <drivers/adc.h>
#include <hal/nrf_saadc.h>

#define INPUT_VOLT_RANGE 3.3f           // Volts
#define VALUE_RANGE_10_BIT 1.023        // (2^10 - 1) / 1000

#define ADC_DEV "ADC_0"

#define ADC_RESOLUTION 10
#define ADC_GAIN ADC_GAIN_1_6
#define ADC_REFERENCE ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 10)
#define ADC_1ST_CHANNEL_ID 0
#define ADC_1ST_CHANNEL_INPUT NRF_SAADC_INPUT_VDD

#define BUFFER_SIZE 1
static s16_t m_sample_buffer[BUFFER_SIZE];

static struct device *adc_dev;

float battery_voltage = 0;
u8_t  left_Battery_voltage;
u8_t  right_Battery_voltage;
u8_t  left_Percent_Battery_voltage;
u8_t  right_Percent_voltage;

static const struct adc_channel_cfg m_1st_channel_cfg = {
	.gain = ADC_GAIN,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	.channel_id = ADC_1ST_CHANNEL_ID,
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
	.input_positive = ADC_1ST_CHANNEL_INPUT,
#endif
};

static float get_battery_voltage(float *battery_voltage)
{
	int err;

	const struct adc_sequence sequence = {
		.channels = BIT(ADC_1ST_CHANNEL_ID),
		.buffer = m_sample_buffer,
		.buffer_size = sizeof(m_sample_buffer), // in bytes!
		.resolution = ADC_RESOLUTION,
	};

	if (!adc_dev) {
		return -1;
	}

	err = adc_read(adc_dev, &sequence);
	if (err) {
		#ifdef LOG
		printk("ADC read err: %d\n", err);
		#endif
		return err;
	}

	float sample_value = 0;
	for (int i = 0; i < BUFFER_SIZE; i++) {
		sample_value += (float) m_sample_buffer[i];
	}
	sample_value /= BUFFER_SIZE;

	*battery_voltage = ((float)(sample_value * (INPUT_VOLT_RANGE / VALUE_RANGE_10_BIT)))/1000.0;

	return 0;
}

bool init_adc()
{
	int err;

	adc_dev = device_get_binding(ADC_DEV);
	if (!adc_dev) {
		#ifdef LOG
		printk("Error getting " ADC_DEV " failed\n");
		#endif
		return false;
	}
	
	err = adc_channel_setup(adc_dev, &m_1st_channel_cfg);
	if (err) {
		#ifdef LOG
		printk("Error in adc setup: %d\n", err);
		#endif
		return false;
	}

	// Trigger offset calibration
	// As this generates a _DONE and _RESULT event
	// the first result will be incorrect.
	//NRF_SAADC_NS->TASKS_CALIBRATEOFFSET = 1;

	// discard incorrect sample
	float dummy = 0;
	get_battery_voltage(&dummy);
	return true;
}

struct device* port0;

#define PIN_LED 18 //PAD 8 NINA B112

#ifdef B_2
#define PIN     13 //PAD 4 NINA B112
#define PULL_UP true
#define EDGE    (GPIO_INT_EDGE | GPIO_INT_ACTIVE_LOW)
#endif

#ifdef B_4
#define DOOR_OUTPUT 29  //PAD 17 NINA B112
#define DOOR_INPUT  30  //PAD 18 NINA B112
uint8_t Door_State = 0;
#endif

#ifdef B_4
/*
 * @Door Close Detect
   The Algorith 
   will config the Gpio P0.9 to output
   will config the Gpio P0.10 to input
   write a LOW value on P0.9, delay and read back on P0.10
   write a HIGH value on P0.9,delay and read back on P0.10
   will config the Gpio P0.9 to input 
 */
void Door_Close_Detect()
{
  uint8_t Door_Sucessfully;

  u32_t val = 0;

  Door_Sucessfully = 0;

  gpio_pin_configure(port0, DOOR_OUTPUT, GPIO_DIR_OUT); 
  gpio_pin_configure(port0, DOOR_INPUT , GPIO_DIR_IN | GPIO_PUD_NORMAL);

  gpio_pin_write(port0, DOOR_OUTPUT, true);
  k_usleep(100);

  gpio_pin_read(port0,DOOR_INPUT,&val);
  
  if(val==true)
    Door_Sucessfully++;

  gpio_pin_write(port0, DOOR_OUTPUT, false);
  k_usleep(100);

  gpio_pin_read(port0,DOOR_INPUT,&val);
  
  if(val==false)
    Door_Sucessfully++;

  gpio_pin_configure(port0, DOOR_OUTPUT, GPIO_DIR_IN | GPIO_PUD_NORMAL | GPIO_DS_DISCONNECT_HIGH | GPIO_DS_DISCONNECT_LOW); 

  if(Door_Sucessfully==2)
	{
     Door_State = 1;
	 #ifdef LOG
	 printk("%s\n","Door Closed");
	 #endif
	}
 else
	{
     Door_State = 0;
	 #ifdef LOG
 	 printk("%s\n","Door open");
	 #endif
	}
} 
#endif

#ifdef B_2
u8_t Movement = 0;
u8_t Time_Movement = 0;
void button_pressed(struct device *port0, struct gpio_callback *cb, u32_t pins)
{
	#ifdef LOG
	printk("*****Movement detected*****\r");
	#endif
	Movement++;
}
#endif

#ifdef B_2
static struct gpio_callback gpio_cb;
#endif 

#ifdef B_1
static volatile u8_t mfg_data[] = { 0x59, 0x00, 0x00, 0x07, 0x00,0x08, 0x00, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00, 0xB1};
#endif
#ifdef B_2
static volatile u8_t mfg_data[] = { 0x59, 0x00, 0x00, 0x07, 0x00,0x08, 0x00, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00, 0xB2};
#endif
#ifdef B_3
static volatile u8_t mfg_data[] = { 0x59, 0x00, 0x00, 0x07, 0x00,0x08, 0x00, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00, 0xB3};
#endif
#ifdef B_4
static volatile u8_t mfg_data[] = { 0x59, 0x00, 0x00, 0x07, 0x00,0x08, 0x00, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00, 0xB4};
#endif

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_data, 20)
};

char g_localName[32];

static void bt_ready(int err)
{
	if (err) {
		return;
	}

sprintf(g_localName,"%s%x","INDRIYA-G-",NRF_FICR->DEVICEADDR[0]); 

/* Set Scan Response data */
	static struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, g_localName, 18),
	};

	/* Start advertising */
	err = bt_le_adv_start(BT_LE_ADV_NCONN, ad, ARRAY_SIZE(ad),
			      sd, ARRAY_SIZE(sd));
	if (err) {
		return;
	}
}

// sensor
#ifdef BME680_CHIP
static struct device* dev_bme680;
#endif

#ifdef BME280_CHIP
static struct device* dev_bme280;
#endif

#ifdef CCS811_CHIP
static struct device* dev_ccs811;
#endif

#ifdef BME680_CHIP
struct sensor_value temp, press, humidity, gas_res;
#endif

#ifdef BME280_CHIP
struct sensor_value temp, press, humidity;
#endif

#ifdef CCS811_CHIP
struct sensor_value co2, voc, voltage, current;
#endif

void Flash_LED()
{
	gpio_pin_configure(port0, PIN_LED, GPIO_DIR_OUT); //Led	  
	gpio_pin_write(port0, PIN_LED, 0);
	k_sleep(K_MSEC(100));
	gpio_pin_write(port0, PIN_LED, 1);
	k_sleep(K_MSEC(100));
	gpio_pin_write(port0, PIN_LED, 0);
	k_sleep(K_MSEC(1000));
	//Low Power
	gpio_pin_configure(port0, PIN_LED, GPIO_DIR_IN | GPIO_PUD_NORMAL | GPIO_DS_DISCONNECT_HIGH | GPIO_DS_DISCONNECT_LOW); 
}

void update_sensor_data()
{
#ifdef BME680_CHIP
	sensor_sample_fetch(dev_bme680);
	sensor_channel_get(dev_bme680, SENSOR_CHAN_AMBIENT_TEMP, &temp);
	sensor_channel_get(dev_bme680, SENSOR_CHAN_PRESS, &press);
	sensor_channel_get(dev_bme680, SENSOR_CHAN_HUMIDITY, &humidity);
	sensor_channel_get(dev_bme680, SENSOR_CHAN_GAS_RES, &gas_res);
	#ifdef LOG
	printk("T: %d.%06d; P: %d.%06d; H: %d.%06d; G: %d.%06d\n",
				temp.val1, temp.val2, press.val1, press.val2,
				humidity.val1, humidity.val2, gas_res.val1,
				gas_res.val2);
	#endif
#endif

#ifdef BME280_CHIP
	sensor_sample_fetch(dev_bme280);
	sensor_channel_get(dev_bme280, SENSOR_CHAN_AMBIENT_TEMP, &temp);
	sensor_channel_get(dev_bme280, SENSOR_CHAN_PRESS, &press);
	sensor_channel_get(dev_bme280, SENSOR_CHAN_HUMIDITY, &humidity);
	#ifdef LOG
   	printk("T: %d.%06d; P: %d.%06d; H: %d.%06d; G: %d.%06d\n",
				temp.val1, temp.val2, press.val1, press.val2,
				humidity.val1, humidity.val2);
	#endif
#endif
				
	get_battery_voltage(&battery_voltage);

	#ifdef LOG
	printf("Battery voltage: %fV\n", battery_voltage);

	printf("Battery voltage left: %dV\n", (u8_t) battery_voltage);
	printf("Battery voltage right: %dV\n", (u8_t) ((battery_voltage - (u8_t) battery_voltage)*100.0));

	printf("Battery Percent voltage left: %d\n", (u8_t) ((battery_voltage / INPUT_VOLT_RANGE) * 100.0)); 
	#endif

	if((u8_t) ((battery_voltage / INPUT_VOLT_RANGE) * 100.0)==100)
	{
		#ifdef LOG
		printf("Battery Percent voltage right: %d%s\n", 0,"%");
		#endif
	}
	else
	{
		#ifdef LOG
		printf("Battery Percent voltage right: %d%s\n", (u8_t) ((battery_voltage - (u8_t) battery_voltage) * 100.0),"%");
		#endif
	}
	//Put zero before UPDATE
	mfg_data[8] = 0; mfg_data[9] = 0; mfg_data[10] = 0; mfg_data[11] = 0; mfg_data[12] = 0; 
	mfg_data[13] = 0; mfg_data[14] = 0; mfg_data[15] = 0; mfg_data[18] = 0; mfg_data[7] = 0;

#ifdef BME280_CHIP
	mfg_data[8] = (uint8_t) temp.val1;
	mfg_data[9] = (uint8_t) temp.val2;	
	mfg_data[10] = (uint8_t) press.val1;
	mfg_data[11] = (uint8_t) press.val2;	
	mfg_data[12] = (uint8_t) humidity.val1;
	mfg_data[13] = (uint8_t) humidity.val2;	
	mfg_data[14] = (uint8_t) 0;  //NO GAS
	mfg_data[15] = (uint8_t) 0;  //NO GAS
#endif

#ifdef BME680_CHIP
	mfg_data[8] = (uint8_t) temp.val1;
	mfg_data[9] = (uint8_t) temp.val2;	
	mfg_data[10] = (uint8_t) press.val1;
	mfg_data[11] = (uint8_t) press.val2;	
	mfg_data[12] = (uint8_t) humidity.val1;
	mfg_data[13] = (uint8_t) humidity.val2;	
	mfg_data[14] = (uint8_t) gas_res.val1;
	mfg_data[15] = (uint8_t) gas_res.val2;	
#endif	

	#ifdef Battery_Power
	mfg_data[16] = (u8_t) battery_voltage;
	mfg_data[17] = (u8_t) ((battery_voltage - (u8_t) battery_voltage)*100.0);	
	#endif

	#ifdef Battery_Percent
	mfg_data[16] = (u8_t) ((battery_voltage / INPUT_VOLT_RANGE) * 100.0);
	if(mfg_data[16]==100)
		mfg_data[17] = 0;
	else
		mfg_data[17] = (u8_t) ((battery_voltage - (u8_t) battery_voltage) * 100.0);
	#endif

	#ifdef B_2
	mfg_data[18] = Movement;
	#endif

	#ifdef B_4
	Door_Close_Detect();
	//Door
	mfg_data[7] = Door_State;
	#endif

	// flash  LED
	Flash_LED();
}

/*
// print CSS811 data
void update_sensor_data()
{
	sensor_sample_fetch(dev_ccs811);
	sensor_channel_get(dev_ccs811, SENSOR_CHAN_CO2, &co2);
	sensor_channel_get(dev_ccs811, SENSOR_CHAN_VOC, &voc);
	sensor_channel_get(dev_ccs811, SENSOR_CHAN_VOLTAGE, &voltage);
	sensor_channel_get(dev_ccs811, SENSOR_CHAN_CURRENT, &current);

	get_battery_voltage(&battery_voltage);

	#ifdef LOG
	printk("\nCo2: %d.%06dppm; VOC: %d.%06dppb\n", co2.val1, co2.val2,
                voc.val1, voc.val2);
	printk("Voltage: %d.%06dV; Current: %d.%06dA\n", voltage.val1,
                voltage.val2, current.val1, current.val2);

	printf("Battery voltage: %fV\n", battery_voltage);

	printf("Battery voltage left: %dV\n", (u8_t) battery_voltage);
	printf("Battery voltage right: %dV\n", (u8_t) ((battery_voltage - (u8_t) battery_voltage)*100.0));

	printf("Battery Percent voltage left: %d\n", (u8_t) ((battery_voltage / INPUT_VOLT_RANGE) * 100.0)); 
	#endif
	if((u8_t) ((battery_voltage / INPUT_VOLT_RANGE) * 100.0)==100)
	{
		#ifdef LOG
		printf("Battery Percent voltage right: %d%s\n", 0,"%");
		#endif
	}
	else
	{
		#ifdef LOG
		printf("Battery Percent voltage right: %d%s\n", (u8_t) ((battery_voltage - (u8_t) battery_voltage) * 100.0),"%");
		#endif
	}
	mfg_data[12] = (uint8_t) co2.val1;
	mfg_data[13] = (uint8_t) co2.val2;	
	mfg_data[14] = (uint8_t) voc.val1;
	mfg_data[15] = (uint8_t) voc.val2;

	#ifdef Battery_Power
	mfg_data[16] = (u8_t) battery_voltage;
	mfg_data[17] = (u8_t) ((battery_voltage - (u8_t) battery_voltage)*100.0);	
	#endif

	#ifdef Battery_Percent
	mfg_data[16] = (u8_t) ((battery_voltage / INPUT_VOLT_RANGE) * 100.0);
	if(mfg_data[16]==100)
		mfg_data[17] = 0;
	else
		mfg_data[17] = (u8_t) ((battery_voltage - (u8_t) battery_voltage) * 100.0);
	#endif

	mfg_data[18] = Movement;

	Door_Close_Detect();
	//Door
	mfg_data[7] = Door_State;

	// flash  LED
	Flash_LED();
}
*/

void main(void)
{
	int err;
	
	init_adc();
	
	port0 = device_get_binding("GPIO_0");

	#ifdef B_2
	gpio_pin_configure(port0, PIN,  GPIO_DIR_IN | GPIO_INT |  PULL_UP | EDGE); //Presence Sensor
	gpio_init_callback(&gpio_cb, button_pressed, BIT(PIN));  
	gpio_add_callback(port0, &gpio_cb); //Call Back
	gpio_pin_enable_callback(port0, PIN);
	#endif

	/// flash  LED
	//Flash_LED();
	
#ifdef BME680_CHIP
	dev_bme680 = device_get_binding("BME680");
#endif

#ifdef BME280_CHIP
	dev_bme280 = device_get_binding("BME280");
#endif

#ifdef CCS811_CHIP
	dev_ccs811 = device_get_binding("CCS811");
#endif
	
	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(bt_ready);
	if (err) {
	}

	while(true)
	{
		update_sensor_data();
		bt_le_adv_update_data(ad, ARRAY_SIZE(ad), NULL, 0);
		k_sleep(4000);
		#ifdef B_2
		Time_Movement++; //each 20 seconds zero
		if(Time_Movement==5)
		{
			Time_Movement=0;
			Movement=0;
		}
		#endif
	}
}