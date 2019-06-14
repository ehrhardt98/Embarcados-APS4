/**
* \file
*
* \brief FreeRTOS configuration
*
* Copyright (c) 2012-2018 Microchip Technology Inc. and its subsidiaries.
*
* \asf_license_start
*
* \page License
*
* Subject to your compliance with these terms, you may use Microchip
* software and any derivatives exclusively with Microchip products.
* It is your responsibility to comply with third party license terms applicable
* to your use of third party software (including open source software) that
* may accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
* WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
* INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
* AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
* LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
* LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
* SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
* POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
* ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
* RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*
* \asf_license_stop
*
*/
/*
* Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
*/

/**
* \mainpage FreeRTOS Real Time Kernel example
*
* \section Purpose
*
* The FreeRTOS example will help users how to use FreeRTOS in SAM boards.
* This basic application shows how to create task and get information of
* created task.
*
* \section Requirements
*
* This package can be used with SAM boards.
*
* \section Description
*
* The demonstration program create two task, one is make LED on the board
* blink at a fixed rate, and another is monitor status of task.
*
* \section Usage
*
* -# Build the program and download it inside the evaluation board. Please
*    refer to the
*    <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6224.pdf">
*    SAM-BA User Guide</a>, the
*    <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6310.pdf">
*    GNU-Based Software Development</a>
*    application note or to the
*    <a href="ftp://ftp.iar.se/WWWfiles/arm/Guides/EWARM_UserGuide.ENU.pdf">
*    IAR EWARM User Guide</a>,
*    depending on your chosen solution.
* -# On the computer, open and configure a terminal application
*    (e.g. HyperTerminal on Microsoft Windows) with these settings:
*   - 115200 bauds
*   - 8 bits of data
*   - No parity
*   - 1 stop bit
*   - No flow control
* -# Start the application.
* -# LED should start blinking on the board. In the terminal window, the
*    following text should appear (values depend on the board and chip used):
*    \code
-- Freertos Example xxx --
-- xxxxxx-xx
-- Compiled: xxx xx xxxx xx:xx:xx --
\endcode
*
*/

#include "asf.h"
#include "conf_board.h"
#include "bme280.h"
#include <string.h>

#define TASK_MONITOR_STACK_SIZE            (2*2048/sizeof(portSTACK_TYPE))
#define TASK_MONITOR_STACK_PRIORITY        (tskIDLE_PRIORITY)

#define task_bme_STACK_SIZE                (8*1024/sizeof(portSTACK_TYPE))
#define task_bme_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define task_SD_STACK_SIZE				   (8*1024/sizeof(portSTACK_TYPE))
#define task_SD_STACK_PRIORITY			   (tskIDLE_PRIORITY)

#define task_UART_STACK_SIZE               (8*1024/sizeof(portSTACK_TYPE))
#define task_UART_STACK_PRIORITY           (tskIDLE_PRIORITY)

/**
* LEDs
*/
#define LED_PIO_ID		  ID_PIOC
#define LED_PIO         PIOC
#define LED_PIN		      8
#define LED_PIN_MASK    (1<<LED_PIN)

/**
* Botão
*/
#define BUT_PIO_ID            ID_PIOA
#define BUT_PIO               PIOA
#define BUT_PIN		            11
#define BUT_PIN_MASK          (1 << BUT_PIN)
#define BUT_DEBOUNCING_VALUE  79

#define TWIHS_MCU6050_ID    ID_TWIHS0
#define TWIHS_MCU6050       TWIHS0

QueueHandle_t xQueueTemp;
QueueHandle_t xQueuePress;
QueueHandle_t xQueueHumid;
QueueHandle_t xQueueSD_temp;
QueueHandle_t xQueueSD_press;
QueueHandle_t xQueueSD_humid;
SemaphoreHandle_t xSemaphore_erroTemp;
SemaphoreHandle_t xSemaphore_erroPress;
SemaphoreHandle_t xSemaphore_erroHumid;

float rangePerDigit ; // 2G
//const float rangePerDigit = 9.80665f ; // 2G

volatile uint8_t flag_led0 = 1;

int16_t  accX, accY, accZ;
volatile uint8_t  accXHigh, accYHigh, accZHigh;
volatile uint8_t  accXLow,  accYLow,  accZLow;

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);
//static void sd_mmc_test_unit_ready(int);
Ctrl_status sd_mmc_test_unit_ready(uint8_t);



void BUT_init(void);
void LED_init(int estado);
void pin_toggle(Pio *pio, uint32_t mask);


static void Button1_Handler(uint32_t id, uint32_t mask)
{
	pin_toggle(PIOD, (1<<28));
	pin_toggle(LED_PIO, LED_PIN_MASK);
}


void pin_toggle(Pio *pio, uint32_t mask){
	if(pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio,mask);
}


void BUT_init(void){
	/* config. pino botao em modo de entrada */
	pmc_enable_periph_clk(BUT_PIO_ID);
	pio_set_input(BUT_PIO, BUT_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	
	/* config. interrupcao em borda de descida no botao do kit */
	/* indica funcao (but_Handler) a ser chamada quando houver uma interrupção */
	pio_enable_interrupt(BUT_PIO, BUT_PIN_MASK);
	pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIN_MASK, PIO_IT_FALL_EDGE, Button1_Handler);
	
	/* habilita interrupçcão do PIO que controla o botao */
	/* e configura sua prioridade                        */
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 1);
};


void LED_init(int estado){
	pmc_enable_periph_clk(LED_PIO_ID);
	pio_set_output(LED_PIO, LED_PIN_MASK, estado, 0, 0 );
};

static void configure_console(void)
{

	/* Configura USART1 Pinos */
	sysclk_enable_peripheral_clock(ID_PIOB);
	sysclk_enable_peripheral_clock(ID_PIOA);
	pio_set_peripheral(PIOB, PIO_PERIPH_D, PIO_PB4);  // RX
	pio_set_peripheral(PIOA, PIO_PERIPH_A, PIO_PA21); // TX
	MATRIX->CCFG_SYSIO |= CCFG_SYSIO_SYSIO4;
	
	const usart_serial_options_t uart_serial_options = {
		.baudrate   = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits   = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
}

/************************************************************************/
/* BME                                                                  */
/************************************************************************/
/*
*  \Brief: The function is used as I2C bus init
*/
void bme280_i2c_bus_init(void)
{
	twihs_options_t bno055_option;
	pmc_enable_periph_clk(TWIHS_MCU6050_ID);

	/* Configure the options of TWI driver */
	bno055_option.master_clk = sysclk_get_cpu_hz();
	bno055_option.speed      = 10000;
	twihs_master_init(TWIHS_MCU6050, &bno055_option);
}

uint8_t bme280_i2c_read_reg(uint CHIP_ADDRESS, uint reg_address, char *value){
	uint i = 1;
	
	twihs_packet_t p_packet;
	p_packet.chip         = CHIP_ADDRESS;//BME280_ADDRESS;
	p_packet.addr_length  = 0;

	char data = reg_address; //BME280_CHIP_ID_REG;
	p_packet.buffer       = &data;
	p_packet.length       = 1;
	
	if(twihs_master_write(TWIHS_MCU6050, &p_packet) != TWIHS_SUCCESS)
	return 1;

	p_packet.addr_length  = 0;
	p_packet.length       = 1;
	p_packet.buffer       = value;

	if(twihs_master_read(TWIHS_MCU6050, &p_packet) != TWIHS_SUCCESS)
	return 1;
	
	return 0;
}

int8_t bme280_i2c_config_temp(void){
	int32_t ierror = 0x00;
	
	twihs_packet_t p_packet;
	p_packet.chip         = BME280_ADDRESS;//BME280_ADDRESS;
	p_packet.addr[0]      = BME280_CTRL_MEAS_REG;
	p_packet.addr_length  = 1;

	char data = 0b00100111; //BME280_CHIP_ID_REG;
	p_packet.buffer       = &data;
	p_packet.length       = 1;
	
	if(twihs_master_write(TWIHS_MCU6050, &p_packet) != TWIHS_SUCCESS)
	return 1;
}

int8_t bme280_i2c_read_temp(uint *temp)
{
	int32_t ierror = 0x00;
	char tmp[3];
	
	bme280_i2c_read_reg(BME280_ADDRESS, BME280_TEMPERATURE_MSB_REG, &tmp[2]);
	bme280_i2c_read_reg(BME280_ADDRESS, BME280_TEMPERATURE_MSB_REG, &tmp[2]);
	
	bme280_i2c_read_reg(BME280_ADDRESS, BME280_TEMPERATURE_LSB_REG, &tmp[1]);
	bme280_i2c_read_reg(BME280_ADDRESS, BME280_TEMPERATURE_LSB_REG, &tmp[1]);

	*temp = tmp[2] << 8 | tmp[1];
	return 0;
}

int8_t bme280_i2c_read_press(uint *press)
{
	int32_t ierror = 0x00;
	char tmp[3];
	
	bme280_i2c_read_reg(BME280_ADDRESS, BME280_PRESSURE_MSB_REG, &tmp[2]);
	bme280_i2c_read_reg(BME280_ADDRESS, BME280_PRESSURE_MSB_REG, &tmp[2]);
	
	bme280_i2c_read_reg(BME280_ADDRESS, BME280_PRESSURE_LSB_REG, &tmp[1]);
	bme280_i2c_read_reg(BME280_ADDRESS, BME280_PRESSURE_LSB_REG, &tmp[1]);

	*press = tmp[2] << 8 | tmp[1];
	return 0;
}

int8_t bme280_i2c_read_humid(uint *umi)
{
	int32_t ierror = 0x00;
	char tmp[3];
	
	bme280_i2c_read_reg(BME280_ADDRESS, BME280_HUMIDITY_MSB_REG, &tmp[2]);
	bme280_i2c_read_reg(BME280_ADDRESS, BME280_HUMIDITY_MSB_REG, &tmp[2]);
	
	bme280_i2c_read_reg(BME280_ADDRESS, BME280_HUMIDITY_LSB_REG, &tmp[1]);
	bme280_i2c_read_reg(BME280_ADDRESS, BME280_HUMIDITY_LSB_REG, &tmp[1]);

	*umi = tmp[2] << 8 | tmp[1];
	return 0;
}

uint8_t bme280_validate_id(void){
	char id;
	bme280_i2c_read_reg(BME280_ADDRESS, BME280_CHIP_ID_REG, &id );
	if (bme280_i2c_read_reg(BME280_ADDRESS, BME280_CHIP_ID_REG, &id ))
	return 1;
	if (id != 0x60)
	return 1;
	return 0;
}

int8_t bme280_i2c_read_compensation_T(ushort *temp, uint reg1, uint reg2)
{
	
	int32_t ierror = 0x00;
	char tmp[3];
	
	bme280_i2c_read_reg(BME280_ADDRESS, reg2, &tmp[2]);
	bme280_i2c_read_reg(BME280_ADDRESS, reg2, &tmp[2]);
	
	bme280_i2c_read_reg(BME280_ADDRESS, reg1, &tmp[1]);
	bme280_i2c_read_reg(BME280_ADDRESS, reg1, &tmp[1]);

	*temp = tmp[2] << 8 | tmp[1];
	return 0;
}

int8_t bme280_i2c_read_compensation_P(ushort *press, uint reg1, uint reg2)
{
	
	int32_t ierror = 0x00;
	char tmp[3];
	
	bme280_i2c_read_reg(BME280_ADDRESS, reg2, &tmp[2]);
	bme280_i2c_read_reg(BME280_ADDRESS, reg2, &tmp[2]);
	
	bme280_i2c_read_reg(BME280_ADDRESS, reg1, &tmp[1]);
	bme280_i2c_read_reg(BME280_ADDRESS, reg1, &tmp[1]);

	*press = tmp[2] << 8 | tmp[1];
	return 0;
}

int8_t bme280_i2c_read_compensation_H(ushort *umi, uint reg1, uint reg2)
{
	
	int32_t ierror = 0x00;
	char tmp[3];
	
	bme280_i2c_read_reg(BME280_ADDRESS, reg2, &tmp[2]);
	bme280_i2c_read_reg(BME280_ADDRESS, reg2, &tmp[2]);
	
	bme280_i2c_read_reg(BME280_ADDRESS, reg1, &tmp[1]);
	bme280_i2c_read_reg(BME280_ADDRESS, reg1, &tmp[1]);

	*umi = tmp[2] << 8 | tmp[1];
	return 0;
}

int8_t bme280_i2c_read_compensation_H_lower(uint8_t *umi, uint reg)
{
	
	int32_t ierror = 0x00;
	uint8_t out;
	
	bme280_i2c_read_reg(BME280_ADDRESS, reg, &out);
	bme280_i2c_read_reg(BME280_ADDRESS, reg, &out);
	
	*umi = out;
	return 0;
}

int32_t t_fine;
int32_t BME280_compensate_T_int32(int32_t adc_T, ushort dig_T1, short dig_T2, short dig_T3)
{
	int32_t var1, var2, T;
	var1 = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1))) >> 12) *
	((int32_t)dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}

uint32_t BME280_compensate_P_int64(int32_t adc_P, ushort dig_P1, short dig_P2, short dig_P3, short dig_P4, short dig_P5, short dig_P6, short dig_P7, short dig_P8, short dig_P9)
{
	int64_t var1, var2, p;
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)dig_P6;
	var2 = var2 + ((var1*(int64_t)dig_P5)<<17);
	var2 = var2 + (((int64_t)dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)dig_P3)>>8) + ((var1 * (int64_t)dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)dig_P1)>>33;
	if (var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}
	p = 1048576-adc_P;
	p = (((p<<31)-var2)*3125)/var1;
	var1 = (((int64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((int64_t)dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7)<<4);
	return (uint32_t)p;
}

uint32_t bme280_compensate_H_int32(int32_t adc_H, uint8_t dig_H1, short dig_H2, uint8_t dig_H3, short dig_H4, short dig_H5, int8_t dig_H6)
{
	int32_t v_x1_u32r;
	v_x1_u32r = (t_fine - ((int32_t)76800));
	v_x1_u32r = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * v_x1_u32r)) +
	((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)dig_H6)) >> 10) * (((v_x1_u32r *
	((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
	((int32_t)dig_H2) + 8192) >> 14));
	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
	v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
	v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
	return (uint32_t)(v_x1_u32r>>12);
}

/**
* \brief Called if stack overflow during execution
*/
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
signed char *pcTaskName)
{
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	* identify which task has overflowed its stack.
	*/
	for (;;) {
	}
}

/**
* \brief This function is called by FreeRTOS idle task
*/
extern void vApplicationIdleHook(void)
{
}

/**
* \brief This function is called by FreeRTOS each tick
*/
extern void vApplicationTickHook(void)
{
}

extern void vApplicationMallocFailedHook(void)
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}

/**
* \brief This task, when activated, send every ten seconds on debug UART
* the whole report of free heap and total tasks status
*/

/**
* \brief This task, when activated, make LED blink at a fixed rate
*/
static void task_bme(void *pvParameters)
{
	xQueueTemp = xQueueCreate( 10, sizeof( int ) );
	xQueuePress = xQueueCreate( 10, sizeof( int ) );
	xQueueHumid = xQueueCreate( 10, sizeof( int ) );
	xQueueSD_temp = xQueueCreate( 10, sizeof( int ) );
	xQueueSD_press = xQueueCreate( 10, sizeof( int ) );
	xQueueSD_humid = xQueueCreate( 10, sizeof( int ) );
	xSemaphore_erroTemp = xSemaphoreCreateBinary();
	xSemaphore_erroPress = xSemaphoreCreateBinary();
	xSemaphore_erroHumid = xSemaphoreCreateBinary();
	LED_init(1);
	BUT_init();
	printf("bme280 init\n");
	bme280_i2c_bus_init();
	//vTaskDelay(10/portTICK_PERIOD_MS);
	
	int32_t dig_T1, dig_T2, dig_T3;
	int32_t dig_P1, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
	int32_t dig_H1, dig_H2, dig_H3, dig_H4, dig_H5, dig_H6;
	/* verificando presenca do chip */
	while(bme280_validate_id()){
		printf("Chip nao encontrado\n");
		vTaskDelay(200/portTICK_PERIOD_MS);
	}
	printf("Chip encontrado, inicializando sensores \n");
	bme280_i2c_config_temp();
	
	bme280_i2c_read_compensation_T(&dig_T1, BME280_DIG_T1_LSB_REG, BME280_DIG_T1_MSB_REG);
	bme280_i2c_read_compensation_T(&dig_T2, BME280_DIG_T2_LSB_REG, BME280_DIG_T2_MSB_REG);
	bme280_i2c_read_compensation_T(&dig_T3, BME280_DIG_T3_LSB_REG, BME280_DIG_T3_MSB_REG);
	
	bme280_i2c_read_compensation_P(&dig_P1, BME280_DIG_P1_LSB_REG, BME280_DIG_P1_MSB_REG);
	bme280_i2c_read_compensation_P(&dig_P2, BME280_DIG_P2_LSB_REG, BME280_DIG_P2_MSB_REG);
	bme280_i2c_read_compensation_P(&dig_P3, BME280_DIG_P3_LSB_REG, BME280_DIG_P3_MSB_REG);
	bme280_i2c_read_compensation_P(&dig_P4, BME280_DIG_P4_LSB_REG, BME280_DIG_P4_MSB_REG);
	bme280_i2c_read_compensation_P(&dig_P5, BME280_DIG_P5_LSB_REG, BME280_DIG_P5_MSB_REG);
	bme280_i2c_read_compensation_P(&dig_P6, BME280_DIG_P6_LSB_REG, BME280_DIG_P6_MSB_REG);
	bme280_i2c_read_compensation_P(&dig_P7, BME280_DIG_P7_LSB_REG, BME280_DIG_P7_MSB_REG);
	bme280_i2c_read_compensation_P(&dig_P8, BME280_DIG_P8_LSB_REG, BME280_DIG_P8_MSB_REG);
	bme280_i2c_read_compensation_P(&dig_P9, BME280_DIG_P9_LSB_REG, BME280_DIG_P9_MSB_REG);
	
	bme280_i2c_read_compensation_H_lower(&dig_H1, BME280_DIG_H1_REG);
	bme280_i2c_read_compensation_H(&dig_H2, BME280_DIG_H2_LSB_REG, BME280_DIG_H2_MSB_REG);
	bme280_i2c_read_compensation_H_lower(&dig_H3, BME280_DIG_H3_REG);
	bme280_i2c_read_compensation_H(&dig_H4, BME280_DIG_H4_LSB_REG, BME280_DIG_H4_MSB_REG);
	

	
	uint temp, press, humid;
	UNUSED(pvParameters);
	for (;;) {
		if (bme280_i2c_read_temp(&temp))
		//		printf("erro readinG temperature \n");
		xSemaphoreGive(xSemaphore_erroTemp);
		else {
			temp = BME280_compensate_T_int32((int32_t)temp << 4, dig_T1, dig_T2, dig_T3)/100;
			xQueueSend(xQueueTemp, &temp, NULL);
			xQueueSend(xQueueSD_temp, &temp, NULL);
			// 			printf("Temperatura: %d \n", temp);
			// 			printf("Temp conv :%d\n", BME280_compensate_T_int32((int32_t)temp << 4, dig_T1, dig_T2, dig_T3));
		}
		
		if (bme280_i2c_read_press(&press))
		xSemaphoreGive(xSemaphore_erroPress);
		else {
			press = BME280_compensate_P_int64((int32_t)press << 4, dig_P1, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9)/256;
			xQueueSend(xQueuePress, &press, NULL);
			xQueueSend(xQueueSD_press, &press, NULL);
		}
		
		if (bme280_i2c_read_humid(&humid))
		xSemaphoreGive(xSemaphore_erroHumid);
		else {
			humid = bme280_compensate_H_int32((int32_t)humid << 4, dig_H1, dig_H2, dig_H3, dig_H4, dig_H5, dig_H6)/1024;
			xQueueSend(xQueueHumid, &humid, NULL);
			xQueueSend(xQueueSD_humid, &humid, NULL);
		}
		
		#if SAM4CM
		LED_Toggle(LED4);
		#else
		LED_Toggle(LED0);
		#endif
		vTaskDelay(1000/portTICK_PERIOD_MS);
	}
}

void task_SD(void){

	char test_file_name[] = "0:sd_mmc_test.txt";
	char analog_file_name[] = "0:analog.txt";

	Ctrl_status status;
	FRESULT res;
	FATFS fs;
	FIL file_object;
	
	int temp = 0;
	int press = 0;
	int humid = 0;
	
	
	/* Initialize SD MMC stack */
	sd_mmc_init();

	printf("\x0C\n\r-- SD/MMC/SDIO Card Example on FatFs --\n\r");
	printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);
	

	while(true){
		
		printf("Please plug an SD, MMC or SDIO card in slot.\n\r");

		/* Wait card present and ready */
		do {
			status = sd_mmc_test_unit_ready(0);
			if (CTRL_FAIL == status) {
				printf("Card install FAIL\n\r");
				printf("Please unplug and re-plug the card.\n\r");
				while (CTRL_NO_PRESENT != sd_mmc_check(0)) {
				}
			}
		} while (CTRL_GOOD != status);

		printf("Mount disk (f_mount)...\r\n");
		memset(&fs, 0, sizeof(FATFS));
		res = f_mount(LUN_ID_SD_MMC_0_MEM, &fs);
		if (FR_INVALID_DRIVE == res) {
			printf("[FAIL] res %d\r\n", res);
			}else{
			printf("[Mounting SDCARD Successful]\r\n");
			break;
		}
	}
	while (1) {
		
		printf("Opening/Creating a file (f_open)...\r\n");
		analog_file_name[0] = LUN_ID_SD_MMC_0_MEM + '0';
		res = f_open(&file_object,
		(char const *)analog_file_name,
		FA_OPEN_EXISTING | FA_WRITE);
		if (res != FR_OK) {
			//printf("[FAIL] res %d\r\n", res);
		}

		if (  xQueueReceive( xQueueSD_temp, &(temp), (TickType_t) 10 / portTICK_PERIOD_MS)  ) {
			f_lseek( &file_object, file_object.fsize);
			
			if (f_printf(&file_object, temp) == 0) {
				f_close(&file_object);
				//printf("FALHOU");
			}
			//printf("DEU BOM -> temp");
			f_close(&file_object);
			
			vTaskDelay(1000/portTICK_PERIOD_MS);
		}
		
		if (xQueueReceive(xQueueSD_press, &(press), (TickType_t) 10 / portTICK_PERIOD_MS)) {
			f_lseek( &file_object, file_object.fsize);
			
			if (f_printf(&file_object, press) == 0) {
				f_close(&file_object);
				//printf("FALHOU");
			}
			//printf("DEU BOM -> press");
			f_close(&file_object);
			
			vTaskDelay(1000/portTICK_PERIOD_MS);
		}
		
		if (xQueueReceive(xQueueSD_humid, &(humid), (TickType_t) 10 / portTICK_PERIOD_MS)) {
			f_lseek( &file_object, file_object.fsize);
			
			if (f_printf(&file_object, humid) == 0) {
				f_close(&file_object);
				//printf("FALHOU\r\n");
			}
			//printf("DEU BOM -> humid");
			f_close(&file_object);
			
			vTaskDelay(1000/portTICK_PERIOD_MS);
		}
	}
}

void task_UART(void) {
	int temp = 0;
	int press = 0;
	int humid = 0;
	
	while(1) {
		
		if (xSemaphoreTake(xSemaphore_erroTemp, (TickType_t) 10 / portTICK_PERIOD_MS)) {
			printf("Erro na leitura de temperatura");
		}
		if (xSemaphoreTake(xSemaphore_erroPress, (TickType_t) 10 / portTICK_PERIOD_MS)) {
			printf("Erro na leitura de pressao");
		}
		if (xSemaphoreTake(xSemaphore_erroHumid, (TickType_t) 10 / portTICK_PERIOD_MS)) {
			printf("Erro na leitura de humidade");
		}
		
		if (xQueueReceive(xQueueTemp, &(temp), (TickType_t) 10 / portTICK_PERIOD_MS)) {
			printf("Temperatura: %d °C\n", temp);
		}
		
		if (xQueueReceive(xQueuePress, &(press), (TickType_t) 10 / portTICK_PERIOD_MS)) {
			printf("Pressão: %d Pa\n", press);
		}
		
		if (xQueueReceive(xQueueHumid, &(humid), (TickType_t) 10 / portTICK_PERIOD_MS)) {
			printf("Humidade: %d \n", humid);
		}
	}
}

/**
*  \brief FreeRTOS Real Time Kernel example entry point.
*
*  \return Unused (ANSI-C compatibility).
*/
int main(void)
{
	/* Initialize the SAM system */
	sysclk_init();
	board_init();

	/* Initialize the console uart */
	configure_console();
	
	/* Output demo information. */
	printf("-- Freertos Example --\n\r");
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);

	/* Create task to make led blink */
	if (xTaskCreate(task_bme, "Bme280", task_bme_STACK_SIZE, NULL,
	task_bme_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test bme task\r\n");
	}
	
	if (xTaskCreate(task_UART, "UART", task_UART_STACK_SIZE, NULL,
	task_UART_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test UART task\r\n");
	}
// 	
// 	if (xTaskCreate(task_SD, "SD", task_SD_STACK_SIZE, NULL,
// 	task_SD_STACK_PRIORITY, NULL) != pdPASS) {
// 		printf("Failed to create test SD task\r\n");
// 	}

	/* Start the scheduler. */
	vTaskStartScheduler();
	
	while(1) {
		
	}
}
