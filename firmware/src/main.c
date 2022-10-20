/************************************************************************
* 5 semestre - Eng. da Computao - Insper
*
* 2021 - Exemplo com HC05 com RTOS
*
*/

#include <asf.h>
#include "conf_board.h"
#include <string.h>
#include <math.h>

#include "mpu6050.h"
#include "Fusion/Fusion.h"

/************************************************************************/
/* defines                                                              */
/************************************************************************/

// Acelera Ré PD30 AFEC
#define AFEC_POT AFEC0
#define AFEC_POT_ID ID_AFEC0
#define AFEC_POT_CHANNEL 0

// Botão AZUL protoboard PA2
#define BUT1_PIO      PIOA
#define BUT1_PIO_ID   ID_PIOA
#define BUT1_IDX      2
#define BUT1_IDX_MASK (1 << BUT1_IDX)

// Botão VERDE protoboard PC19
#define BUT2_PIO      PIOC
#define BUT2_PIO_ID   ID_PIOC
#define BUT2_IDX      19
#define BUT2_IDX_MASK (1 << BUT2_IDX)

// Botão LIGADESLIGA protoboard PD26
#define BUTONOFF_PIO      PIOD
#define BUTONOFF_PIO_ID   ID_PIOD
#define BUTONOFF_IDX      26
#define BUTONOFF_IDX_MASK (1 << BUTONOFF_IDX)

// LED de Conexão (Liga/Desliga) PA24
#define LED_PIO		PIOA
#define LED_PIO_ID	ID_PIOA
#define LED_IDX     24
#define LED_IDX_MASK (1 << LED_IDX)

// --------------- JOYSTICK ----------------------
// x do Joystick PB2
#define AFECx_POT AFEC0
#define AFECx_POT_ID ID_AFEC0
#define AFECx_POT_CHANNEL 5

// y do joystick PB3
#define AFECy_POT AFEC0
#define AFECy_POT_ID ID_AFEC0
#define AFECy_POT_CHANNEL 2

// BOTAO PC31
#define BUT_JOY_PIO    PIOC
#define BUT_JOY_PIO_ID ID_PIOC
#define BUT_JOY_IDX    31
#define BUT_JOY_IDX_MASK  (1 << BUT_JOY_IDX)

#define END_OF_PCK	  'X'
#define WAIT_TIME	  100 / portTICK_PERIOD_MS

typedef struct {
	char eixo_x;
	char eixo_y;
} joyData;

// usart (bluetooth ou serial)
// Descomente para enviar dados
// pela serial debug

#define DEBUG_SERIAL

#ifdef DEBUG_SERIAL
#define USART_COM USART1
#define USART_COM_ID ID_USART1
#else
#define USART_COM USART0
#define USART_COM_ID ID_USART0
#endif

/************************************************************************/
/* VAR globais                                                          */
/************************************************************************/
int16_t  raw_acc_x, raw_acc_y, raw_acc_z;
volatile uint8_t  raw_acc_xHigh, raw_acc_yHigh, raw_acc_zHigh;
volatile uint8_t  raw_acc_xLow,  raw_acc_yLow,  raw_acc_zLow;
float proc_acc_x, proc_acc_y, proc_acc_z;

int16_t  raw_gyr_x, raw_gyr_y, raw_gyr_z;
volatile uint8_t  raw_gyr_xHigh, raw_gyr_yHigh, raw_gyr_zHigh;
volatile uint8_t  raw_gyr_xLow,  raw_gyr_yLow,  raw_gyr_zLow;
float proc_gyr_x, proc_gyr_y, proc_gyr_z;

/************************************************************************/
/* RTOS                                                                 */
/************************************************************************/

// Filas
QueueHandle_t xQueueKeyUp;
QueueHandle_t xQueueKeyDown;
QueueHandle_t xQueueAfec;
QueueHandle_t xQueuePot;
QueueHandle_t xQueueIMU;

QueueHandle_t xQueueAfecX;
QueueHandle_t xQueueAfecY;
QueueHandle_t xQueueJoy;

// Semaforos
SemaphoreHandle_t xSemaphoreOnOff;

#define TASK_MAIN_STACK_SIZE            (4096/sizeof(portSTACK_TYPE))
#define TASK_MAIN_STACK_PRIORITY        (tskIDLE_PRIORITY)
#define TASK_POT_STACK_SIZE            (4096/sizeof(portSTACK_TYPE))
#define TASK_POT_STACK_PRIORITY        (tskIDLE_PRIORITY)
#define TASK_IMU_STACK_SIZE            (4096/sizeof(portSTACK_TYPE))
#define TASK_IMU_STACK_PRIORITY        (tskIDLE_PRIORITY)

/************************************************************************/
/* prototypes                                                           */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/************************************************************************/
/* RTOS application HOOK                                                */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {
	}
}

extern void vApplicationIdleHook(void) {
	pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
}

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

void but_joy_callback(void){
	char butId = 'Y';
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if(!pio_get(BUT_JOY_PIO, PIO_INPUT, BUT_JOY_IDX_MASK)){
		xQueueSendFromISR(xQueueKeyDown, &butId , &xHigherPriorityTaskWoken);
		}else{
		xQueueSendFromISR(xQueueKeyUp, &butId , &xHigherPriorityTaskWoken);
	}
}

void but_on_off_callback(void){
	// Indica que o botão onoff foi pressionado:
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphoreOnOff, xHigherPriorityTaskWoken);
}

void but1_callback(void)
{
	
	char butId = 'A';
	// Indica que o botão azul foi pressionado:
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	// Borda subida (Apertou):
	if(!pio_get(BUT1_PIO, PIO_INPUT, BUT1_IDX_MASK)){
		xQueueSendFromISR(xQueueKeyDown, &butId , &xHigherPriorityTaskWoken);
		}else{
		// Borda de descida (Soltou):
		xQueueSendFromISR(xQueueKeyUp, &butId , &xHigherPriorityTaskWoken);
	}
}

void but2_callback(void)
{
	char butId = 'B';
	// Indica que o botão verde foi pressionado:
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	// Borda subida (Apertou):
	if(!pio_get(BUT2_PIO, PIO_INPUT, BUT2_IDX_MASK)){
		xQueueSendFromISR(xQueueKeyDown, &butId , &xHigherPriorityTaskWoken);
		}else{
		// Borda de descida (Soltou):
		xQueueSendFromISR(xQueueKeyUp, &butId , &xHigherPriorityTaskWoken);
	}
}

void TC1_Handler(void) {
	volatile uint32_t ul_dummy;

	ul_dummy = tc_get_status(TC0, 1);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/* Selecina canal e inicializa conversão */
	afec_channel_enable(AFEC_POT, AFEC_POT_CHANNEL);
	
	// Adicionei:
	afec_channel_enable(AFEC_POT, AFECx_POT_CHANNEL);
	afec_channel_enable(AFEC_POT, AFECy_POT_CHANNEL);
	
	afec_start_software_conversion(AFEC_POT);
}

static void AFEC_pot_Callback(void) {
	int value = afec_channel_get_value(AFEC_POT, AFEC_POT_CHANNEL);
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	xQueueSendFromISR(xQueueAfec, &value , &xHigherPriorityTaskWoken);
	
	// x - JOYSTICK
	afec_channel_enable(AFECx_POT, AFECx_POT_CHANNEL);
	afec_start_software_conversion(AFECx_POT);
}

static void AFEC_xJoy_Callback(void) {
	int value = afec_channel_get_value(AFECx_POT, AFECx_POT_CHANNEL);
	
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	xQueueSendFromISR(xQueueAfecX, &value , &xHigherPriorityTaskWoken);
}

static void AFEC_yJoy_Callback (void){
	int value = afec_channel_get_value(AFECy_POT, AFECy_POT_CHANNEL);
	
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	xQueueSendFromISR(xQueueAfecY, &value , &xHigherPriorityTaskWoken);
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

void led_config(void){
	pmc_enable_periph_clk(LED_PIO_ID);
	pio_set_output(LED_PIO, LED_IDX_MASK, 0, 0, 0);
}

void buts_config(void){
	
	// Inicializa PIO dos botões
	pmc_enable_periph_clk(BUTONOFF_PIO_ID);
	pmc_enable_periph_clk(BUT1_PIO_ID);
	pmc_enable_periph_clk(BUT2_PIO_ID);
	
	// Inicializando botoes como entrada , configurando debounce
	pio_configure(BUTONOFF_PIO, PIO_INPUT, BUTONOFF_IDX_MASK,  PIO_PULLUP | PIO_DEBOUNCE);
	pio_configure(BUT1_PIO, PIO_INPUT, BUT1_IDX_MASK,  PIO_PULLUP | PIO_DEBOUNCE);
	pio_configure(BUT2_PIO, PIO_INPUT, BUT2_IDX_MASK,  PIO_PULLUP | PIO_DEBOUNCE);
	
	// Aplicando filtro debounce
	pio_set_debounce_filter(BUTONOFF_PIO, BUTONOFF_IDX_MASK, 80);
	pio_set_debounce_filter(BUT1_PIO, BUT1_IDX_MASK, 80);
	pio_set_debounce_filter(BUT2_PIO, BUT2_IDX_MASK, 80);
}


void but_interrupt_config(Pio *p_pio , uint32_t pio_id, const uint32_t ul_mask , uint32_t priority){
	
	// Ativa interrupção e limpa primeira IRQ gerada na ativacao
	pio_enable_interrupt(p_pio, ul_mask);
	pio_get_interrupt_status(p_pio);
	
	// Configura NVIC para receber interrupcoes do PIO dos botões com mesma prioridade.
	NVIC_EnableIRQ(pio_id);
	NVIC_SetPriority(pio_id, priority);
}

void but_vjoy_config(void){
	pmc_enable_periph_clk(BUT_JOY_PIO_ID);
	pio_configure(BUT_JOY_PIO, PIO_INPUT, BUT_JOY_IDX_MASK,  PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_JOY_PIO, BUT_JOY_IDX_MASK, 80);
	
	pio_handler_set(BUT_JOY_PIO,
	BUT_JOY_PIO_ID,
	BUT_JOY_IDX_MASK,
	PIO_IT_EDGE,
	but_joy_callback);
	but_interrupt_config(BUT_JOY_PIO, BUT_JOY_PIO_ID, BUT_JOY_IDX_MASK, 4);
}

void io_init(void) {
	
	led_config();
	buts_config();
	
	pio_handler_set(BUTONOFF_PIO,
	BUTONOFF_PIO_ID,
	BUTONOFF_IDX_MASK,
	PIO_IT_RISE_EDGE,
	but_on_off_callback);
	
	pio_handler_set(BUT1_PIO,
	BUT1_PIO_ID,
	BUT1_IDX_MASK,
	PIO_IT_EDGE,
	but1_callback);
	
	pio_handler_set(BUT2_PIO,
	BUT2_PIO_ID,
	BUT2_IDX_MASK,
	PIO_IT_EDGE,
	but2_callback);
	
	
	// Ativa interrupção nos botões:
	but_interrupt_config(BUTONOFF_PIO, BUTONOFF_PIO_ID, BUTONOFF_IDX_MASK, 4);
	but_interrupt_config(BUT1_PIO, BUT1_PIO_ID, BUT1_IDX_MASK, 4);
	but_interrupt_config(BUT2_PIO, BUT2_PIO_ID, BUT2_IDX_MASK, 4);
	
}

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		#if (defined CONF_UART_CHAR_LENGTH)
		.charlength = CONF_UART_CHAR_LENGTH,
		#endif
		.paritytype = CONF_UART_PARITY,
		#if (defined CONF_UART_STOP_BITS)
		.stopbits = CONF_UART_STOP_BITS,
		#endif
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	#if defined(__GNUC__)
	setbuf(stdout, NULL);
	#else
	/* Already the case in IAR's Normal DLIB default configuration: printf()
	* emits one character at a time.
	*/
	#endif
}

uint32_t usart_puts(uint8_t *pstring) {
	uint32_t i ;

	while(*(pstring + i))
	if(uart_is_tx_empty(USART_COM))
	usart_serial_putchar(USART_COM, *(pstring+i++));
}

void usart_put_string(Usart *usart, char str[]) {
	usart_serial_write_packet(usart, str, strlen(str));
}

int usart_get_string(Usart *usart, char buffer[], int bufferlen, uint timeout_ms) {
	uint timecounter = timeout_ms;
	uint32_t rx;
	uint32_t counter = 0;

	while( (timecounter > 0) && (counter < bufferlen - 1)) {
		if(usart_read(usart, &rx) == 0) {
			buffer[counter++] = rx;
		}
		else{
			timecounter--;
			vTaskDelay(1);
		}
	}
	buffer[counter] = 0x00;
	return counter;
}

void usart_send_command(Usart *usart, char buffer_rx[], int bufferlen,
char buffer_tx[], int timeout) {
	usart_put_string(usart, buffer_tx);
	usart_get_string(usart, buffer_rx, bufferlen, timeout);
}

void config_usart0(void) {
	sysclk_enable_peripheral_clock(ID_USART0);
	usart_serial_options_t config;
	config.baudrate = 9600;
	config.charlength = US_MR_CHRL_8_BIT;
	config.paritytype = US_MR_PAR_NO;
	config.stopbits = false;
	usart_serial_init(USART0, &config);
	usart_enable_tx(USART0);
	usart_enable_rx(USART0);

	// RX - PB0  TX - PB1
	pio_configure(PIOB, PIO_PERIPH_C, (1 << 0), PIO_DEFAULT);
	pio_configure(PIOB, PIO_PERIPH_C, (1 << 1), PIO_DEFAULT);
}

int hc05_init(void) {
	char buffer_rx[128];
	usart_send_command(USART_COM, buffer_rx, 1000, "AT", 100);
	vTaskDelay( 500 / portTICK_PERIOD_MS);
	usart_send_command(USART_COM, buffer_rx, 1000, "AT", 100);
	vTaskDelay( 500 / portTICK_PERIOD_MS);
	usart_send_command(USART_COM, buffer_rx, 1000, "AT+NAMEagoravai", 100);
	vTaskDelay( 500 / portTICK_PERIOD_MS);
	usart_send_command(USART_COM, buffer_rx, 1000, "AT", 100);
	vTaskDelay( 500 / portTICK_PERIOD_MS);
	usart_send_command(USART_COM, buffer_rx, 1000, "AT+PIN0000\n", 100);
}

static void config_AFEC_pot() {
	/*************************************
	* Ativa e configura AFEC
	*************************************/
	/* Ativa AFEC - 0 */
	afec_enable(AFEC_POT);

	/* struct de configuracao do AFEC */
	struct afec_config afec_cfg;

	/* Carrega parametros padrao */
	afec_get_config_defaults(&afec_cfg);

	/* Configura AFEC */
	afec_init(AFEC_POT, &afec_cfg);

	/* Configura trigger por software */
	afec_set_trigger(AFEC_POT, AFEC_TRIG_SW);

	/*** Configuracao específica do canal AFEC ***/
	struct afec_ch_config afec_ch_cfg;
	afec_ch_get_config_defaults(&afec_ch_cfg);
	afec_ch_cfg.gain = AFEC_GAINVALUE_0;
	afec_ch_set_config(AFEC_POT, AFEC_POT_CHANNEL, &afec_ch_cfg);
	afec_ch_set_config(AFECx_POT, AFECx_POT_CHANNEL, &afec_ch_cfg);
	afec_ch_set_config(AFECy_POT, AFECy_POT_CHANNEL, &afec_ch_cfg);
	
	/*
	* Calibracao:
	* Because the internal ADC offset is 0x200, it should cancel it and shift
	down to 0.
	*/
	afec_channel_set_analog_offset(AFEC_POT, AFEC_POT_CHANNEL, 0x200);
	afec_channel_set_analog_offset(AFECx_POT, AFECx_POT_CHANNEL, 0x200);
	afec_channel_set_analog_offset(AFECy_POT, AFECy_POT_CHANNEL, 0x200);

	/* configura IRQ */
	afec_set_callback(AFEC_POT, AFEC_POT_CHANNEL, AFEC_pot_Callback, 1);
	afec_set_callback(AFECx_POT, AFECx_POT_CHANNEL, AFEC_xJoy_Callback, 1);
	afec_set_callback(AFECy_POT, AFECy_POT_CHANNEL, AFEC_yJoy_Callback, 1);

	NVIC_SetPriority(ID_AFEC0, 4);
	NVIC_EnableIRQ(ID_AFEC0);
}

void TC_init(Tc *TC, int ID_TC, int TC_CHANNEL, int freq) {
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	pmc_enable_periph_clk(ID_TC);

	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	NVIC_SetPriority((IRQn_Type)ID_TC, 4);
	NVIC_EnableIRQ((IRQn_Type)ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);
}

static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

	uint16_t pllPreScale = (int) (((float) 32768) / freqPrescale);
	
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	if (rttIRQSource & RTT_MR_ALMIEN) {
		uint32_t ul_previous_time;
		ul_previous_time = rtt_read_timer_value(RTT);
		while (ul_previous_time == rtt_read_timer_value(RTT));
		rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);
	}

	/* config NVIC */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 4);
	NVIC_EnableIRQ(RTT_IRQn);

	/* Enable RTT interrupt */
	if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
	rtt_enable_interrupt(RTT, rttIRQSource);
	else
	rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
	
}


/*	
 *  \Brief: The function is used as I2C bus init
 */
void mcu6050_i2c_bus_init(void)
{
	twihs_options_t bno055_option;
	pmc_enable_periph_clk(TWIHS_MCU6050_ID);

	/* Configure the options of TWI driver */
	bno055_option.master_clk = sysclk_get_cpu_hz();
	bno055_option.speed      = 40000;
	twihs_master_init(TWIHS_MCU6050, &bno055_option);
}

/*	\Brief: The function is used as I2C bus write
 *	\Return : Status of the I2C write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
int8_t mcu6050_i2c_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
	int32_t ierror = 0x00;

	twihs_packet_t p_packet;
	p_packet.chip         = dev_addr;
	p_packet.addr[0]      = reg_addr;
	p_packet.addr_length  = 1;
	p_packet.buffer       = reg_data;
	p_packet.length       = cnt;
	
	ierror = twihs_master_write(TWIHS_MCU6050, &p_packet);

	return (int8_t)ierror;
}


 /*	\Brief: The function is used as I2C bus read
 *	\Return : Status of the I2C read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be read
 *	\param reg_data : This data read from the sensor, which is hold in an array
 *	\param cnt : The no of byte of data to be read
 */
int8_t mcu6050_i2c_bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
	int32_t ierror = 0x00;
	
	twihs_packet_t p_packet;
	p_packet.chip         = dev_addr;
	p_packet.addr[0]      = reg_addr;
	p_packet.addr_length  = 1;
	p_packet.buffer       = reg_data;
	p_packet.length       = cnt;
	
  // TODO: Algum problema no SPI faz com que devemos ler duas vezes o registrador para
  //       conseguirmos pegar o valor correto.
	ierror = twihs_master_read(TWIHS_MCU6050, &p_packet);
	ierror = twihs_master_read(TWIHS_MCU6050, &p_packet);

	return (int8_t)ierror;
}

void connect_led(char handshake){
	if(handshake == '1'){
		pio_set(LED_PIO,LED_IDX_MASK);
		}else{
		pio_clear(LED_PIO,LED_IDX_MASK);
	}
}

void send(char arg){
	while(!usart_is_tx_ready(USART_COM)) {
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
	usart_write(USART_COM, arg);
}

void send_package(char tipo, char id , char status ){
	
	// Envia tipo:
	send(tipo);
	
	// Envia id do botão:
	send(id);
	
	// Envia status do botão:
	send(status);
	
	// envia fim de pacote
	send(END_OF_PCK);
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/
void task_imu(void){
	printf("Task IMU started \n");
	
	
	/* buffer para recebimento de dados */
	uint8_t bufferRX[100];
	uint8_t bufferTX[100];
	
	uint8_t rtn;
	
	/* Inicializa funcao de delay */
	delay_init( sysclk_get_cpu_hz());

	/* Inicializa Função de fusão */
	FusionAhrs ahrs;
	FusionAhrsInitialise(&ahrs);
	
	/************************************************************************/
	/* MPU                                                                  */
	/************************************************************************/
	
	/* Inicializa i2c */
	printf("Inicializando bus i2c \n");
	mcu6050_i2c_bus_init();
	
	// Verifica MPU
	
	rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_WHO_AM_I, bufferRX, 1);
	
	
	if(rtn != TWIHS_SUCCESS){
		printf("[ERRO] [i2c] [read] \n");
	}
	
	// Por algum motivo a primeira leitura é errada.
	if(bufferRX[0] != 0x68){
		printf("[ERRO] [mcu] [Wrong device] [0x%2X] \n", bufferRX[0]);
	}
	
	// Set Clock source
	bufferTX[0] = MPU6050_CLOCK_PLL_XGYRO;
	rtn = mcu6050_i2c_bus_write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, bufferTX, 1);
	if(rtn != TWIHS_SUCCESS)
	printf("[ERRO] [i2c] [write] \n");

	RTT_init(1000, 0, 0);
	int tick_old = 0;
	
	// Variável de estado , envio de dados apenas se houver mudança de estado:
	uint32_t send = 0;
	char state;
	// 's' -> stop
	// 'r' -> right
	// 'l' -> left
	
	while (1) {
		// Configura range acelerometro para operar com 2G
		bufferTX[0] = 0x00; // 2G
		rtn = mcu6050_i2c_bus_write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, bufferTX, 1);
		
		// Le valor do acc X High e Low
		rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, &raw_acc_xHigh, 1);
		rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_L, &raw_acc_xLow,  1);
		
		// Le valor do acc y High e  Low
		rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_YOUT_H, &raw_acc_yHigh, 1);
		rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_ZOUT_L, &raw_acc_yLow,  1);
		
		// Le valor do acc z HIGH e Low
		rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_ZOUT_H, &raw_acc_zHigh, 1);
		rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_ZOUT_L, &raw_acc_zLow,  1);
		
		// Dados são do tipo complemento de dois
		raw_acc_x = (raw_acc_xHigh << 8) | (raw_acc_xLow << 0);
		raw_acc_y = (raw_acc_yHigh << 8) | (raw_acc_yLow << 0);
		raw_acc_z = (raw_acc_zHigh << 8) | (raw_acc_zLow << 0);
		proc_acc_x = (float)raw_acc_x/16384;
		proc_acc_y = (float)raw_acc_y/16384;
		proc_acc_z = (float)raw_acc_z/16384;
		
		// Configura range gyroscopio para operar com 250 °/s
		bufferTX[0] = 0x00; // 250 °/s
		rtn = mcu6050_i2c_bus_write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, bufferTX, 1);
		
		// Le valor do gyr X High e Low
		rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_XOUT_H, &raw_gyr_xHigh, 1);
		rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_XOUT_L, &raw_gyr_xLow,  1);
		
		// Le valor do gyr y High e  Low
		rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_YOUT_H, &raw_gyr_yHigh, 1);
		rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_ZOUT_L, &raw_gyr_yLow,  1);
		
		// Le valor do gyr z HIGH e Low
		rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_ZOUT_H, &raw_gyr_zHigh, 1);
		rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_ZOUT_L, &raw_gyr_zLow,  1);
		
		// Dados são do tipo complemento de dois
		raw_gyr_x = (raw_gyr_xHigh << 8) | (raw_gyr_xLow << 0);
		raw_gyr_y = (raw_gyr_yHigh << 8) | (raw_gyr_yLow << 0);
		raw_gyr_z = (raw_gyr_zHigh << 8) | (raw_gyr_zLow << 0);
		proc_gyr_x = (float)raw_gyr_x/131;
		proc_gyr_y = (float)raw_gyr_y/131;
		proc_gyr_z = (float)raw_gyr_z/131;
		
		// replace this with actual gyroscope data in degrees/s
		const FusionVector gyroscope = {proc_gyr_x, proc_gyr_y, proc_gyr_z};
		// replace this with actual accelerometer data in g
		const FusionVector accelerometer = {proc_acc_x, proc_acc_y, proc_acc_z};
		
		// calcula runtime do código acima para definir delta t do programa
		int tick = rtt_read_timer_value(RTT);
		float delta_time = (tick - tick_old)/1000.0;
		tick_old = tick;

		// aplica o algoritmo
		FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, delta_time);

		// dados em pitch roll e yaw
		const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
		
		// 		printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
		// 		printf("Roll: %f\n", euler.angle.roll);
		//
		
		if( euler.angle.roll > 30){
			
			if(state != 'r'){
				//printf("Direita!\n");
				send = 1;
				state = 'r';
			}
			
			}else if(euler.angle.roll < -30){
			
			if(state != 'l'){
				//printf("Esquerda!\n");
				send = 1;
				state = 'l';
			}
			
			}else {
			if(state != 's'){
				//printf("Horizontal!\n");
				send = 1;
				state = 's';
			}
		}
		

		if(send){
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			xQueueSend(xQueueIMU, &state,  &xHigherPriorityTaskWoken);
			send = 0;
		}
		
	}
	
}

void task_joystick(void){

	printf("\nTask joystick\n");
	
	but_vjoy_config();
	
	// Variaveis
	uint32_t send;
	uint32_t value;
	
	// Inicializa dados
	joyData data;
	data.eixo_x = '0';
	data.eixo_y  = '0';
	
	// 'd' -> seta para direita
	// 'e' -> seta para esquerda
	// 'c' -> seta para cima
	// 'b' -> seta para baixo
	// '0' -> Não mexeu
	
	while(1){
		if(xQueueReceive(xQueueAfecX , &(value) , 0)){
			
			if(log10(value) < 2.5){
				//printf("%d",value);
				if(data.eixo_x != 'e'){
					data.eixo_x = 'e';
					send = 1;
				}
				
				}else if (log10(value) > 3.5){
				if(data.eixo_x != 'd'){
					data.eixo_x = 'd';
					send = 1;
				}
				}else{
				if(data.eixo_x != '0'){
					data.eixo_x = '0';
					send = 1;
				}
			}
		}
		
		
		if(xQueueReceive(xQueueAfecY , &(value) , 0)){
			if(log10(value) < 2.5){
				
				if(data.eixo_y != 'c'){
					data.eixo_y = 'c';
					send = 1;
				}
				
				}else if (log10(value) > 3.5){
				if(data.eixo_y != 'b'){
					data.eixo_y = 'b';
					send = 1;
				}
				}else{
				if(data.eixo_y != '0'){
					data.eixo_y = '0';
					send = 1;
				}
			}
		}
		
		
		if(send){
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			xQueueSend(xQueueJoy, &data,  &xHigherPriorityTaskWoken);
			send = 0;
		}
		
	}
}

void task_potenciometro(void){
	printf("Task potenciometro started \n");
	
	// configura ADC e TC para controlar a leitura
	config_AFEC_pot();
	
	TC_init(TC0, ID_TC1, 1, 10);
	tc_start(TC0, 1);
	
	int value;
	
	// Variável de estado , envio de dados apenas se houver mudança de estado:
	uint32_t send = 0;
	char state;
	// 's' -> stop
	// 'd' -> decelerate
	// 'a' -> accelerate
	
	while(1){
		if (xQueueReceive(xQueueAfec, &(value), 80)) {
			
			if( log10(value) > 3.5){
				if(state != 'a'){
					send = 1;
					state = 'a';
				}
				
				}else if( log10(value) < 2.5){
				
				if(state != 'd'){
					send = 1;
					state = 'd';
				}
				
				}else {
				if(state != 's'){
					send = 1;
					state = 's';
				}
			}
			
		}
		
		if(send){
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			xQueueSend(xQueuePot, &state,  &xHigherPriorityTaskWoken);
			send = 0;
		}
		
	}
	
}

void task_main(void) {
	
	printf("Task main started \n");
	printf("Inicializando HC05 \n");
	config_usart0();
	hc05_init();

	// Configura  botões
	io_init();
	
	char tipo;
	char button;
	char button_ID;
	char status;
	joyData datajoy;
	
	char handshake = '0';
	char speed_state;
	char rotation_state;
	
	int send = 0;
	int firstTime = 1;

	//                PROTOCOLO
	// ----------------------------------------------
	// tipo de dado -> A (analogico) D (Digital)
	// id_botão     -> botão azul 'A'   botão verde 'B'
	// status botão -> '1' pressionado  '0' soltou
	
	//                HANDSHAKE
	// ---------------------------------------------
	// tipo = 'H'
	// id_botao = '0'
	// status = '0' (antes ligado) '1' (antes desigado)
	
	//                VJOY DATA
	// ---------------------------------------------
	// tipo = 'J'
	// id_botao = info eixo x
	// status = info eixo y
	
	while(1) {
		
		// DIGITAL
		if(xQueueReceive(xQueueKeyDown, &button, 0)){
			printf("ENTREI");
			tipo = 'D';
			button_ID = button;
			status = '1';
			send = 1;
		}
		
		if(xQueueReceive(xQueueKeyUp, &button, 0)){
			tipo = 'D';
			button_ID = button;
			status = '0';
			send = 1;
		}
		
		// ANALOGIC
		if(xQueueReceive(xQueuePot, &speed_state, 0)){
			if(handshake == '1'){
				send_package('A', speed_state , '0');
			}
		}
		
		// JOYSTICK
		if(xQueueReceive(xQueueJoy, &datajoy , 0)){
			if(handshake == '1'){
				send_package('J', datajoy.eixo_x, datajoy.eixo_y);
			}
		}
		
		//IMU
		if(xQueueReceive(xQueueIMU, &rotation_state, 0)){
			if(handshake == '1'){
				send_package('I', rotation_state , '0');
				//printf("Enviei rotação\n");
			}
		}	
		
		// HANDSHAKE
		if(xSemaphoreTake(xSemaphoreOnOff, 0)){
			
			if(handshake == '1'){
				handshake = '0';
				}else{
				handshake  = '1';
			}
			
			connect_led(handshake);
			send_package('H', '0', handshake);
		}
		
		if(handshake == '0'){
			send = 0;
		}
		
		// Envio de dados
		if(send && handshake){
			// Envia pacote:
			send_package(tipo, button_ID, status);
			// Variável de envio
			send = 0;
		}

	}
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/

int main(void) {
	
	// Inicializa sistema
	sysclk_init();
	board_init();
	configure_console();

	// Cria semáforos para verificar quao botão foi apertado:
	xSemaphoreOnOff = xSemaphoreCreateBinary();
	
	if (xSemaphoreOnOff == NULL)
	printf("falha em criar semáforo \n");
	
	// Cria fila
	xQueueKeyUp = xQueueCreate(100, sizeof(char));
	xQueueKeyDown = xQueueCreate(100, sizeof(char));
	xQueueAfec = xQueueCreate(100, sizeof(uint32_t));
	xQueuePot = xQueueCreate(100, sizeof(char));
	
	xQueueAfecX = xQueueCreate(100, sizeof(uint32_t));
	xQueueAfecY = xQueueCreate(100, sizeof(uint32_t));
	xQueueJoy = xQueueCreate(100, sizeof(joyData));
	xQueueIMU = xQueueCreate(100, sizeof(char));
	
	
	// Cria task
	xTaskCreate(task_main, "Main", TASK_MAIN_STACK_SIZE, NULL,	TASK_MAIN_STACK_PRIORITY, NULL);
	xTaskCreate(task_potenciometro, "Potenciometro", TASK_MAIN_STACK_SIZE, NULL, TASK_MAIN_STACK_PRIORITY, NULL);
	xTaskCreate(task_joystick , "Joystick" , TASK_MAIN_STACK_SIZE, NULL,  TASK_MAIN_STACK_PRIORITY , NULL);
	xTaskCreate(task_imu,"IMU", TASK_MAIN_STACK_SIZE, NULL, TASK_MAIN_STACK_PRIORITY, NULL);
	
	
	if (xQueueKeyUp == NULL || xQueueKeyDown == NULL || xQueueAfec == NULL || xQueuePot == NULL || xQueueAfecX == NULL || xQueueAfecY == NULL || xQueueJoy == NULL || xQueueIMU == NULL)
	printf("falha em criar fila \n");

	// Start the scheduler.
	vTaskStartScheduler();

	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
