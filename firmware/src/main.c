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

/************************************************************************/
/* defines                                                              */
/************************************************************************/

// Acelera Ré PD30 AFEC
#define AFEC_POT AFEC0
#define AFEC_POT_ID ID_AFEC0
#define AFEC_POT_CHANNEL 0 

// Botão AZUL protoboard PD26
#define BUT1_PIO      PIOD
#define BUT1_PIO_ID   ID_PIOD
#define BUT1_IDX      26
#define BUT1_IDX_MASK (1 << BUT1_IDX)

// Botão VERDE protoboard PC19
#define BUT2_PIO      PIOC
#define BUT2_PIO_ID   ID_PIOC
#define BUT2_IDX      19
#define BUT2_IDX_MASK (1 << BUT2_IDX)

// Botão LIGADESLIGA protoboard PA2
#define BUTONOFF_PIO      PIOA
#define BUTONOFF_PIO_ID   ID_PIOA
#define BUTONOFF_IDX      2
#define BUTONOFF_IDX_MASK (1 << BUTONOFF_IDX)

#define END_OF_PCK	  'X'
#define WAIT_TIME	  100 / portTICK_PERIOD_MS	

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
/* RTOS                                                                 */
/************************************************************************/

// Filas 
QueueHandle_t xQueueKeyUp;
QueueHandle_t xQueueKeyDown;
QueueHandle_t xQueueAfec;
QueueHandle_t xQueuePot;

// Semaforosx
SemaphoreHandle_t xSemaphoreOnOff;

#define TASK_MAIN_STACK_SIZE            (4096/sizeof(portSTACK_TYPE))
#define TASK_MAIN_STACK_PRIORITY        (tskIDLE_PRIORITY)
#define TASK_POT_STACK_SIZE            (4096/sizeof(portSTACK_TYPE))
#define TASK_POT_STACK_PRIORITY        (tskIDLE_PRIORITY)

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
	if(pio_get(BUT1_PIO, PIO_INPUT, BUT1_IDX_MASK)){
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
	if(pio_get(BUT2_PIO, PIO_INPUT, BUT2_IDX_MASK)){
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
	afec_start_software_conversion(AFEC_POT);
}

static void AFEC_pot_Callback(void) {
	int value = afec_channel_get_value(AFEC_POT, AFEC_POT_CHANNEL);
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	xQueueSendFromISR(xQueueAfec, &value , &xHigherPriorityTaskWoken);
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/


void buts_config(void){
	
	// Inicializa PIO dos botões
	pmc_enable_periph_clk(BUTONOFF_PIO_ID);
	pmc_enable_periph_clk(BUT1_PIO_ID);
	pmc_enable_periph_clk(BUT2_PIO_ID);
	
	
	// Inicializando botoes como entrada , configurando debounce
	pio_configure(BUTONOFF_PIO, PIO_INPUT, BUTONOFF_IDX_MASK,  PIO_DEBOUNCE);
	pio_configure(BUT1_PIO, PIO_INPUT, BUT1_IDX_MASK,  PIO_DEBOUNCE);
	pio_configure(BUT2_PIO, PIO_INPUT, BUT2_IDX_MASK,  PIO_DEBOUNCE);
	
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

void io_init(void) {

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

static void config_AFEC_pot(Afec *afec, uint32_t afec_id, uint32_t afec_channel,
                            afec_callback_t callback) {
  /*************************************
   * Ativa e configura AFEC
   *************************************/
  /* Ativa AFEC - 0 */
  afec_enable(afec);

  /* struct de configuracao do AFEC */
  struct afec_config afec_cfg;

  /* Carrega parametros padrao */
  afec_get_config_defaults(&afec_cfg);

  /* Configura AFEC */
  afec_init(afec, &afec_cfg);

  /* Configura trigger por software */
  afec_set_trigger(afec, AFEC_TRIG_SW);

  /*** Configuracao específica do canal AFEC ***/
  struct afec_ch_config afec_ch_cfg;
  afec_ch_get_config_defaults(&afec_ch_cfg);
  afec_ch_cfg.gain = AFEC_GAINVALUE_0;
  afec_ch_set_config(afec, afec_channel, &afec_ch_cfg);

  /*
  * Calibracao:
  * Because the internal ADC offset is 0x200, it should cancel it and shift
  down to 0.
  */
  afec_channel_set_analog_offset(afec, afec_channel, 0x200);

  /***  Configura sensor de temperatura ***/
  struct afec_temp_sensor_config afec_temp_sensor_cfg;

  afec_temp_sensor_get_config_defaults(&afec_temp_sensor_cfg);
  afec_temp_sensor_set_config(afec, &afec_temp_sensor_cfg);

  /* configura IRQ */
  afec_set_callback(afec, afec_channel, callback, 1);
  NVIC_SetPriority(afec_id, 4);
  NVIC_EnableIRQ(afec_id);
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

void task_potenciometro(void){
	printf("Task potenciometro started \n");
	
	// configura ADC e TC para controlar a leitura
	config_AFEC_pot(AFEC_POT, AFEC_POT_ID, AFEC_POT_CHANNEL, AFEC_pot_Callback);
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
	
	char handshake = '0';
	char speed_state;
	
	int send = 0;

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
	
	while(1) {
		
		if(xQueueReceive(xQueuePot, &speed_state, 0)){
			printf("\n %c \n", speed_state);
			send_package('A', speed_state , '0');
		}
		
		if(xQueueReceive(xQueueKeyDown, &button, 0)){
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
		
		// HANDSHAKE
		if(xSemaphoreTake(xSemaphoreOnOff, 0)){
			
			if(handshake == '1'){
				handshake = '0';
				}else{
				handshake  = '1';
			}
			
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

	// Cria task
	xTaskCreate(task_main, "Main", TASK_MAIN_STACK_SIZE, NULL,	TASK_MAIN_STACK_PRIORITY, NULL);
	xTaskCreate(task_potenciometro, "Potenciometro", TASK_MAIN_STACK_SIZE, NULL, TASK_MAIN_STACK_PRIORITY, NULL);
	
	// Cria semáforos para verificar quao botão foi apertado:
	xSemaphoreOnOff = xSemaphoreCreateBinary();
	
	if (xSemaphoreOnOff == NULL)
		printf("falha em criar semáforo \n");
	
	// Cria fila
	xQueueKeyUp = xQueueCreate(100, sizeof(char));
	xQueueKeyDown = xQueueCreate(100, sizeof(char));
	xQueueAfec = xQueueCreate(100, sizeof(uint32_t));
	xQueuePot = xQueueCreate(100, sizeof(char));
	
	if (xQueueKeyUp == NULL || xQueueKeyDown == NULL || xQueueAfec == NULL || xQueuePot == NULL )
		printf("falha em criar fila \n");

	// Start the scheduler.
	vTaskStartScheduler();

	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
