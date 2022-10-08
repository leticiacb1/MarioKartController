/************************************************************************
* 5 semestre - Eng. da Computao - Insper
*
* 2021 - Exemplo com HC05 com RTOS
*
*/

#include <asf.h>
#include "conf_board.h"
#include <string.h>

/************************************************************************/
/* defines                                                              */
/************************************************************************/

// Botão AZUL protoboard PD30
#define BUT1_PIO      PIOD
#define BUT1_PIO_ID   ID_PIOD
#define BUT1_IDX      30
#define BUT1_IDX_MASK (1 << BUT1_IDX)

// Botão VERDE protoboard PA6
#define BUT2_PIO      PIOA
#define BUT2_PIO_ID   ID_PIOA
#define BUT2_IDX      6
#define BUT2_IDX_MASK (1 << BUT2_IDX)


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

// Semaforos
SemaphoreHandle_t xSemaphoreBlue;
SemaphoreHandle_t xSemaphoreGreen;

#define TASK_MAIN_STACK_SIZE            (4096/sizeof(portSTACK_TYPE))
#define TASK_MAIN_STACK_PRIORITY        (tskIDLE_PRIORITY)

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

void but1_callback(void)
{
	
	// Indica que o botão azul foi pressionado:
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphoreBlue, xHigherPriorityTaskWoken);

}

void but2_callback(void)
{
	
	// Indica que o botão verde foi pressionado:
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphoreGreen, xHigherPriorityTaskWoken);

}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/


void buts_config(void){
	
	// Inicializa PIO dos botões
	pmc_enable_periph_clk(BUT1_PIO_ID);
	pmc_enable_periph_clk(BUT2_PIO_ID);
	
	
	// Inicializando botoes como entrada , configurando debounce
	pio_configure(BUT1_PIO, PIO_INPUT, BUT1_IDX_MASK,  PIO_DEBOUNCE);
	pio_configure(BUT2_PIO, PIO_INPUT, BUT2_IDX_MASK,  PIO_DEBOUNCE);
	
	// Aplicando filtro debounce
	pio_set_debounce_filter(BUT1_PIO, BUT1_IDX_MASK, 60);
	pio_set_debounce_filter(BUT2_PIO, BUT2_IDX_MASK, 60);
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
	
	
	pio_handler_set(BUT1_PIO,
					BUT1_PIO_ID,
					BUT1_IDX_MASK,
					PIO_IT_RISE_EDGE,
					but1_callback);
	
	pio_handler_set(BUT2_PIO,
					BUT2_PIO_ID,
					BUT2_IDX_MASK,
					PIO_IT_RISE_EDGE,
					but2_callback);
	
	
	// Ativa interrupção nos botões:
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
	usart_send_command(USART_COM, buffer_rx, 1000, "AT+PIN0000", 100);
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

void task_main(void) {
	
	printf("Task main started \n");
	printf("Inicializando HC05 \n");
	config_usart0();
	hc05_init();

	// configura  botões
	io_init();

	// Espera (500ms) em ticks
	 TickType_t wait = 500 / portTICK_PERIOD_MS;
	char button1 = '0';
	char eof = 'X';

	// Task não deve retornar.
	while(1) {
		
		// Caso identificao interrupção:
		if (xSemaphoreTake(xSemaphoreBlue, wait)) {
			printf("\n APERTOU  AZUL \n");
			button1 = '1';
		}
		
		if (xSemaphoreTake(xSemaphoreGreen, wait)) {
			printf("\n APERTOU  VERDE \n");
			button1 = '1';
		}
		
		// envia status botão
		while(!usart_is_tx_ready(USART_COM)) {
			vTaskDelay(10 / portTICK_PERIOD_MS);			
		}
		
		usart_write(USART_COM, button1);
		
		// Atualiza status do botão:
		button1 = '0';
		
		// envia fim de pacote
		while(!usart_is_tx_ready(USART_COM)) {
			vTaskDelay(10 / portTICK_PERIOD_MS);
		}
		usart_write(USART_COM, eof);

		// dorme por 500 ms
		vTaskDelay(500 / portTICK_PERIOD_MS);
	}
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/

int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();

	configure_console();

	// Cria task
	xTaskCreate(task_main, "Main", TASK_MAIN_STACK_SIZE, NULL,	TASK_MAIN_STACK_PRIORITY, NULL);

	
	// Cria semáforos para verificar quao botão foi apertado:
	xSemaphoreBlue = xSemaphoreCreateBinary();
	xSemaphoreGreen = xSemaphoreCreateBinary();
	
	if (xSemaphoreBlue == NULL || xSemaphoreGreen == NULL)
		printf("falha em criar semáforo \n");


	// Start the scheduler.
	vTaskStartScheduler();

	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
