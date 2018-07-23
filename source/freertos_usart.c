
/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_flexcomm.h"
#include "board.h"

#include "fsl_usart_freertos.h"
#include "fsl_usart.h"

#include "pin_mux.h"
#include <stdbool.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_USART USART0
#define DEMO_USART2 USART4

#define DEMO_USART_IRQHandler FLEXCOMM0_IRQHandler
#define DEMO_USART_IRQHandler2 FLEXCOMM4_IRQHandler

#define DEMO_USART_IRQn FLEXCOMM0_IRQn
#define DEMO_USART_IRQn2 FLEXCOMM4_IRQn
/* Task priorities. */
#define uart_task_PRIORITY (configMAX_PRIORITIES - 1)
#define USART_NVIC_PRIO 5
#define USART_NVIC_PRIO2 5

/*******************************************************************************
 * State of the satellite
 ******************************************************************************/
#define SAFE_MODE 1
#define NOMINAL_MODE 2
#define SCIENCE_MODE 3
#define DOWNLINK_MODE 4
#define UPDATE_MODE 5
/*******************************************************************************
 * Subsystem definitions
 ******************************************************************************/
#define EPS 1
#define ADC 2
#define PLD 3
#define COM 4
#define OBC 5
/*******************************************************************************
 * Telemetry Packet priorities
 ******************************************************************************/
#define PRIORITY_LOW 1
#define PRIORITY_MEDIUM 2
#define PRIORITY_HIGH 3
/*******************************************************************************
 * Status vector of the satellite
 ******************************************************************************/
struct {
	float voltages[4];
	float aocs_state[4];
	float temperatures[4];
} bammsat_state_vector;

/*******************************************************************************
 * Task Prototypes
 ******************************************************************************/
static void uart_task(void *pvParameters);
static void uart_task2(void *pvParameters);
static void vReceiverTask( void *pvParameters);
static void vMasterLoop(void *pvParameters);
/*******************************************************************************
 * Code
 ******************************************************************************/
const char *to_send1 = "FreeRTOS USART reply 1!\r\n";
const char *to_send2 = "FreeRTOS USART reply 2!\r\n";

const char *send_buffer_overrun = "\r\nRing buffer overrun!\r\n";
uint8_t background_buffer[40];
uint8_t recv_buffer[20];

uint8_t background_buffer2[40];
uint8_t recv_buffer2[20];

usart_rtos_handle_t handle;
struct _usart_handle t_handle;

usart_rtos_handle_t handle2;
struct _usart_handle t_handle2;

struct rtos_usart_config usart_config = {
    .baudrate = 115200,
    .parity = kUSART_ParityDisabled,
    .stopbits = kUSART_OneStopBit,
    .buffer = background_buffer,
    .buffer_size = sizeof(background_buffer),
};

struct rtos_usart_config usart_config2 = {
    .baudrate = 115200,
    .parity = kUSART_ParityDisabled,
    .stopbits = kUSART_OneStopBit,
    .buffer = background_buffer2,
    .buffer_size = sizeof(background_buffer2),
};

typedef struct {
	uint8_t subsystem_id;
	uint8_t type;
	uint8_t priority;
	uint8_t checksum;//or timestamp
	union {
		float voltages[4]; //EPS
		float aocs_vector[4]; // ADC
		uint8_t data[16]; //PLD or COM
	};
} data_packet;

QueueHandle_t data_queue;

/*!
 * @brief Application entry point.
 */
int main(void)
{
	/* Init board hardware. */

    // attach 12 MHz clock to FLEXCOMM0 (debug console)
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);
    // attach 12 MHz clock to UART4
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM4);
    BOARD_InitPins();
    BOARD_InitDebugConsole();
    RESET_PeripheralReset(kFC4_RST_SHIFT_RSTn);

    /* create queue reader task */
     data_queue = xQueueCreate(32, sizeof(recv_buffer));

    /* create UART tasks */
    if (xTaskCreate(uart_task, "Uart_task", configMINIMAL_STACK_SIZE + 100, NULL, uart_task_PRIORITY, NULL) != pdPASS)
    {
        PRINTF("USART 1 Task creation failed!.\r\n");
        while (1)
            ;
    }
    if (xTaskCreate(uart_task2, "Uart_task2", configMINIMAL_STACK_SIZE + 100, NULL, uart_task_PRIORITY, NULL) != pdPASS)
    {
        PRINTF("USART 2 Task creation failed!.\r\n");
        while (1)
            ;
     }

    if (xTaskCreate(vReceiverTask, "QueueReader", configMINIMAL_STACK_SIZE + 100, NULL, uart_task_PRIORITY, NULL) != pdPASS)
    {
        PRINTF("Queue Reader Task creation failed!.\r\n");
        while (1)
               ;
     }

    if (xTaskCreate(vMasterLoop, "MasterLoop", configMINIMAL_STACK_SIZE + 100, NULL, uart_task_PRIORITY, NULL) != pdPASS)
    {
    	PRINTF("Master Loop Task creation failed!.\r\n");
        while (1)
              ;
     }

    /* start task scheduler */
    vTaskStartScheduler();
    for (;;)
        ;
}

 void process_packet(data_packet packet)
{
	uint8_t i=0;
	switch (packet.subsystem_id)
	{
		case EPS:
			PRINTF("Subsystem: EPS\r\n");
			PRINTF("Packet type: %u\r\n",packet.type);
			//print packet data
			switch (packet.type)
			{
				case 1:
					PRINTF("Data packet: ");
					for(i=0;i<4;i++)
					{
						bammsat_state_vector.voltages[i] = packet.voltages[i];
						PRINTF("%3.2f ",packet.voltages[i]);
					}
					PRINTF("\r\n");
					break;
				default:
					break;
			}
			break;
		default:
			break;
	}
}

 void init_bammsat_state()
 {
	 uint8_t i;
	 for (i=0;i<4;i++)
	 {
		 bammsat_state_vector.voltages[i]=0.0;
		 bammsat_state_vector.aocs_state[i]=0.0;
		 bammsat_state_vector.temperatures[i]=0.0;
	 }
 }

static void vMasterLoop(void *pvParameters)
{
	uint8_t mode=1;
	float voltage_threshold = 5.0;
	//uint8_t next_mode;
	const TickType_t xDelay1000ms = pdMS_TO_TICKS( 10000 );
	init_bammsat_state();

	for( ;; )
	{
		switch (mode)
		{
			case SAFE_MODE:
				PRINTF("I am in the safe mode!\r\n");
				//read bammsat state vector
				if (bammsat_state_vector.voltages[0]>voltage_threshold)
				{
					PRINTF("Transition into the nominal mode!\r\n");
					//send telemetry packet?
					mode = NOMINAL_MODE;
				}
				//decide if I want to go into next mode or not
				break;
			case NOMINAL_MODE:
				PRINTF("I am in the nominal mode!\r\n");
				break;
			case SCIENCE_MODE:
				PRINTF("I am in the science mode!\r\n");
				break;
			case DOWNLINK_MODE:
				PRINTF("I am in the data downlink mode!\r\n");
				break;
			case UPDATE_MODE:
				break;
			default:
				break;
		}
		vTaskDelay( xDelay1000ms );
	}
}

/* queue read task */
static void vReceiverTask( void *pvParameters )
{
	/* Declare the variable that will hold the values received from the queue. */
	data_packet test_packet;
	BaseType_t xStatus;
	const TickType_t xTicksToWait = pdMS_TO_TICKS( 10000 );
	/* This task is also defined within an infinite loop. */
	for( ;; )
	{
		/* This call should always find the queue empty because this task
		 * will immediately remove any data that is written to the queue. */
		if( uxQueueMessagesWaiting( data_queue ) != 0 )
		{
			PRINTF( "Queue should have been empty!\r\n" );
		}

		/* Receive data from the queue. The first parameter is the queue from which data is to be received.
		 * The queue is created before the scheduler is started, and therefore before this
		 * task runs for the first time. The second parameter is the buffer into which the
		 * received data will be placed. In this case the buffer is simply the address of a
		 * variable that has the required size to hold the received data. The last parameter is
		 * the block time – the maximum amount of time that the task will remain in the
		 * Blocked state to wait for data to be available should the queue already be empty. */
		xStatus = xQueueReceive( data_queue, &test_packet, xTicksToWait );
		if( xStatus == pdPASS )
		{
			/* Data was successfully received from the queue, print out the received values */
			process_packet(test_packet);
		}
		else
		{
			/* Data was not received from the queue even after waiting for 100ms.
			 * This must be an error as the sending tasks are free running and will be
			 * continuously writing to the queue. */
			PRINTF( "Could not receive from the queue.\r\n" );
		}
	}
}


/*
 * @brief Task responsible for loopback.
 */
static void uart_task(void *pvParameters)
{
    int error;
    size_t n;
    BaseType_t xStatus;
    usart_config.srcclk = BOARD_DEBUG_UART_CLK_FREQ;
    usart_config.base = DEMO_USART;

    NVIC_SetPriority(DEMO_USART_IRQn, USART_NVIC_PRIO);

    if (0 > USART_RTOS_Init(&handle, &t_handle, &usart_config))
    {
        vTaskSuspend(NULL);
    }

    /* Receive user input and send it back to terminal. */
    do
    {
        error = USART_RTOS_Receive(&handle, recv_buffer, sizeof(recv_buffer), &n);
        if (error == kStatus_USART_RxRingBufferOverrun)
        {
            /* Notify about hardware buffer overrun */
            if (kStatus_Success !=
                USART_RTOS_Send(&handle, (uint8_t *)send_buffer_overrun, strlen(send_buffer_overrun)))
            {
                vTaskSuspend(NULL);
            }
        }
        if (n > 0)
        {
            /* send back the received data */
        	xStatus = xQueueSendToBack( data_queue, &recv_buffer, 0 );
        	if (xStatus != pdPASS )
        	{
        		/* The send operation could not complete because the queue was full -
        		 * this must be an error as the queue should never contain more than one item! */
        	       PRINTF( "Could not send to the queue.\r\n" );
        	}
        	//read the outbound queue from here
        	USART_RTOS_Send(&handle, (uint8_t *)to_send1, strlen(to_send1));

        }
    } while (kStatus_Success == error);

    USART_RTOS_Deinit(&handle);
    vTaskSuspend(NULL);
}

static void uart_task2(void *pvParameters)
{
    int error;
    size_t n;
    BaseType_t xStatus;
    usart_config2.srcclk = CLOCK_GetFreq(kCLOCK_Flexcomm4);
    usart_config2.base = DEMO_USART2;

	NVIC_SetPriority(DEMO_USART_IRQn2, USART_NVIC_PRIO2);

	if (0 > USART_RTOS_Init(&handle2, &t_handle2, &usart_config2))
    {
    	PRINTF("Problem initializing second USART\n");
        vTaskSuspend(NULL);
    }

    /* Receive subsystem data. */
	/* check if any data is up for the transmission */
    do
    {
        error = USART_RTOS_Receive(&handle2, recv_buffer2, sizeof(recv_buffer2), &n);
        if (error == kStatus_USART_RxRingBufferOverrun)
        {
            /* Notify about hardware buffer overrun */
            if (kStatus_Success !=
                USART_RTOS_Send(&handle2, (uint8_t *)send_buffer_overrun, strlen(send_buffer_overrun)))
            {
                vTaskSuspend(NULL);
            }
        }
        if (n > 0)
        {

        	xStatus = xQueueSendToBack( data_queue, &recv_buffer2, 0 );
        	if (xStatus != pdPASS )
        	{
        	/* The send operation could not complete because the queue was full - this must be an error as the queue should never contain more than one item! */
        		PRINTF( "Could not send to the queue.\r\n" );
        	}

            /* send back the received data */
        	USART_RTOS_Send(&handle2, (uint8_t *)to_send2, strlen(to_send2));
        }
    } while (kStatus_Success == error);

    USART_RTOS_Deinit(&handle2);
    vTaskSuspend(NULL);
}
