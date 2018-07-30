
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
#define DEMO_USART3 USART8

#define DEMO_USART_IRQHandler FLEXCOMM0_IRQHandler
#define DEMO_USART_IRQHandler2 FLEXCOMM4_IRQHandler
#define DEMO_USART_IRQHandler3 FLEXCOMM8_IRQHandler

#define DEMO_USART_IRQn FLEXCOMM0_IRQn
#define DEMO_USART_IRQn2 FLEXCOMM4_IRQn
#define DEMO_USART_IRQn3 FLEXCOMM8_IRQn

/* Task priorities. */
#define uart_task_PRIORITY (configMAX_PRIORITIES - 1)
#define USART_NVIC_PRIO 5
#define USART_NVIC_PRIO2 5
#define USART_NVIC_PRIO3 5

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
	float sensors[4];
	float adc_vector[4];
	uint32_t eps_packet_count;
	uint8_t command_queue[16];
} bammsat_state_vector;

/*******************************************************************************
 * Task Prototypes
 ******************************************************************************/
static void uart_task(void *pvParameters);
static void uart_task2(void *pvParameters);
static void uart_task3(void *pvParameters);

static void vReceiverTask( void *pvParameters);
static void vMasterLoop(void *pvParameters);
/*******************************************************************************
 * Code
 ******************************************************************************/
const char *to_send1 = "Replying on USART:FLEXCOMM0\r\n";
const char *to_send2 = "Replying on USART:FLEXCOMM4\r\n";
const char *to_send3 = "Replying on USART:FLEXCOMM8"
		"\r\n";

const char *send_buffer_overrun = "\r\nRing buffer overrun!\r\n";
uint8_t background_buffer[40];
uint8_t recv_buffer[20];

uint8_t background_buffer2[40];
uint8_t recv_buffer2[20];

uint8_t background_buffer3[40];
uint8_t recv_buffer3[20];

usart_rtos_handle_t handle;
struct _usart_handle t_handle;

usart_rtos_handle_t handle2;
struct _usart_handle t_handle2;

usart_rtos_handle_t handle3;
struct _usart_handle t_handle3;

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

struct rtos_usart_config usart_config3 = {
    .baudrate = 115200,
    .parity = kUSART_ParityDisabled,
    .stopbits = kUSART_OneStopBit,
    .buffer = background_buffer3,
    .buffer_size = sizeof(background_buffer3),
};

typedef struct {
	uint8_t subsystem_id;
	uint8_t type;
	uint8_t priority;
	uint8_t reserved;//checksum or timestamp
	union {
		float data[4]; //EPS or ADC
		uint8_t command[16]; //PLD or COM
	};
} data_packet;

QueueHandle_t data_queue; //common queue - receiving commands from subsystems
QueueHandle_t comms_queue; //queue for packets sent to communication subsystem

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
    // attach 12 MHz clock to UART8
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM8);

    BOARD_InitPins();
    BOARD_InitDebugConsole();
    RESET_PeripheralReset(kFC4_RST_SHIFT_RSTn);
    RESET_PeripheralReset(kFC8_RST_SHIFT_RSTn);

    /* create inbound queue */
    data_queue = xQueueCreate(32, sizeof(recv_buffer));

    /* create outbound queues */
    comms_queue = xQueueCreate(32, sizeof(recv_buffer));

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

    if (xTaskCreate(uart_task3, "Uart_task3", configMINIMAL_STACK_SIZE + 100, NULL, uart_task_PRIORITY, NULL) != pdPASS)
       {
           PRINTF("USART 3 Task creation failed!.\r\n");
           while (1)
               ;
       }

    //create queue reader task
    if (xTaskCreate(vReceiverTask, "QueueReader", configMINIMAL_STACK_SIZE + 100, NULL, uart_task_PRIORITY, NULL) != pdPASS)
    {
        PRINTF("Queue Reader Task creation failed!.\r\n");
        while (1)
               ;
    }

    //create master state machine
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
	uint32_t eps_packet_count=0;
	switch (packet.subsystem_id)
	{
		case EPS:
			PRINTF("Subsystem: EPS\r\n");
			PRINTF("Packet type: %u\r\n",packet.type);
			//print packet data
			switch (packet.type)
			{
				case 1:
					PRINTF("Voltages data packet: ");
					eps_packet_count++;
					for(i=0;i<4;i++)
					{
						bammsat_state_vector.voltages[i] = packet.data[i];
						PRINTF("%3.2f ",packet.data[i]);
					}
					PRINTF("\r\n");
					break;
				case 2:
					PRINTF("Sensors data packet: ");
					eps_packet_count++;
					for(i=0;i<4;i++)
					{
						bammsat_state_vector.sensors[i] = packet.data[i];
						PRINTF("%3.2f ",packet.data[i]);
					}
					PRINTF("\r\n");
					break;
				default:
					break;
			}
			break;
		case COM:
			PRINTF("Subsystem: COM\r\n");
			PRINTF("Packet type: %u\r\n",packet.type);
			switch (packet.type)
			{
				case 1:
					for (i=0;i<16;i++)
						bammsat_state_vector.command_queue[i] = packet.command[i];
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
		 bammsat_state_vector.adc_vector[i]=0.0;
		 bammsat_state_vector.sensors[i]=0.0;
	 }
	 for (i=0;i<16;i++)
		 bammsat_state_vector.command_queue[i] = 0;
 }

void send_comms_packet(uint8_t p_subsystem, uint8_t p_type, uint8_t p_priority, uint8_t* message)
{
	data_packet comms_packet;
	BaseType_t xStatus;
	comms_packet.subsystem_id = p_subsystem;
	comms_packet.type = p_type;
	comms_packet.priority = p_priority;
	comms_packet.reserved = 0;
	strncpy(comms_packet.data, message, 16);
	xStatus = xQueueSendToBack( comms_queue, &comms_packet, 0 );
	if (xStatus != pdPASS )
	{
		PRINTF( "Could not send to the comms queue.\r\n" );
	}
}

void process_command_queue()
{
	switch(bammsat_state_vector.command_queue[0])
	{
		case 1:
			PRINTF ("Received COM command A\r\n");
			break;
		default:
			break;
	}
}
static void vMasterLoop(void *pvParameters)
{
	uint8_t mode=1;
	bammsat_state_vector.eps_packet_count = 0;
	data_packet comms_packet;
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
				send_comms_packet(OBC, 1, PRIORITY_LOW, (uint8_t*)"OBC mode:   safe");
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
				send_comms_packet(OBC, 1, PRIORITY_LOW, (uint8_t*)"OBC mode:nominal");
				//parse command queue
				process_command_queue();
				PRINTF("I am in the nominal mode!\r\n");
				break;
			case SCIENCE_MODE:
				send_comms_packet(OBC, 1, PRIORITY_LOW, (uint8_t*)"OBC mode:science");
				PRINTF("I am in the science mode!\r\n");
				break;
			case DOWNLINK_MODE:
				send_comms_packet(OBC, 1, PRIORITY_LOW, (uint8_t*)"OBC mode:dwnlink");
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
	data_packet comms_packet;

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
        }
        //now send the outstanding data
        xStatus = xQueueReceive( comms_queue, &comms_packet, 0 );
        if( xStatus == pdPASS )
        {
        	/* Data was successfully received from the queue, print out the received values */
        	PRINTF( "Send the packet to the COMMS subsystem!\r\n" );
        	USART_RTOS_Send(&handle2, (uint8_t *) &comms_packet,20);
        }
        else
        {
        		/* Data was not received from the queue even after waiting for 100ms.
        		* This must be an error as the sending tasks are free running and will be
        		* continuously writing to the queue. */
        		USART_RTOS_Send(&handle2, (uint8_t *)to_send2, strlen(to_send2));
        		PRINTF( "Could not receive from the comms queue.\r\n" );
        }

    } while (kStatus_Success == error);
    USART_RTOS_Deinit(&handle2);
    vTaskSuspend(NULL);
}

/*
 * @brief Task responsible for loopback.
 */
static void uart_task3(void *pvParameters)
{
    int error;
    size_t n;
    BaseType_t xStatus;
    usart_config3.srcclk = CLOCK_GetFreq(kCLOCK_Flexcomm8);
    usart_config3.base = DEMO_USART3;

    NVIC_SetPriority(DEMO_USART_IRQn3, USART_NVIC_PRIO3);

    if (0 > USART_RTOS_Init(&handle3, &t_handle3, &usart_config3))
    {
        vTaskSuspend(NULL);
    }

    /* Receive user input and send it back to terminal. */
    do
    {
        error = USART_RTOS_Receive(&handle3, recv_buffer3, sizeof(recv_buffer3), &n);
        if (error == kStatus_USART_RxRingBufferOverrun)
        {
            /* Notify about hardware buffer overrun */
            if (kStatus_Success !=
                USART_RTOS_Send(&handle3, (uint8_t *)send_buffer_overrun, strlen(send_buffer_overrun)))
            {
                vTaskSuspend(NULL);
            }
        }
        if (n > 0)
        {
            /* send back the received data */
        	xStatus = xQueueSendToBack( data_queue, &recv_buffer3, 0 );
        	if (xStatus != pdPASS )
        	{
        		/* The send operation could not complete because the queue was full -
        		 * this must be an error as the queue should never contain more than one item! */
        	       PRINTF( "Could not send to the queue.\r\n" );
        	}
        	//read the outbound queue from here
        	USART_RTOS_Send(&handle3, (uint8_t *)to_send3, strlen(to_send3));
        }
    } while (kStatus_Success == error);
    USART_RTOS_Deinit(&handle3);
    vTaskSuspend(NULL);
}
