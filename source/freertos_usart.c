/*
 * The Clear BSD License
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// Features left to verify
// TODO: SDRAM R/W operations
// TODO: SD Card functionality
// TODO: real-time-clock/watch dog timer
// TODO: self-reflash from SD Card data
// TODO: power management functionality

/* Standard library includes */
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_emc.h"
#include "fsl_flexcomm.h"
#include "fsl_sd.h"
#include "fsl_usart_freertos.h"
#include "fsl_usart.h"
#include "fsl_iocon.h"

/* FatFS definitions */
#include "ff.h"
#include "diskio.h"
#include "fsl_sd_disk.h"

/* Board includes */
#include "board.h"
#include "pin_mux.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_USART USART0
#define DEMO_USART2 USART4
#define DEMO_USART3 USART8
#define DEMO_USART4 USART3

#define DEMO_USART_IRQHandler FLEXCOMM0_IRQHandler
#define DEMO_USART_IRQHandler2 FLEXCOMM4_IRQHandler
#define DEMO_USART_IRQHandler3 FLEXCOMM8_IRQHandler
#define DEMO_USART_IRQHandler4 FLEXCOMM3_IRQHandler


#define DEMO_USART_IRQn FLEXCOMM0_IRQn
#define DEMO_USART_IRQn2 FLEXCOMM4_IRQn
#define DEMO_USART_IRQn3 FLEXCOMM8_IRQn
#define DEMO_USART_IRQn4 FLEXCOMM3_IRQn

/* buffer size (in byte) for read/write operations */
#define BUFFER_SIZE (100U)

/* SDRAM values */
#define SDRAM_BASE_ADDR 0xa0000000
#define SDRAM_SIZE_BYTES (8 * 1024 * 1024)
#define SDRAM_EXAMPLE_DATALEN (SDRAM_SIZE_BYTES / 4)
#define SDRAM_TEST_PATTERN (2)

/* Task priorities. */
#define uart_task_PRIORITY (configMAX_PRIORITIES - 1)
#define USART_NVIC_PRIO 5
#define USART_NVIC_PRIO2 5
#define USART_NVIC_PRIO3 5
#define USART_NVIC_PRIO4 5

/*******************************************************************************
 * State of the satellite
 ******************************************************************************/
#define SAFE_MODE 1
#define NOMINAL_MODE 2
#define SCIENCE_MODE 3
#define DOWNLINK_MODE 4
#define UPDATE_MODE 5
/*******************************************************************************
 * State of the satellite
 ******************************************************************************/
#define DO_NOTHING 0
#define GOTO_SCIENCE_MODE 1
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
	float adc_vector_a[3];
	float adc_vector_g[3];
	float adc_vector_m[3];
	float adc_vector_i[3];
	uint32_t eps_packet_count;
	uint8_t command_queue[16];
	uint8_t mission_data[16];
} bammsat_state_vector;

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/
static status_t sdcardWaitCardInsert(void);
static void uart_task1(void *pvParameters);
static void uart_task2(void *pvParameters);
static void uart_task3(void *pvParameters);
static void uart_task4(void *pvParameters);

static void vReceiverTask( void *pvParameters);
static void vMasterLoop(void *pvParameters);
int test_sd_card();

/*******************************************************************************
 * SD Card definitions
 ******************************************************************************/
static FATFS g_fileSystem; /* File system object */
static FIL g_fileObject;   /* File object */

/* @brief decription about the read/write buffer
* The size of the read/write buffer should be a multiple of 512, since SDHC/SDXC card uses 512-byte fixed
* block length and this driver example is enabled with a SDHC/SDXC card.If you are using a SDSC card, you
* can define the block length by yourself if the card supports partial access.
* The address of the read/write buffer should align to the specific DMA data buffer address align value if
* DMA transfer is used, otherwise the buffer address is not important.
* At the same time buffer address/size should be aligned to the cache line size if cache is supported.
*/
SDK_ALIGN(uint8_t g_bufferWrite[SDK_SIZEALIGN(BUFFER_SIZE, SDMMC_DATA_BUFFER_ALIGN_CACHE)],
          MAX(SDMMC_DATA_BUFFER_ALIGN_CACHE, SDMMCHOST_DMA_BUFFER_ADDR_ALIGN));
SDK_ALIGN(uint8_t g_bufferRead[SDK_SIZEALIGN(BUFFER_SIZE, SDMMC_DATA_BUFFER_ALIGN_CACHE)],
          MAX(SDMMC_DATA_BUFFER_ALIGN_CACHE, SDMMCHOST_DMA_BUFFER_ADDR_ALIGN));
/*! @brief SDMMC host detect card configuration */
static const sdmmchost_detect_card_t s_sdCardDetect = {
#ifndef BOARD_SD_DETECT_TYPE
    .cdType = kSDMMCHOST_DetectCardByGpioCD,
#else
    .cdType = BOARD_SD_DETECT_TYPE,
#endif
    .cdTimeOut_ms = (~0U),
};

/*! @brief SDMMC card power control configuration */
#if defined DEMO_SDCARD_POWER_CTRL_FUNCTION_EXIST
static const sdmmchost_pwr_card_t s_sdCardPwrCtrl = {
    .powerOn = BOARD_PowerOnSDCARD, .powerOnDelay_ms = 500U, .powerOff = BOARD_PowerOffSDCARD, .powerOffDelay_ms = 0U,
};
#endif
/*******************************************************************************
 * UART definitions
 ******************************************************************************/
const char *to_send1 = "Replying on USART:FLEXCOMM0\r\n";
const char *to_send2 = "Replying on USART:FLEXCOMM4\r\n";
const char *to_send3 = "Replying on USART:FLEXCOMM8\r\n";
const char *to_send4 = "Replying on USART:FLEXCOMM3\r\n";

const char *send_buffer_overrun = "\r\nRing buffer overrun!\r\n";
uint8_t background_buffer1[40];
uint8_t recv_buffer1[20];

uint8_t background_buffer2[40];
uint8_t recv_buffer2[20];

uint8_t background_buffer3[40];
uint8_t recv_buffer3[20];

uint8_t background_buffer4[40];
uint8_t recv_buffer4[20];

usart_rtos_handle_t handle1;
struct _usart_handle t_handle1;

usart_rtos_handle_t handle2;
struct _usart_handle t_handle2;

usart_rtos_handle_t handle3;
struct _usart_handle t_handle3;

usart_rtos_handle_t handle4;
struct _usart_handle t_handle4;

struct rtos_usart_config usart_config1 = {
    .baudrate = 115200,
    .parity = kUSART_ParityDisabled,
    .stopbits = kUSART_OneStopBit,
    .buffer = background_buffer1,
    .buffer_size = sizeof(background_buffer1),
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

struct rtos_usart_config usart_config4 = {
    .baudrate = 115200,
    .parity = kUSART_ParityDisabled,
    .stopbits = kUSART_OneStopBit,
    .buffer = background_buffer4,
    .buffer_size = sizeof(background_buffer4),
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


/*******************************************************************************
 * Command queues definitions
 *  data_queue: receiving commands from subsystems
 *	comms_queue: for packets sent to communication subsystem
 ******************************************************************************/
QueueHandle_t data_queue;
QueueHandle_t comms_queue;

/*******************************************************************************
 * Code
 ******************************************************************************/

/* Checks the SDRAM functionality */
status_t SDRAM_DataBusCheck(volatile uint32_t *address)
{
    uint32_t data = 0;

    /* Write the walking 1's data test. */
    for (data = 1; data != 0; data <<= 1)
    {
        *address = data;

        /* Read the data out of the address and check. */
        if (*address != data)
        {
            return kStatus_Fail;
        }
    }
    return kStatus_Success;
}

status_t SDRAM_AddressBusCheck(volatile uint32_t *address, uint32_t bytes)
{
    uint32_t pattern = 0x55555555;
    uint32_t size = bytes / 4;
    uint32_t offset;
    uint32_t checkOffset;

    /* write the pattern to the power-of-two address. */
    for (offset = 1; offset < size; offset <<= 1)
    {
        address[offset] = pattern;
    }
    address[0] = ~pattern;

    /* Read and check. */
    for (offset = 1; offset < size; offset <<= 1)
    {
        if (address[offset] != pattern)
        {
            return kStatus_Fail;
        }
    }

    if (address[0] != ~pattern)
    {
        return kStatus_Fail;
    }

    /* Change the data to the revert one address each time
     * and check there is no effect to other address. */
    for (offset = 1; offset < size; offset <<= 1)
    {
        address[offset] = ~pattern;
        for (checkOffset = 1; checkOffset < size; checkOffset <<= 1)
        {
            if ((checkOffset != offset) && (address[checkOffset] != pattern))
            {
                return kStatus_Fail;
            }
        }
        address[offset] = pattern;
    }
    return kStatus_Success;
}

void sdram_check()
{
	uint32_t index;
	uint32_t *sdram = (uint32_t *)SDRAM_BASE_ADDR; /* SDRAM start address. */
	//check hardware status
	if (SDRAM_DataBusCheck(sdram) != kStatus_Success)
	{
		PRINTF("\r\n SDRAM data bus check is failure.\r\n");
	}
	if (SDRAM_AddressBusCheck(sdram, SDRAM_SIZE_BYTES) != kStatus_Success)
	{
		PRINTF("\r\n SDRAM address bus check is failure.\r\n");
	}
	//check functionality
	PRINTF("\r\n Start EMC SDRAM access example.\r\n");
	PRINTF("\r\n SDRAM Write Start, Start Address 0x%x, Data Length %d !\r\n", sdram, SDRAM_EXAMPLE_DATALEN);
	/* Prepare data and write to SDRAM. */
	for (index = 0; index < SDRAM_EXAMPLE_DATALEN; index++)
	{
		*(uint32_t *)(sdram + index) = index;
	}
	PRINTF("\r\n SDRAM Write finished!\r\n");
	PRINTF("\r\n SDRAM Read/Check Start, Start Address 0x%x, Data Length %d !\r\n", sdram, SDRAM_EXAMPLE_DATALEN);
	/* Read data from the SDRAM. */
	for (index = 0; index < SDRAM_EXAMPLE_DATALEN; index++)
	{
		if (*(uint32_t *)(sdram + index) != index)
		{
			PRINTF("\r\n SDRAM Write Data and Read Data Check Error!\r\n");
			break;
		}
	}
	PRINTF("\r\n SDRAM Write Data and Read Data Succeed.\r\n");
	EMC_Deinit(EMC);
	PRINTF("\r\n SDRAM Example End.\r\n");
}

int test_sd_card()
{
	FRESULT error;
	DIR directory; /* Directory object */
	FILINFO fileInformation;
	UINT bytesWritten;
	UINT bytesRead;
	const TCHAR driverNumberBuffer[3U] = {SDDISK + '0', ':', '/'};
	volatile bool failedFlag = false;
	char ch = '0';
	BYTE work[FF_MAX_SS];

	PRINTF("\r\nFATFS example to demonstrate how to use FATFS with SD card.\r\n");

	PRINTF("\r\nPlease insert a card into board.\r\n");

	if (sdcardWaitCardInsert() != kStatus_Success)
	{
	        return -1;
	}

	if (f_mount(&g_fileSystem, driverNumberBuffer, 0U))
	{
	        PRINTF("Mount volume failed.\r\n");
	        return -1;
	}

	#if (FF_FS_RPATH >= 2U)
	    error = f_chdrive((char const *)&driverNumberBuffer[0U]);
	    if (error)
	    {
	        PRINTF("Change drive failed.\r\n");
	        return -1;
	    }
	#endif

	#if FF_USE_MKFS
	    PRINTF("\r\nMake file system......The time may be long if the card capacity is big.\r\n");
	    if (f_mkfs(driverNumberBuffer, FM_ANY, 0U, work, sizeof work))
	    {
	        PRINTF("Make file system failed.\r\n");
	        return -1;
	    }
	#endif /* FF_USE_MKFS */

	PRINTF("\r\nCreate directory......\r\n");
	error = f_mkdir(_T("/dir_1"));
    if (error)
	{
    	if (error == FR_EXIST)
	     {
	            PRINTF("Directory exists.\r\n");
	        }
	        else
	        {
	            PRINTF("Make directory failed.\r\n");
	            return -1;
	        }
	}

	 PRINTF("\r\nCreate a file in that directory......\r\n");
	    error = f_open(&g_fileObject, _T("/dir_1/f_1.dat"), (FA_WRITE | FA_READ | FA_CREATE_ALWAYS));
	    if (error)
	    {
	        if (error == FR_EXIST)
	        {
	            PRINTF("File exists.\r\n");
	        }
	        else
	        {
	            PRINTF("Open file failed.\r\n");
	            return -1;
	        }
	    }

	    PRINTF("\r\nCreate a directory in that directory......\r\n");
	    error = f_mkdir(_T("/dir_1/dir_2"));
	    if (error)
	    {
	        if (error == FR_EXIST)
	        {
	            PRINTF("Directory exists.\r\n");
	        }
	        else
	        {
	            PRINTF("Directory creation failed.\r\n");
	            return -1;
	        }
	    }

	    PRINTF("\r\nList the file in that directory......\r\n");
	    if (f_opendir(&directory, "/dir_1"))
	    {
	        PRINTF("Open directory failed.\r\n");
	        return -1;
	    }

	    for (;;)
	    {
	        error = f_readdir(&directory, &fileInformation);

	        /* To the end. */
	        if ((error != FR_OK) || (fileInformation.fname[0U] == 0U))
	        {
	            break;
	        }
	        if (fileInformation.fname[0] == '.')
	        {
	            continue;
	        }
	        if (fileInformation.fattrib & AM_DIR)
	        {
	            PRINTF("Directory file : %s.\r\n", fileInformation.fname);
	        }
	        else
	        {
	            PRINTF("General file : %s.\r\n", fileInformation.fname);
	        }
	    }

	    memset(g_bufferWrite, 'a', sizeof(g_bufferWrite));
	    g_bufferWrite[BUFFER_SIZE - 2U] = '\r';
	    g_bufferWrite[BUFFER_SIZE - 1U] = '\n';

	    PRINTF("\r\nWrite/read file until encounters error......\r\n");
	    while (true)
	    {
	        if (failedFlag || (ch == 'q'))
	        {
	            break;
	        }

	        PRINTF("\r\nWrite to above created file.\r\n");
	        error = f_write(&g_fileObject, g_bufferWrite, sizeof(g_bufferWrite), &bytesWritten);
	        if ((error) || (bytesWritten != sizeof(g_bufferWrite)))
	        {
	            PRINTF("Write file failed. \r\n");
	            failedFlag = true;
	            continue;
	        }

	        /* Move the file pointer */
	        if (f_lseek(&g_fileObject, 0U))
	        {
	            PRINTF("Set file pointer position failed. \r\n");
	            failedFlag = true;
	            continue;
	        }

	        PRINTF("Read from above created file.\r\n");
	        memset(g_bufferRead, 0U, sizeof(g_bufferRead));
	        error = f_read(&g_fileObject, g_bufferRead, sizeof(g_bufferRead), &bytesRead);
	        if ((error) || (bytesRead != sizeof(g_bufferRead)))
	        {
	            PRINTF("Read file failed. \r\n");
	            failedFlag = true;
	            continue;
	        }

	        PRINTF("Compare the read/write content......\r\n");
	        if (memcmp(g_bufferWrite, g_bufferRead, sizeof(g_bufferWrite)))
	        {
	            PRINTF("Compare read/write content isn't consistent.\r\n");
	            failedFlag = true;
	            continue;
	        }
	        PRINTF("The read/write content is consistent.\r\n");

	        PRINTF("\r\nInput 'q' to quit read/write.\r\nInput other char to read/write file again.\r\n");
	        ch = GETCHAR();
	        PUTCHAR(ch);
	    }
	    PRINTF("\r\nThe example will not read/write file again.\r\n");

	    if (f_close(&g_fileObject))
	    {
	        PRINTF("\r\nClose file failed.\r\n");
	        return -1;
	    }

	    /*while (true)
	    {
	    } */
	    return 0;
}


/* Init board hardware:
 * 	attach 12 MHz clock to FLEXCOMM0 (debug console)
 *	attach 12 MHz clock to FLEXCOMM4 (UART4)
 *	attach 12 MHz clock to FLEXCOMM8 (UART8)
 *  init SDRAM memory
 *  init SD Card hardware
 *  reset the FLEXCOMM interfaces
 */
void init_hardware()
{
	CLOCK_EnableClock(kCLOCK_InputMux);
	CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);
	CLOCK_AttachClk(kFRO12M_to_FLEXCOMM4);
	CLOCK_AttachClk(kFRO12M_to_FLEXCOMM8);
	CLOCK_AttachClk(kFRO12M_to_FLEXCOMM3);
	CLOCK_AttachClk(BOARD_SDIF_CLK_ATTACH);
	CLOCK_SetClkDiv(kCLOCK_DivSdioClk, 1U, true);
	BOARD_InitPins();
	BOARD_BootClockFROHF96M();
	/* need call this function to clear the halt bit in clock divider register */
	CLOCK_SetClkDiv(kCLOCK_DivSdioClk, (uint32_t)(SystemCoreClock / FSL_FEATURE_SDIF_MAX_SOURCE_CLOCK + 1U), true);
	BOARD_InitDebugConsole();
	BOARD_InitSDRAM();
	RESET_PeripheralReset(kFC4_RST_SHIFT_RSTn);
	RESET_PeripheralReset(kFC3_RST_SHIFT_RSTn);
	RESET_PeripheralReset(kFC8_RST_SHIFT_RSTn);
}

/*!
 * @brief Application entry point.
 */
int main(void)
{
	init_hardware();
	sdram_check();
	test_sd_card();
	/* create inbound and outbound queues */
    data_queue = xQueueCreate(32, sizeof(recv_buffer1));
    comms_queue = xQueueCreate(32, sizeof(recv_buffer1));

    /* create UART tasks */
    if (xTaskCreate(uart_task1, "Uart_task1", configMINIMAL_STACK_SIZE + 100, NULL, uart_task_PRIORITY, NULL) != pdPASS)
    {
        PRINTF("UART 1 Task creation failed!.\r\n");
        while (1)
            ;
    }

    if (xTaskCreate(uart_task2, "Uart_task2", configMINIMAL_STACK_SIZE + 100, NULL, uart_task_PRIORITY, NULL) != pdPASS)
    {
        PRINTF("UART 2 Task creation failed!.\r\n");
        while (1)
            ;
    }

    if (xTaskCreate(uart_task3, "Uart_task3", configMINIMAL_STACK_SIZE + 100, NULL, uart_task_PRIORITY, NULL) != pdPASS)
    {
       PRINTF("UART 3 Task creation failed!.\r\n");
       while (1)
    	   ;
    }

    if (xTaskCreate(uart_task4, "Uart_task4", configMINIMAL_STACK_SIZE + 100, NULL, uart_task_PRIORITY, NULL) != pdPASS)
    {
    	PRINTF("UART 4 Task creation failed!.\r\n");
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
		case PLD:
			PRINTF("Subsystem: PLD\r\n");
			PRINTF("Packet type: %u\r\n",packet.type);
			switch (packet.type)
			{
				case 1:
					for (i=0;i<16;i++)
						bammsat_state_vector.mission_data[i] = packet.command[i];
					break;
				default:
					break;
			}
			break;
		case ADC:
			PRINTF("Subsystem: ADC\r\n");
			PRINTF("Packet type: %u\r\n",packet.type);
			PRINTF("ADC packet data: \r\n");
			for (i=0;i<3;i++) PRINTF("%3.2f ",packet.data[i]);
			PRINTF("\r\n");
			switch (packet.type)
			{
				case 1:
					for (i=0;i<3;i++) bammsat_state_vector.adc_vector_a[i] = packet.data[i];
					break;
				case 2:
					for (i=0;i<3;i++) bammsat_state_vector.adc_vector_g[i] = packet.data[i];
					break;
				case 3:
					for (i=0;i<3;i++) bammsat_state_vector.adc_vector_m[i] = packet.data[i];
					break;
				case 4:
					for (i=0;i<3;i++) bammsat_state_vector.adc_vector_i[i] = packet.data[i];
					break;
				default:
					break;
			}

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
		// bammsat_state_vector.adc_vector[i]=0.0;
		 bammsat_state_vector.sensors[i]=0.0;
	 }
	 for (i=0;i<16;i++)
	 {
		 bammsat_state_vector.command_queue[i] = 0;
	 	 bammsat_state_vector.mission_data[i] = 0;
	 }
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

uint8_t process_command_queue()
{
	uint8_t command;
	switch(bammsat_state_vector.command_queue[0])
	{
		case 1:
			PRINTF ("Received COM command with first value=1\r\n");
			command = GOTO_SCIENCE_MODE;
			break;
		default:
			break;
	}
	return command;
}

void process_mission_data()
{
	switch(bammsat_state_vector.mission_data[0])
	{
		case 1:
			PRINTF ("Received Payload packet with first value=1\r\n");
			break;
		default:
			break;
	}
}

uint8_t identify_packet_subsystem(data_packet* packet)
{
		return packet->subsystem_id;
}

/* FreeRTOS tasks */
static void vMasterLoop(void *pvParameters)
{
	uint8_t mode=1;
	uint8_t command;
	bammsat_state_vector.eps_packet_count = 0;
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
				PRINTF("I am in the nominal mode!\r\n");
				send_comms_packet(OBC, 1, PRIORITY_LOW, (uint8_t*)"OBC mode:nominal");
				//parse command queue
				//decide what to do next
				command = process_command_queue();
				//pre-conditions must match too
				if (command == GOTO_SCIENCE_MODE)
					mode = SCIENCE_MODE;
				break;
			case SCIENCE_MODE:
				PRINTF("I am in the science mode!\r\n");
				send_comms_packet(OBC, 1, PRIORITY_LOW, (uint8_t*)"OBC mode:science");
				process_mission_data();
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

static status_t sdcardWaitCardInsert(void)
{
    /* Save host information. */
    g_sd.host.base = SD_HOST_BASEADDR;
    g_sd.host.sourceClock_Hz = SD_HOST_CLK_FREQ;
    /* card detect type */
    g_sd.usrParam.cd = &s_sdCardDetect;
#if defined DEMO_SDCARD_POWER_CTRL_FUNCTION_EXIST
    g_sd.usrParam.pwr = &s_sdCardPwrCtrl;
#endif
    /* SD host init function */
    if (SD_HostInit(&g_sd) != kStatus_Success)
    {
        PRINTF("\r\nSD host init fail\r\n");
        return kStatus_Fail;
    }
    /* power off card */
    SD_PowerOffCard(g_sd.host.base, g_sd.usrParam.pwr);
    /* wait card insert */
    if (SD_WaitCardDetectStatus(SD_HOST_BASEADDR, &s_sdCardDetect, true) == kStatus_Success)
    {
        PRINTF("\r\nCard inserted.\r\n");
        /* power on the card */
        SD_PowerOnCard(g_sd.host.base, g_sd.usrParam.pwr);
    }
    else
    {
        PRINTF("\r\nCard detect fail.\r\n");
        return kStatus_Fail;
    }

    return kStatus_Success;
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

static void uart_task1(void *pvParameters)
{
    int error;
    size_t n;
    BaseType_t xStatus;
    data_packet comms_packet;
    uint8_t subsystem_id;
    usart_config1.srcclk = BOARD_DEBUG_UART_CLK_FREQ;
    usart_config1.base = DEMO_USART;

    NVIC_SetPriority(DEMO_USART_IRQn, USART_NVIC_PRIO);

    if (0 > USART_RTOS_Init(&handle1, &t_handle1, &usart_config1))
    {
        vTaskSuspend(NULL);
    }
    /* Receive user input and send it back to terminal. */
    do
    {
        error = USART_RTOS_Receive(&handle1, recv_buffer1, sizeof(recv_buffer1), &n);
        if (error == kStatus_USART_RxRingBufferOverrun)
        {
            /* Notify about hardware buffer overrun */
            if (kStatus_Success !=
                USART_RTOS_Send(&handle1, (uint8_t *)send_buffer_overrun, strlen(send_buffer_overrun)))
            {
                vTaskSuspend(NULL);
            }
        }
        if (n > 0)
        {
            /* send back the received data */
        	xStatus = xQueueSendToBack( data_queue, &recv_buffer1, 0 );
        	if (xStatus != pdPASS )
        	{
        		/* The send operation could not complete because the queue was full -
        		 * this must be an error as the queue should never contain more than one item! */
        	       PRINTF( "Could not send to the queue.\r\n" );
        	}
        }
        subsystem_id = identify_packet_subsystem((data_packet*)recv_buffer1);
        if (subsystem_id == COM)
        {
        	//now send the outstanding data
            xStatus = xQueueReceive( comms_queue, &comms_packet, 0 );
            if( xStatus == pdPASS )
            {
            	/* Data was successfully received from the queue, print out the received values */
                PRINTF( "Send the packet to the COMMS subsystem!\r\n" );
                USART_RTOS_Send(&handle1, (uint8_t *) &comms_packet,20);
            }
            else
            {
            	/* Data was not received from the queue even after waiting for 100ms.
                 * This must be an error as the sending tasks are free running and will be
                 * continuously writing to the queue. */
                PRINTF( "Could not receive from the comms queue.\r\n" );
             }
        }
        else
        {
        	//read the outbound queue from here
        	USART_RTOS_Send(&handle1, (uint8_t *)to_send1, strlen(to_send1));
        }
    } while (kStatus_Success == error);
    USART_RTOS_Deinit(&handle1);
    vTaskSuspend(NULL);
}

static void uart_task2(void *pvParameters)
{
    int error;
    size_t n;
    BaseType_t xStatus;
	data_packet comms_packet;
	uint8_t subsystem_id;
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
        subsystem_id = identify_packet_subsystem((data_packet*)recv_buffer2);
        if (subsystem_id == COM)
        {
        	//now send the outstanding data from comms queue
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
        		PRINTF( "Could not receive from the comms queue.\r\n" );
        	}
        }
        else
        {
        	//read the outbound queue from here
        	USART_RTOS_Send(&handle2, (uint8_t *)to_send2, strlen(to_send2));
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
    data_packet comms_packet;
    uint8_t subsystem_id;
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
        }
        subsystem_id = identify_packet_subsystem((data_packet*)recv_buffer3);
        if (subsystem_id == COM)
        {
        	//now send the outstanding data from comms queue
            xStatus = xQueueReceive( comms_queue, &comms_packet, 0 );
            if( xStatus == pdPASS )
            {
            	/* Data was successfully received from the queue, print out the received values */
                PRINTF( "Send the packet to the COMMS subsystem!\r\n" );
                USART_RTOS_Send(&handle3, (uint8_t *) &comms_packet,20);
             }
             else
             {
            	 /* Data was not received from the queue even after waiting for 100ms.
                 * This must be an error as the sending tasks are free running and will be
                 * continuously writing to the queue. */
                PRINTF( "Could not receive from the comms queue.\r\n" );
              }
         }
         else
         {
        	 //read the outbound queue from here
             USART_RTOS_Send(&handle3, (uint8_t *)to_send3, strlen(to_send3));
         }
    } while (kStatus_Success == error);
    USART_RTOS_Deinit(&handle3);
    vTaskSuspend(NULL);
}

static void uart_task4(void *pvParameters)
{
    int error;
    size_t n;
    BaseType_t xStatus;
    data_packet comms_packet;
    uint8_t subsystem_id;
    usart_config4.srcclk = CLOCK_GetFreq(kCLOCK_Flexcomm3);
    usart_config4.base = DEMO_USART4;
    NVIC_SetPriority(DEMO_USART_IRQn4, USART_NVIC_PRIO4);

    if (0 > USART_RTOS_Init(&handle4, &t_handle4, &usart_config4))
    {
        vTaskSuspend(NULL);
    }

    /* Receive user input and send it back to terminal. */
    do
    {
        error = USART_RTOS_Receive(&handle4, recv_buffer4, sizeof(recv_buffer4), &n);
        if (error == kStatus_USART_RxRingBufferOverrun)
        {
            /* Notify about hardware buffer overrun */
            if (kStatus_Success !=
                USART_RTOS_Send(&handle4, (uint8_t *)send_buffer_overrun, strlen(send_buffer_overrun)))
            {
                vTaskSuspend(NULL);
            }
        }
        if (n > 0)
        {
            /* send back the received data */
        	xStatus = xQueueSendToBack( data_queue, &recv_buffer4, 0 );
        	if (xStatus != pdPASS )
        	{
        		/* The send operation could not complete because the queue was full -
        		 * this must be an error as the queue should never contain more than one item! */
        	       PRINTF( "Could not send to the queue.\r\n" );
        	}
         }
        subsystem_id = identify_packet_subsystem((data_packet*)recv_buffer4);
        if (subsystem_id == COM)
        {
        	//now send the outstanding data from comms queue
            xStatus = xQueueReceive( comms_queue, &comms_packet, 0 );
            if( xStatus == pdPASS )
            {
            	/* Data was successfully received from the queue, print out the received values */
                PRINTF( "Send the packet to the COMMS subsystem!\r\n" );
                USART_RTOS_Send(&handle4, (uint8_t *) &comms_packet,20);
            }
            else
            {
            	/* Data was not received from the queue even after waiting for 100ms.
                 * This must be an error as the sending tasks are free running and will be
                 * continuously writing to the queue. */
                PRINTF( "Could not receive from the comms queue.\r\n" );
             }
         }
         else
         {
               //read the outbound queue from here
               USART_RTOS_Send(&handle4, (uint8_t *)to_send4, strlen(to_send4));
         }
    } while (kStatus_Success == error);
    USART_RTOS_Deinit(&handle4);
    vTaskSuspend(NULL);
}
