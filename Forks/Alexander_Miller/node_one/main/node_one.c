#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include <stdlib.h>
#include <stdint.h>
#include "driver/can.h"
#include "esp_timer.h"
#include "soc/soc.h"
#include "CANopen.h"
#include "CO_OD.h"
#include "CO_config.h"
#include "modul_config.h"
#include "dunker.h"
#include "Gyro.h"

uint8_t counter = 0;

volatile uint32_t coInterruptCounter = 0U; /* variable increments each millisecond */

//Timer Interrupt Configuration
static void coMainTask(void *arg);

esp_timer_create_args_t coMainTaskArgs;
//Timer Handle
esp_timer_handle_t periodicTimer;

void mainTask(void *pvParameter)
{
		coMainTaskArgs.callback = &coMainTask;
		coMainTaskArgs.name = "coMainTask";
		CO_NMT_reset_cmd_t reset = CO_RESET_NOT;
		vTaskDelay(BOOT_WAIT / portTICK_PERIOD_MS);
		while (reset != CO_RESET_APP)
		{
				/* CANopen communication reset - initialize CANopen objects *******************/
				CO_ReturnError_t err;
				uint32_t coInterruptCounterPrevious;

				/* initialize CANopen */
				err = CO_init(NULL, NODE_ID_SELF /* NodeID */, CAN_BITRATE /* bit rate */);
				if (err != CO_ERROR_NO)
				{
						ESP_LOGE("mainTask", "CO_init failed. Errorcode: %d", err);
						CO_errorReport(CO->em, CO_EM_MEMORY_ALLOCATION_ERROR, CO_EMC_SOFTWARE_INTERNAL, err);
						esp_restart();
				}
				/* Configure Timer interrupt function for execution every CO_MAIN_TASK_INTERVAL */
				ESP_ERROR_CHECK(esp_timer_create(&coMainTaskArgs, &periodicTimer));
				ESP_ERROR_CHECK(esp_timer_start_periodic(periodicTimer, CO_MAIN_TASK_INTERVAL));
				/* start CAN */
				CO_CANsetNormalMode(CO->CANmodule[0]);

				reset = CO_RESET_NOT;
				coInterruptCounterPrevious = coInterruptCounter;

				/*Set Operating Mode of Slaves to Operational*/
				CO_sendNMTcommand(CO, 0x01, 16);
				
				CO_CANtx_t *buffer = CO_CANtxBufferInit(
														CO->CANmodule[0], /* pointer to CAN module used for sending this message */
														0,     /* index of specific buffer inside CAN module */
														0x0000,           /* CAN identifier */
														0,                /* rtr */
														2,                /* number of data bytes */
														0);    
				/* Initialise system components */
				// gyro_init(CO);
				/* application init code goes here. */
				//rosserialSetup();
				
				//Mensaje CAN simple
				buffer->data[0]=5;
				buffer->ident=0x1A;
				CO_CANsend(CO->CANmodule[1],buffer);

				//Comunicación cliente SDO 
				
				uint32_t response = 0;
				uint32_t data = 0;
				uint8_t *datos;
				ESP_LOGI("mainTask", "SDO download");
				CO_SDOclientDownloadInitiate(CO->SDOclient[0], 0x1600, 0x1, &datos,sizeof(buffer),5);
				CO_SDOclientDownload(CO->SDOclient[0],100,1000, &response);
				CO_SDOclientClose(CO->SDOclient[0]);
				ESP_LOGI("mainTask", "SDO download end");
				
				//uint16_t index = CO_OD_find(CO->SDO[0], 0x1600);
				//uint16_t length = CO_OD_getLength(CO->SDO[0], index, 2);
				//uint8_t* p = CO_OD_getDataPointer(CO->SDO[0], index, 2);
				// *p = 123456;
				//printf("VALOR OD: %d \n", *p);

				ESP_LOGI("mainTask", "SDO upload");
				CO_SDOclientUploadInitiate(CO->SDOclient[0], 0x1600, 0x1, &buffer,sizeof(buffer),5);
				CO_SDOclientUpload(CO->SDOclient[0],100,1000, &data,&response);
				CO_SDOclientClose(CO->SDOclient[0]);
				ESP_LOGI("mainTask", "SDO upload end");



				//Comunicación PDO
				/*
				uint8_t *mapPointer[8];
				ESP_LOGI("mainTask", "PDO ");
				CO->TPDO[0]->sendRequest = 1;
				*CO->TPDO[0]->mapPointer = 0x1600;
				CO->TPDO[0]->valid = true; // set to true to send PDO
				CO->TPDO[0]->sendIfCOSFlags = 1;
				vTaskDelay(1000/portTICK_PERIOD_MS);
				ESP_LOGI("mainTask", "PDO end ");
				*/


				//Lectura objeto diccionario
				ESP_LOGI("mainTask","Leyendo objeto diccionario");
				ESP_LOGI("mainTask","Respuesta inicio %d ",CO_SDO_initTransfer(CO->SDO[0],0x1005,0));
				ESP_LOGI("mainTask","Respuesta lectura %d ",CO_SDO_readOD(CO->SDO[0], sizeof(CO->SDO[0]->ODF_arg.data)));
				ESP_LOGI("mainTask","OD: %d ", CO->SDO[0]->ODF_arg.data[0]);

				//Escritura objeto diccionario
				ESP_LOGI("mainTask","Escribiendo objeto diccionario");
				ESP_LOGI("mainTask","Respuesta inicio %d ",CO_SDO_initTransfer(CO->SDO[0],0x6205,0x0));
				CO->SDO[0]->ODF_arg.data[0] = 1;
				ESP_LOGI("mainTask","Respuesta escritura %d ",CO_SDO_writeOD(CO->SDO[0], sizeof(CO->SDO[0]->ODF_arg.data)));
				ESP_LOGI("mainTask","OD: %d ", CO->SDO[0]->ODF_arg.data[0]);

				
				while (reset == CO_RESET_NOT)
				{
						/* loop for normal program execution ******************************************/
						uint32_t coInterruptCounterCopy;
						uint32_t coInterruptCounterDiff;
						coInterruptCounterCopy = coInterruptCounter;
						coInterruptCounterDiff = coInterruptCounterCopy - coInterruptCounterPrevious;
						coInterruptCounterPrevious = coInterruptCounterCopy;

						/* CANopen process */
						reset = CO_process(CO, coInterruptCounterDiff, NULL);
						

						/* Wait */
						vTaskDelay(MAIN_WAIT / portTICK_PERIOD_MS);
				}
		}
		/* program exit
		 * ***************************************************************/
		/* reset */
		esp_restart();
}

/* CanOpen-Task executes in constant intervals ********************************/
static void coMainTask(void *arg)
{
		coInterruptCounter++;
		if (CO->CANmodule[0]->CANnormal)
		{
				bool_t syncWas;

				/* Process Sync */
				syncWas = CO_process_SYNC(CO, CO_MAIN_TASK_INTERVAL);
				/* Read inputs */
				CO_process_RPDO(CO, syncWas);

				/* Write outputs */
				CO_process_TPDO(CO, syncWas, CO_MAIN_TASK_INTERVAL);
		}
}

void app_main()
{
		xTaskCreate(&mainTask, "mainTask", 4096, NULL, 5, NULL);
}
