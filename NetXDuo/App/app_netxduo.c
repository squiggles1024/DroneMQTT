/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_netxduo.c
  * @author  MCD Application Team
  * @brief   NetXDuo applicative file
  ******************************************************************************
    * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_netxduo.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_azure_rtos.h"
#include "nx_ip.h"
#include "app_threadx.h"
#include "BSP_camera.h"
#include CERT_FILE
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*********************************************************************************************************************************************/
//Private Defines
#define STRLEN(p)                   (sizeof(p) - 1) //Used to calculate lengths of MQTT Topic Strings
#define MQTT_TopicBufferLen         (32)
#define MQTT_MessageBufferLen       (32)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/*********************************************************************************************************************************************/
//Memory pool where packets are stored.
NX_PACKET_POOL PacketPool;
/*********************************************************************************************************************************************/
//Required TCP/IP Client Objects
NX_IP           IP;
NX_DHCP         DHCPClient;
NX_SNTP_CLIENT  SNTPClient;
NXD_MQTT_CLIENT MQTTClient;
static NX_DNS   DNSClient;

/*********************************************************************************************************************************************/
//Internet Based Thread Pointers
TX_THREAD IPInitThread, SNTPInitThread, MQTTThread; //Thread pointers

/*********************************************************************************************************************************************/
//Flow Control Variables
TX_SEMAPHORE IP_ADDR_SEM;
TX_SEMAPHORE SNTP_UPDATE_SEM;
TX_MUTEX SPI_MUTEX;
TX_EVENT_FLAGS_GROUP MQTT_TREvent;

/*********************************************************************************************************************************************/
//Static Globals
static ULONG CurrentTime;
static ULONG MqttClientStack[MQTT_CLIENT_STACK_SIZE];
static CHAR CryptoMetadataClient[11600];
static UCHAR TLSPacketBuffer[4000];
extern const NX_SECURE_TLS_CRYPTO nx_crypto_tls_ciphers;
static UCHAR MQTT_TopicBuffer[MQTT_TopicBufferLen];
static UCHAR MQTT_MessageBuffer[MQTT_MessageBufferLen];

/*********************************************************************************************************************************************/
//Imported Variables from security certificate files
extern unsigned int mosquitto_der_len; //Length of certificate
extern unsigned char mosquitto_der[];  //der encoded x509 certificate exported as a .h file.

TX_QUEUE ControllerQueue;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
/*********************************************************************************************************************************************/
//Private Functions
static VOID IP_Init(ULONG input);
static VOID SNTP_Init(ULONG input);
static VOID MQTT_Loop(ULONG input);
static VOID IP_Address_Changed_CB(NX_IP *IPInstance, VOID *Input);
static VOID Time_Update_CB(NX_SNTP_TIME_MESSAGE  *TimeMsg, NX_SNTP_TIME *LocalTime);
static VOID MQTTMessageRecieved_CB(NXD_MQTT_CLIENT* Client, UINT MsgCount);
static VOID MQTTDisconnected_CB(NXD_MQTT_CLIENT *Client);
static UINT TLS_Init(NXD_MQTT_CLIENT *Client, NX_SECURE_TLS_SESSION *TLS_Session, NX_SECURE_X509_CERT *Cert, NX_SECURE_X509_CERT *TrustedCert);
static ULONG GetSessionTime(void);
/*********************************************************************************************************************************************/
/* USER CODE END PFP */
/**
  * @brief  Application NetXDuo Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT MX_NetXDuo_Init(VOID *memory_ptr)
{
  UINT ret = NX_SUCCESS;
  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;

   /* USER CODE BEGIN App_NetXDuo_MEM_POOL */
  CHAR* Ptr;
  ret = tx_byte_allocate(byte_pool, (VOID**) &Ptr, PACKET_POOL_SIZE, TX_NO_WAIT);                           //Allocate Memory for Packet Pool
  if(ret != TX_SUCCESS)
  {
	  return TX_POOL_ERROR;
  }

  ret = nx_packet_pool_create(&PacketPool, "Main Network Packet Pool", PAYLOAD_SIZE, Ptr, PACKET_POOL_SIZE); //Create the packet pool

  if(ret != NX_SUCCESS)
  {
	  return ret;
  }
  /* USER CODE END App_NetXDuo_MEM_POOL */

  /* USER CODE BEGIN MX_NetXDuo_Init */
  ret = tx_byte_allocate(byte_pool, (VOID**)&Ptr, IP_HELPER_STACK_SIZE, TX_NO_WAIT);                        //Allocate memory for IP Instance
  if(ret != TX_SUCCESS)
  {
	  return TX_POOL_ERROR;
  }

  ret = nx_ip_create(&IP,                      //IP Object                                                  //Create IP Instance
		             "IP Instance",            //Name
					 NULL_ADDRESS,             //IP Address
					 NULL_ADDRESS,             //Net Mask
					 &PacketPool,              //Packet Pool
					 nx_driver_emw3080_entry,  //IP Network Driver
					 Ptr,                      //Memory Ptr
					 IP_HELPER_STACK_SIZE,     //Memory Size
					 15);                      //Priority of IP Helper thread

  if(ret != NX_SUCCESS)
  {
	  return NX_NOT_ENABLED;
  }

  ret = tx_byte_allocate(byte_pool, (VOID**) &Ptr, ARP_STACK_SIZE, TX_NO_WAIT);                            //Allocate memory for ARP
  if(ret != TX_SUCCESS)
  {
	  return TX_POOL_ERROR;
  }

  ret = nx_arp_enable(&IP, (VOID**)Ptr, ARP_STACK_SIZE);                                                   //Enable Arp: Required to resolve network interface address (MAC Address)
  if(ret != NX_SUCCESS)
  {
	  return ret;
  }

  ret = nx_dhcp_create(&DHCPClient, &IP, "DHCP Client");                                                  //Create DHCP Client: Required for network gateway (my router) to assign this device an IP address.

  if(ret != NX_SUCCESS)
  {
	  return ret;
  }

  ret = nx_icmp_enable(&IP);                                                                              //Enable ICMP Protocol: Required by most TCP/IP stacks to receive/decode error messages.
  if(ret != NX_SUCCESS)
  {
	  return ret;
  }

  ret = nx_udp_enable(&IP);                                                                               //Enable UDP Required which is required for DHCP, and (in some cases) SNTP
  if(ret != NX_SUCCESS)
  {
	  return ret;
  }

  ret = nx_tcp_enable(&IP);                                                                               //Enable TCP: Required for MQTT over TLS and DNS
  if(ret != NX_SUCCESS)
  {
	  return ret;
  }

  ret = tx_byte_allocate(byte_pool, (VOID**)&Ptr, MQTT_STACK_SIZE, TX_NO_WAIT);                           //Allocate memory for MQTT Thread Stack
  if(ret != TX_SUCCESS)
  {
	  return ret;
  }

  ret = tx_thread_create(&MQTTThread,                                                                     //Create MQTT Thread, which is used to transmit/receive sensor data over MQTT.
		                 "MQTT Thread",
						 MQTT_Loop,
						 0,
						 Ptr,
						 MQTT_STACK_SIZE,
						 SEND_ENVDATA_PRIORITY,
						 SEND_ENVDATA_PRIORITY,
						 TX_NO_TIME_SLICE,
						 TX_DONT_START);

  ret = tx_event_flags_create(&MQTT_TREvent, "MQTT TxRx Event");                                          //Create Event group used to signal to MQTT Thread that data is ready for transfer.

  ret = tx_byte_allocate(byte_pool, (VOID**) &Ptr, IP_INIT_STACK_SIZE, TX_NO_WAIT);                       //Allocate memory for IP Init thread stack
  if(ret != TX_SUCCESS)
  {
	  return ret;
  }

  ret = tx_thread_create(&IPInitThread,                                                                   //Create IP Initialization thread
		                 "IP Init Thread",
						 IP_Init,
                         0,
						 Ptr,
						 IP_INIT_STACK_SIZE,
						 IP_INIT_PRIORITY,
						 IP_INIT_PRIORITY,
						 TX_NO_TIME_SLICE,
						 TX_AUTO_START);

  ret = tx_byte_allocate(byte_pool, (VOID**)&Ptr, SNTP_STACK_SIZE, TX_NO_WAIT);                           //Allocate memory for SNTP Init Thread stack size
  if(ret != TX_SUCCESS)
  {
	  return ret;
  }

  ret = tx_thread_create(&SNTPInitThread,                                                                 //Create SNTP Initialization thread
		                 "SNTP Init Thread",
						 SNTP_Init,
						 0,
						 Ptr,
						 SNTP_STACK_SIZE,
						 SNTP_INIT_PRIORITY,
						 SNTP_INIT_PRIORITY,
						 TX_NO_TIME_SLICE,
						 TX_DONT_START);
  if(ret != TX_SUCCESS)
  {
	  return ret;
  }
  ret = tx_mutex_create(&SPI_MUTEX,"SPI Mutex", TX_INHERIT);                                                  //Create SPI Mutex to prevent MQTT Recieve/Transmit threads from clashing.

  if(tx_byte_allocate(byte_pool, (VOID **) &Ptr, 100, TX_NO_WAIT) != TX_SUCCESS)                              //Allocate space for Controller Queue
  {
	  return TX_POOL_ERROR;
  }
  ret = tx_queue_create(&ControllerQueue, "Controller Queue", TX_4_ULONG, Ptr, CONTROLLER_QUEUE_SIZE);        //Create Controller queue
  /* USER CODE END MX_NetXDuo_Init */

  return ret;
}

/* USER CODE BEGIN 1 */
/*******************************************************************************************************************************************************************
 *@Brief: Performs initialization required for other IP-based threads to function.
 *@Param: Unused
 *@Return: None
 *******************************************************************************************************************************************************************/
static VOID IP_Init(ULONG input)
{
	  UINT ret = NX_SUCCESS;
	  ULONG AssignedIPAddress = 0;
	  ULONG SubnetMask = 0;

	  ret = nx_dns_create(&DNSClient, &IP, (UCHAR *)"DNS Client");                                      //Create a DNS instance for the Client
	  if (ret != NX_SUCCESS)
	  {
	      Error_Handler();
	  }

	  ret = nx_dns_server_add(&DNSClient, DNS_ADDRESS);                                                 //Initialize DNS instance with a dummy server
	  if (ret != NX_SUCCESS)
	  {
	      Error_Handler();
	  }

      tx_semaphore_create(&IP_ADDR_SEM, "IP Address Semaphore", 0);                                     //Create Semaphore to wait for DHCP to Assign IP Address

	  ret = nx_ip_address_change_notify(&IP, IP_Address_Changed_CB, NULL);                              //Link callback function to run when IP Address is assigned.
	  if (ret != NX_SUCCESS)
	  {
	      Error_Handler();
	  }

	  ret = nx_dhcp_start(&DHCPClient);                                                                 //Start DHCP Client
	  if (ret != NX_SUCCESS)
	  {
	      Error_Handler();
	  }


	  if(tx_semaphore_get(&IP_ADDR_SEM, TX_WAIT_FOREVER) != TX_SUCCESS)                                 //Wait for IP Address to be assigned.
	  {
		  Error_Handler();
	  }


	  ret = nx_ip_address_get(&IP, &AssignedIPAddress, &SubnetMask);                                    //Retrieve IP Address from Network IF
	  if (ret != NX_SUCCESS)
	  {
	      Error_Handler();
	  }


      printf("IP Address Assigned: %lu\r\n", AssignedIPAddress);
	  tx_thread_resume(&SNTPInitThread);                                                                //Initiate the SNTP Thread.

	  tx_thread_terminate(&IPInitThread);                                                               //Kill the thread as its not needed anymore.
}

/*******************************************************************************************************************************************************************
 *@Brief: Fetches current time from SNTP Server (google). This is required for
 *        TLS, as security certificates are only valid for a finite amount of time.
 *@Param: Unused
 *@Return: None
 *******************************************************************************************************************************************************************/
static VOID SNTP_Init(ULONG input)
{
	UINT ret = NX_SUCCESS;
	UINT ServerStatus;
	ULONG Fraction;
	NXD_ADDRESS SNTP_Server_IP;
	SNTP_Server_IP.nxd_ip_version = 4;

	ret = nx_dns_host_by_name_get(&DNSClient, (UCHAR *)SNTP_SERVER_URL, &SNTP_Server_IP.nxd_ip_address.v4,SNTP_TIMEOUT); 	//Get time.google.com IP address from DNS
	if(ret != NX_SUCCESS)
	{
		Error_Handler();
	}

	ret = nx_sntp_client_create(&SNTPClient,&IP,0, &PacketPool, NULL,NULL,NULL);                                            //Create the SNTP client
	if(ret != NX_SUCCESS)
	{
		Error_Handler();
	}

    tx_semaphore_create(&SNTP_UPDATE_SEM, "SNTP Semaphore",0);                                                              //Create Semaphore to wait for SNTP Server response

	ret = nx_sntp_client_set_time_update_notify(&SNTPClient,Time_Update_CB);                                                //Set callback to trigger when SNTP Server responds
	if (ret != NX_SUCCESS)
	{
	    Error_Handler();
	}

	ret = nx_sntp_client_initialize_unicast(&SNTPClient, SNTP_Server_IP.nxd_ip_address.v4);                                 //Set SNTP to run in Unicast mode (one-to-one communication)
	if (ret != NX_SUCCESS)
	{
	    Error_Handler();
	}

	ret = nx_sntp_client_run_unicast(&SNTPClient);                                                                          //Start SNTP Client
	if (ret != NX_SUCCESS)
	{
	    Error_Handler();
	}

	if(tx_semaphore_get(&SNTP_UPDATE_SEM, TX_WAIT_FOREVER) != TX_SUCCESS)                                                  	//Wait SNTP to respond with time
	{
		Error_Handler();
	}

	ret = nx_sntp_client_receiving_updates(&SNTPClient,&ServerStatus);                                                      //Ensure updates from SNTP client are valid
	if(ret != NX_SUCCESS || ServerStatus == NX_FALSE)
	{
		Error_Handler();
	}

    ret = nx_sntp_client_get_local_time_extended(&SNTPClient, &CurrentTime, &Fraction, NX_NULL, 0);                         //Fetch the local time and store it in "CurrentTime"
	if(ret != NX_SUCCESS)
	{
		Error_Handler();
	}

	nx_sntp_client_stop(&SNTPClient);                                                                                       //Stop SNTP Client: Its no longer needed.

    /* take off 70 years difference */
    CurrentTime = CurrentTime - EPOCH_TIME_DIFF;

    tx_thread_resume(&MQTTThread);                                                                                          //Start MQTT Client Thread
    tx_thread_terminate(&SNTPInitThread);                                                                                   //Kill this thread, it's no longer needed.
}


/*******************************************************************************************************************************************************************
 *@Brief: Callback which is called when an IP Address is assigned.
 *        Signals semaphore allowing thread to continue.
 *@Param: Unused
 *@Return: None
 *******************************************************************************************************************************************************************/
static VOID IP_Address_Changed_CB(NX_IP *IPInstance, VOID *Input)
{
	tx_semaphore_put(&IP_ADDR_SEM);
}

/*******************************************************************************************************************************************************************
 *@Brief: Callback which is called when an SNTP Server gives time.
 *        Signals semaphore allowing thread to continue.
 *@Param: Unused
 *@Return: None
 *******************************************************************************************************************************************************************/
static VOID Time_Update_CB(NX_SNTP_TIME_MESSAGE  *TimeMsg, NX_SNTP_TIME *LocalTime)
{
	tx_semaphore_put(&SNTP_UPDATE_SEM);
}

/*******************************************************************************************************************************************************************
 *@Brief: Callback which is called when the MQTT Broker sends a message.
 *        The only message the MQTT Client should be is "Drone/Control" messages.
 *        The thread will retrieve all messages, and discard all of them except for the most
 *        recent.
 *@Param: Unused
 *@Return: None
 *******************************************************************************************************************************************************************/
static VOID MQTTMessageRecieved_CB(NXD_MQTT_CLIENT* Client, UINT MsgCount)
{
    UINT Remaining = MsgCount;
	UINT ActualTopicLength = 0;
	UINT ActualMessageLength = 0;
    static ULONG MessageCounter = 0;
	UINT ret = NX_SUCCESS;
	DroneSetpoint_t *SetPoint;
	DroneSetpoint_t Dummy;
    while(Remaining > 0)
    {
    	tx_mutex_get(&SPI_MUTEX,TX_NO_WAIT);                             //Take SPI Mutex
    	ret = nxd_mqtt_client_message_get(&MQTTClient,                   //Retrieve Message
    			                          MQTT_TopicBuffer,
										  MQTT_TopicBufferLen,
										  &ActualTopicLength,
										  MQTT_MessageBuffer,
										  MQTT_MessageBufferLen,
										  &ActualMessageLength
										  );
    	tx_mutex_put(&SPI_MUTEX);                                        //Put mutex back
    	if(ret != NXD_MQTT_SUCCESS)
    	{
    		Error_Handler();
    	}
    	//printf("Message %lu received: TOPIC = %s, MESSAGE = %s\n", MessageCounter + 1, MQTT_TopicBuffer, MQTT_MessageBuffer);
    	Remaining--;
    	MessageCounter++;
    }
    SetPoint = (DroneSetpoint_t*)MQTT_MessageBuffer;
    if(ControllerQueue.tx_queue_available_storage == 0)
    {
    	tx_queue_receive(&ControllerQueue, &Dummy,TX_NO_WAIT);
    }
    tx_queue_send(&ControllerQueue, SetPoint, TX_WAIT_FOREVER);
    //Push Controller Data Here
}

/*******************************************************************************************************************************************************************
 *@Brief: The main MQTT Client Loop.
 *        Thread initializes the MQTT Client and then checks the event flags to verify if
 *        there are messages ready to send.
 *@Param: Unused
 *@Return: None
 *******************************************************************************************************************************************************************/
static VOID MQTT_Loop(ULONG input)
{
	UINT ret = NX_SUCCESS;
	ULONG MQTTEvents = 0;
	NXD_ADDRESS MQTTServerIP;
	ULONG *CameraBufferPtr;
	ULONG CameraQueueReceiver = 0;

	float QueueRecieveBuffer[MAGNETIC_PAYLOAD_SAMPLES*3];                          //Buffer is used to fetch data from Sensor queues. Must be large enough to store 10 samples from each magnetometer axis.
	MQTTServerIP.nxd_ip_address.v4 = MQTT_SERVER_IP;                               //Set IP Address of MQTT Broker
	MQTTServerIP.nxd_ip_version = 4;                                               //Set IP Version to use (IPv4, as the broker is on my local network)

	ret = nxd_mqtt_client_create(&MQTTClient,                                      //Create the MQTT Client
			                     "MQTT Client",
			                     "Drone Client ID",
								 STRLEN("Drone Client ID"),
								 &IP,
								 &PacketPool,
								 (VOID*) MqttClientStack,
								 MQTT_CLIENT_STACK_SIZE,
								 MQTT_CLIENT_PRIORITY,
								 NX_NULL,
								 0);
	if(ret != NX_SUCCESS)
	{
		Error_Handler();
	}

	nxd_mqtt_client_receive_notify_set(&MQTTClient, MQTTMessageRecieved_CB);      //Set Callback function to be called when a message is recieved.
	nxd_mqtt_client_disconnect_notify_set(&MQTTClient, MQTTDisconnected_CB);      //Set Callback function to be called when the client disconnects (NYI)

	if(ret != NX_SUCCESS)
	{
		Error_Handler();
	}


	ret = nxd_mqtt_client_secure_connect(&MQTTClient,                             //Connect to MQTT Broker over TLS
			                             &MQTTServerIP,
									     NXD_MQTT_TLS_PORT,
										 TLS_Init,                                //Function to be called to setup TLS
										 MQTT_KEEPALIVE_TIMER,
										 NX_TRUE,
										 NX_WAIT_FOREVER);
	if(ret != NX_SUCCESS)
	{
		Error_Handler();
	}

	nxd_mqtt_client_subscribe(&MQTTClient, MQTT_CLIENT_SUB_TOPIC01, STRLEN(MQTT_CLIENT_SUB_TOPIC01), 1);   //Subscribe to Drone Controls Topic

    while(1)
    {
    	tx_event_flags_get(&MQTT_TREvent, MESSAGE_ALL_EVT_Msk, TX_OR_CLEAR, &MQTTEvents, TX_WAIT_FOREVER); //Fetch event flags
    	if(MQTTEvents & MESSAGE_TRANSMIT_PUB01_EVT_Msk)
    	{
    		for(uint8_t i = 0; i < TEMPERATURE_PAYLOAD_SAMPLES; i++)
    		{
                tx_queue_receive(&TemperatureQueue,(VOID*)&QueueRecieveBuffer[i],TX_WAIT_FOREVER);         //Retrieve sample(s) from Queue
    		}
            tx_mutex_get(&SPI_MUTEX,TX_WAIT_FOREVER);                                                      //Take SPI Mutex
            nxd_mqtt_client_publish(&MQTTClient, MQTT_CLIENT_PUB_TOPIC01,                                  //Publish Temperature data
            		                STRLEN(MQTT_CLIENT_PUB_TOPIC01),
									(VOID*)QueueRecieveBuffer,
									TEMPERATURE_QUEUE_SIZE,
									NX_TRUE,
									0,
									NX_WAIT_FOREVER);
            tx_mutex_put(&SPI_MUTEX);                                                                      //Put Mutex Back
    	}

    	if(MQTTEvents & MESSAGE_TRANSMIT_PUB02_EVT_Msk)
    	{
    		for(uint8_t i = 0; i < HUMIDITY_PAYLOAD_SAMPLES; i++)
    		{
                tx_queue_receive(&HumidityQueue,(VOID*)&QueueRecieveBuffer[i],TX_WAIT_FOREVER);            //Retrieve sample(s) from Queue
    		}
    		tx_mutex_get(&SPI_MUTEX,TX_WAIT_FOREVER);                                                      //Take SPI Mutex
            nxd_mqtt_client_publish(&MQTTClient, MQTT_CLIENT_PUB_TOPIC02,                                  //Publish humidity data
            		                STRLEN(MQTT_CLIENT_PUB_TOPIC02),
									(VOID*)QueueRecieveBuffer,
									HUMIDITY_QUEUE_SIZE,
									NX_TRUE,
									0,
									NX_WAIT_FOREVER);
            tx_mutex_put(&SPI_MUTEX);                                                                      //Put Mutex Back
    	}

    	if(MQTTEvents & MESSAGE_TRANSMIT_PUB03_EVT_Msk)
    	{
    		for(uint8_t i = 0; i < PRESSURE_PAYLOAD_SAMPLES; i++)
    		{
                tx_queue_receive(&PressureQueue,(VOID*)&QueueRecieveBuffer[i],TX_WAIT_FOREVER);            //Retrieve sample(s) from Queue
    		}
            tx_queue_receive(&PressureQueue,(VOID*)QueueRecieveBuffer,TX_WAIT_FOREVER);
            tx_mutex_get(&SPI_MUTEX,TX_WAIT_FOREVER);                                                      //Take SPI Mutex
            nxd_mqtt_client_publish(&MQTTClient, MQTT_CLIENT_PUB_TOPIC03,                                  //Publish pressure data
            		                STRLEN(MQTT_CLIENT_PUB_TOPIC03),
									(VOID*)QueueRecieveBuffer,
									PRESSURE_QUEUE_SIZE,
									NX_TRUE,
									0,
									NX_WAIT_FOREVER);
            tx_mutex_put(&SPI_MUTEX);                                                                      //Put Mutex Back
    	}

    	if(MQTTEvents & MESSAGE_TRANSMIT_PUB04_EVT_Msk)
    	{
    		for(uint8_t i = 0; i < LIGHT_PAYLOAD_SAMPLES; i++)
    		{
                tx_queue_receive(&LightQueue,(VOID*)&QueueRecieveBuffer[i],TX_WAIT_FOREVER);               //Retrieve sample(s) from Queue
    		}
            tx_queue_receive(&LightQueue,(VOID*)QueueRecieveBuffer,TX_WAIT_FOREVER);
            tx_mutex_get(&SPI_MUTEX,TX_WAIT_FOREVER);                                                      //Take SPI Mutex
            nxd_mqtt_client_publish(&MQTTClient, MQTT_CLIENT_PUB_TOPIC04,                                  //Publish light data
            		                STRLEN(MQTT_CLIENT_PUB_TOPIC04),
									(VOID*)QueueRecieveBuffer,
									LIGHT_QUEUE_SIZE,
									NX_TRUE,
									0,
									NX_WAIT_FOREVER);
            tx_mutex_put(&SPI_MUTEX);                                                                      //Put Mutex Back
    	}

    	if(MQTTEvents & MESSAGE_TRANSMIT_PUB05_EVT_Msk)
    	{
    		for(uint8_t i = 0; i < MAGNETIC_PAYLOAD_SAMPLES; i++)
    		{
                tx_queue_receive(&MagXQueue,(VOID*)&QueueRecieveBuffer[i],TX_WAIT_FOREVER);               //Retrieve sample(s) from Queue
                tx_queue_receive(&MagYQueue,(VOID*)&QueueRecieveBuffer[i + 10],TX_WAIT_FOREVER);
                tx_queue_receive(&MagZQueue,(VOID*)&QueueRecieveBuffer[i + 20],TX_WAIT_FOREVER);
    		}
    		tx_mutex_get(&SPI_MUTEX,TX_WAIT_FOREVER);                                                     //Take SPI Mutex
            nxd_mqtt_client_publish(&MQTTClient, MQTT_CLIENT_PUB_TOPIC05,                                 //Publish magnetic field data
            		                STRLEN(MQTT_CLIENT_PUB_TOPIC05),
									(VOID*)QueueRecieveBuffer,
									MAGNETIC_QUEUE_SIZE*3,
									NX_TRUE,
									0,
									NX_WAIT_FOREVER);
            tx_mutex_put(&SPI_MUTEX);                                                                     //Put Mutex Back
    	}

    	if(MQTTEvents & MESSAGE_TRANSMIT_PUB11_EVT_Msk)
    	{
    		tx_queue_receive(&CameraQueue, &CameraQueueReceiver,TX_WAIT_FOREVER);                         //Retrieve sample(s) from Queue
    		CameraBufferPtr = (VOID*)CameraQueueReceiver;                                                 //Cast number fetched from queue into a ULONG pointer.
    		tx_mutex_get(&SPI_MUTEX,TX_WAIT_FOREVER);                                                     //Take SPI Mutex
    		for(uint8_t i = 0; i < 10; i++)
    		{
                nxd_mqtt_client_publish(&MQTTClient, MQTT_CLIENT_PUB_TOPIC11,                             //Publish camera data 1 slice at a time.
                		                STRLEN(MQTT_CLIENT_PUB_TOPIC11),                                  //OpenSSL bug requires the data be sliced into chunks less than 16320 bytes.
										(VOID*) (CameraBufferPtr),                                        //TLS Has a record limit of 16KB
										CAMERA_DATA_SIZE_BYTES / 10,
    									NX_TRUE,
    									0,
    									NX_WAIT_FOREVER);
                CameraBufferPtr = CameraBufferPtr + ((CAMERA_DATA_SIZE_BYTES / 10) / 4);                  //Fetch address of the next image data chunk
    		}
    		/*
            nxd_mqtt_client_publish(&MQTTClient, MQTT_CLIENT_PUB_TOPIC11,                                 //TLS has a record limit of 16KB
            		                STRLEN(MQTT_CLIENT_PUB_TOPIC11),
									(VOID*) (CameraBufferPtr),
									CAMERA_DATA_SIZE_BYTES,
									NX_TRUE,
									0,
									NX_WAIT_FOREVER);
			*/
    		tx_mutex_put(&SPI_MUTEX);                                                                      //Put Mutex Back
    		CameraBufferPtr = CameraBufferPtr - (CAMERA_DATA_SIZE_BYTES / 4);                              //Reset Frame buffer back to initial state.
    		if(CameraQueueReceiver == CAMERA_FRAMEBUFFER1_ADDR)                                            //Signal semaphore based upon which frame was transmitted.
    		{
    			tx_semaphore_put(&CameraSendFrame[0]);
    		}else
    		{
    			tx_semaphore_put(&CameraSendFrame[1]);
    		}
    	}

    	tx_thread_suspend(&MQTTThread);                                                                    //All messages have been transmitted, suspend thread.
    }
}

/*******************************************************************************************************************************************************************
 *@Brief: Callback function called when MQTT Client disconnects. Not yet implemented.
 *@Param: Unused
 *@Return: None
 *******************************************************************************************************************************************************************/
static VOID MQTTDisconnected_CB(NXD_MQTT_CLIENT *Client)
{
  //Handle Disconnects
  //Try to reconnect?
  //Gracefully Land?
}

/*******************************************************************************************************************************************************************
 *@Brief: Performs TLS initialization
 *@Param: Required by NetX Duo TLS Setup callback functions. (See nxd_mqtt_client_secure_connect parameters)
 *@Return: None
 *******************************************************************************************************************************************************************/
static UINT TLS_Init(NXD_MQTT_CLIENT *Client, NX_SECURE_TLS_SESSION *TLS_Session, NX_SECURE_X509_CERT *Cert, NX_SECURE_X509_CERT *TrustedCert)
{
	UINT ret = NX_SUCCESS;

	nx_secure_tls_initialize();                                                                                                       //NetX Duo TLS initialization
	ret = nx_secure_tls_session_create(TLS_Session, &nx_crypto_tls_ciphers, CryptoMetadataClient, sizeof(CryptoMetadataClient));      //Create a TLS Session
	if(ret != NX_SUCCESS)
	{
		Error_Handler();
	}

	memset(Cert, 0, sizeof(NX_SECURE_X509_CERT));                                                                                     //Clear certificate buffer

	ret = nx_secure_tls_session_time_function_set(TLS_Session,GetSessionTime);                                                        //Register callback function that returns current time (time retrieved earlier by SNTP Client)
    if(ret != NX_SUCCESS)
    {
    	Error_Handler();
    }

    ret = nx_secure_tls_session_packet_buffer_set(TLS_Session, TLSPacketBuffer, sizeof(TLSPacketBuffer));                             //Create buffer for TLS packet re-assembly
    if(ret != NX_SUCCESS)
    {
    	Error_Handler();
    }

    ret = nx_secure_x509_certificate_initialize(TrustedCert,                                                                          //Initialize x509 certificate.
    		                                   (UCHAR*)mosquitto_der,
											   mosquitto_der_len,
											   NX_NULL,
											   0,
											   NULL,
											   0,
											   NX_SECURE_X509_KEY_TYPE_NONE);
    if(ret != NX_SUCCESS)
    {
    	Error_Handler();
    }

    ret = nx_secure_tls_trusted_certificate_add(TLS_Session, TrustedCert);                                                            //Set certificate as trusted.
    if(ret != NX_SUCCESS)
    {
    	Error_Handler();
    }

    return ret;
}

/*********************************************************************************************
 *@Brief: Called from TLS init function to retrieve current time for ceriticate validation.
 *@Param: None
 *@Return: None
 *********************************************************************************************/
static ULONG GetSessionTime(void)
{
	return CurrentTime;
}

/*************************************************************************************************
 *@Brief: ISR That is called when the flow control and notify pins change on the network interface.
 *@Param: Pin which triggered the ISR
 *@Return: None
 *************************************************************************************************/
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin)
  {
    case (GPIO_PIN_1):
      break;

    case (MXCHIP_FLOW_Pin):
      mxchip_WIFI_ISR(MXCHIP_FLOW_Pin);
      nx_driver_emw3080_interrupt();
      break;

    case (MXCHIP_NOTIFY_Pin):
      mxchip_WIFI_ISR(MXCHIP_NOTIFY_Pin);
      nx_driver_emw3080_interrupt();
      break;

    default:
      break;
  }
}
/* USER CODE END 1 */
