/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpdma.c
  * @brief   This file provides code for the configuration
  *          of the GPDMA instances.
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
#include "gpdma.h"

/* USER CODE BEGIN 0 */
#include "spi.h"
#include "dcmi.h"
#include "linked_list.h"
extern DMA_NodeTypeDef DCMItoMemory;
extern DMA_QListTypeDef ProjectDMAQueue;
static DMA_QListTypeDef  DCMIQueue;
extern DMA_NodeTypeDef SPI_Rx_Node;
extern DMA_QListTypeDef SPI_Rx_Queue;
extern DMA_NodeTypeDef SPI_Tx_Node;
extern DMA_QListTypeDef SPI_Tx_Queue;
/* USER CODE END 0 */

DMA_HandleTypeDef handle_GPDMA1_Channel12;
DMA_HandleTypeDef handle_GPDMA1_Channel7;
DMA_HandleTypeDef handle_GPDMA1_Channel6;
DMA_HandleTypeDef handle_GPDMA1_Channel12;
DMA_HandleTypeDef handle_GPDMA1_Channel7;
DMA_HandleTypeDef handle_GPDMA1_Channel6;

/* GPDMA1 init function */
void MX_GPDMA1_Init(void)
{

  /* USER CODE BEGIN GPDMA1_Init 0 */

  /* USER CODE END GPDMA1_Init 0 */

  /* Peripheral clock enable */
  __HAL_RCC_GPDMA1_CLK_ENABLE();

  /* GPDMA1 interrupt Init */
    HAL_NVIC_SetPriority(GPDMA1_Channel6_IRQn, 9, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel6_IRQn);
    HAL_NVIC_SetPriority(GPDMA1_Channel7_IRQn, 9, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel7_IRQn);
    HAL_NVIC_SetPriority(GPDMA1_Channel12_IRQn, 12, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel12_IRQn);

  /* USER CODE BEGIN GPDMA1_Init 1 */

  /* USER CODE END GPDMA1_Init 1 */
  handle_GPDMA1_Channel12.Instance = GPDMA1_Channel12;
  handle_GPDMA1_Channel12.InitLinkedList.Priority = DMA_LOW_PRIORITY_LOW_WEIGHT;
  handle_GPDMA1_Channel12.InitLinkedList.LinkStepMode = DMA_LSM_FULL_EXECUTION;
  handle_GPDMA1_Channel12.InitLinkedList.LinkAllocatedPort = DMA_LINK_ALLOCATED_PORT1;
  handle_GPDMA1_Channel12.InitLinkedList.TransferEventMode = DMA_TCEM_LAST_LL_ITEM_TRANSFER;
  handle_GPDMA1_Channel12.InitLinkedList.LinkedListMode = DMA_LINKEDLIST_CIRCULAR;
  if (HAL_DMAEx_List_Init(&handle_GPDMA1_Channel12) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel12, DMA_CHANNEL_PRIV) != HAL_OK)
  {
    Error_Handler();
  }
  handle_GPDMA1_Channel7.Instance = GPDMA1_Channel7;
  handle_GPDMA1_Channel7.InitLinkedList.Priority = DMA_LOW_PRIORITY_HIGH_WEIGHT;
  handle_GPDMA1_Channel7.InitLinkedList.LinkStepMode = DMA_LSM_FULL_EXECUTION;
  handle_GPDMA1_Channel7.InitLinkedList.LinkAllocatedPort = DMA_LINK_ALLOCATED_PORT1;
  handle_GPDMA1_Channel7.InitLinkedList.TransferEventMode = DMA_TCEM_LAST_LL_ITEM_TRANSFER;
  handle_GPDMA1_Channel7.InitLinkedList.LinkedListMode = DMA_LINKEDLIST_NORMAL;
  if (HAL_DMAEx_List_Init(&handle_GPDMA1_Channel7) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel7, DMA_CHANNEL_PRIV) != HAL_OK)
  {
    Error_Handler();
  }
  handle_GPDMA1_Channel6.Instance = GPDMA1_Channel6;
  handle_GPDMA1_Channel6.InitLinkedList.Priority = DMA_LOW_PRIORITY_HIGH_WEIGHT;
  handle_GPDMA1_Channel6.InitLinkedList.LinkStepMode = DMA_LSM_FULL_EXECUTION;
  handle_GPDMA1_Channel6.InitLinkedList.LinkAllocatedPort = DMA_LINK_ALLOCATED_PORT1;
  handle_GPDMA1_Channel6.InitLinkedList.TransferEventMode = DMA_TCEM_LAST_LL_ITEM_TRANSFER;
  handle_GPDMA1_Channel6.InitLinkedList.LinkedListMode = DMA_LINKEDLIST_NORMAL;
  if (HAL_DMAEx_List_Init(&handle_GPDMA1_Channel6) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel6, DMA_CHANNEL_PRIV) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN GPDMA1_Init 2 */
  MX_ProjectDMAQueue_Config();
  HAL_DMAEx_List_LinkQ(&handle_GPDMA1_Channel12, &ProjectDMAQueue);
  __HAL_LINKDMA(&hdcmi, DMA_Handle, handle_GPDMA1_Channel12);

  MX_SPI_Rx_Queue_Config();
  HAL_DMAEx_List_LinkQ(&handle_GPDMA1_Channel6, &SPI_Rx_Queue);
  __HAL_LINKDMA(&hspi2, hdmarx, handle_GPDMA1_Channel6);

  MX_SPI_Tx_Queue_Config();
  HAL_DMAEx_List_LinkQ(&handle_GPDMA1_Channel7, &SPI_Tx_Queue);
  __HAL_LINKDMA(&hspi2, hdmatx, handle_GPDMA1_Channel7);
  //HAL_DMAEx_List_Start(&handle_GPDMA1_Channel12);

  /* USER CODE END GPDMA1_Init 2 */

}

/* USER CODE BEGIN 1 */
void DCMI_DMA_Init(void){

	  __HAL_RCC_DCMI_PSSI_CLK_ENABLE();
	  __HAL_RCC_GPDMA1_CLK_ENABLE();
	  static DMA_NodeTypeDef DCMINode1;
	  static DMA_NodeTypeDef DCMINode2;
	  DMA_NodeConfTypeDef node_config;
	  node_config.NodeType                            = DMA_GPDMA_2D_NODE;
	  /* Set common node parameters */
	  node_config.Init.Request                        = GPDMA1_REQUEST_DCMI;
	  node_config.Init.BlkHWRequest                   = DMA_BREQ_SINGLE_BURST;
	  node_config.Init.Direction                      = DMA_PERIPH_TO_MEMORY;
	  node_config.Init.SrcInc                         = DMA_SINC_FIXED;
	  node_config.Init.DestInc                        = DMA_DINC_INCREMENTED;
	  node_config.Init.SrcDataWidth                   = DMA_SRC_DATAWIDTH_WORD;
	  node_config.Init.DestDataWidth                  = DMA_DEST_DATAWIDTH_WORD;
	  node_config.Init.SrcBurstLength                 = 1;
	  node_config.Init.DestBurstLength                = 1;
	  node_config.Init.TransferAllocatedPort          = DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT1;
	  node_config.Init.TransferEventMode              = DMA_TCEM_EACH_LL_ITEM_TRANSFER;
	  /* Set node data handling parameters */
	  node_config.DataHandlingConfig.DataExchange     = DMA_EXCHANGE_NONE;
	  node_config.DataHandlingConfig.DataAlignment    = DMA_DATA_RIGHTALIGN_ZEROPADDED;
	  /* Set node trigger parameters */
	  node_config.TriggerConfig.TriggerPolarity       = DMA_TRIG_POLARITY_MASKED;
	  /* Set node repeated block parameters */
	  node_config.RepeatBlockConfig.RepeatCount       = 1U;
	  node_config.RepeatBlockConfig.SrcAddrOffset     = 0;
	  node_config.RepeatBlockConfig.DestAddrOffset    = 0;
	  node_config.RepeatBlockConfig.BlkSrcAddrOffset  = 0;
	  node_config.RepeatBlockConfig.BlkDestAddrOffset = 0;

	  /* Build DCMINode1 */
	  HAL_DMAEx_List_BuildNode(&node_config, &DCMINode1);

	  /* Insert DCMINode1 to DCMI queue */
	  HAL_DMAEx_List_InsertNode_Tail(&DCMIQueue, &DCMINode1);

	  /* Build DCMINode2 */
	  HAL_DMAEx_List_BuildNode(&node_config, &DCMINode2);

	  /* Insert DCMINode2 to DCMI queue */
	  HAL_DMAEx_List_InsertNode_Tail(&DCMIQueue, &DCMINode2);

	  /* Set queue circular mode for DCMI queue */
	  HAL_DMAEx_List_SetCircularMode(&DCMIQueue);

	  /*** Configure the DMA channel ***/
	  handle_GPDMA1_Channel12.Instance                         = GPDMA1_Channel12;
	  handle_GPDMA1_Channel12.InitLinkedList.Priority          = DMA_LOW_PRIORITY_LOW_WEIGHT;
	  handle_GPDMA1_Channel12.InitLinkedList.LinkStepMode      = DMA_LSM_FULL_EXECUTION;
	  handle_GPDMA1_Channel12.InitLinkedList.LinkAllocatedPort = DMA_LINK_ALLOCATED_PORT1;
	  handle_GPDMA1_Channel12.InitLinkedList.LinkedListMode    = DMA_LINKEDLIST_CIRCULAR;
	  handle_GPDMA1_Channel12.InitLinkedList.TransferEventMode = DMA_TCEM_LAST_LL_ITEM_TRANSFER;

	  /* DMA linked list init */
	  HAL_DMAEx_List_Init(&handle_GPDMA1_Channel12);

	  /* Link DCMI queue to DMA channel */
	  HAL_DMAEx_List_LinkQ(&handle_GPDMA1_Channel12, &DCMIQueue);

	  /* Associate the initialized DMA handle to the DCMI handle */
	  __HAL_LINKDMA(&hdcmi, DMA_Handle, handle_GPDMA1_Channel12);

	  /*** Configure the NVIC for DCMI and DMA ***/
	  /* NVIC configuration for DCMI transfer complete interrupt */
	  HAL_NVIC_SetPriority(DCMI_PSSI_IRQn, 10, 0);
	  HAL_NVIC_EnableIRQ(DCMI_PSSI_IRQn);

	  /* NVIC configuration for GPDMA1 transfer complete interrupt */
	  HAL_NVIC_SetPriority(GPDMA1_Channel12_IRQn, 10, 0);
	  HAL_NVIC_EnableIRQ(GPDMA1_Channel12_IRQn);
}
/* USER CODE END 1 */
