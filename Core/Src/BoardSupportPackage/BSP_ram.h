/*
 * BSP_ram.h
 *
 *  Created on: Sep 16, 2022
 *      Author: evanl
 */

#ifndef SRC_BOARDSUPPORTPACKAGE_BSP_RAM_H_
#define SRC_BOARDSUPPORTPACKAGE_BSP_RAM_H_

#define OSPI1_RAM_BASE    (0x90000000U)
#define OSPI1_RAM_LENGTH  (0x800000)
#define OSPI1_RAM_LAST    (OSPI1_RAM_BASE_ADDR + OSPI1_RAM_LENGTH - 1)

//See BSP_Ram.c for more details.

/*
 * @brief: Initializes external RAM chip (64 Mb) in Memory Mapped mode
 */
void BSP_RamInit(void);
void BSP_RamErase(void);
//void BSP_RamTest(void);

#endif /* SRC_BOARDSUPPORTPACKAGE_BSP_RAM_H_ */
