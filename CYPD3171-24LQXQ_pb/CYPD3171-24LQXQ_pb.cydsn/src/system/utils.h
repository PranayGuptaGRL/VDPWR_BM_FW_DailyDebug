/**
 * @file utils.h
 *
 * @brief @{General utility macros and definitions for CCGx firmware stack.@}
 */

/*
 * Copyright (2014-2018), Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All rights reserved.
 *
 * This software, including source code, documentation and related materials
 * (“Software”), is owned by Cypress Semiconductor Corporation or one of its
 * subsidiaries (“Cypress”) and is protected by and subject to worldwide patent
 * protection (United States and foreign), United States copyright laws and
 * international treaty provisions. Therefore, you may use this Software only
 * as provided in the license agreement accompanying the software package from
 * which you obtained this Software (“EULA”).
 *
 * If no EULA applies, Cypress hereby grants you a personal, nonexclusive,
 * non-transferable license to copy, modify, and compile the Software source
 * code solely for use in connection with Cypress’s integrated circuit
 * products. Any reproduction, modification, translation, compilation, or
 * representation of this Software except as specified above is prohibited
 * without the express written permission of Cypress. Disclaimer: THIS SOFTWARE
 * IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress reserves the
 * right to make changes to the Software without notice. Cypress does not
 * assume any liability arising out of the application or use of the Software
 * or any product or circuit described in the Software. Cypress does not
 * authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death (“High Risk Product”). By
 * including Cypress’s product in a High Risk Product, the manufacturer of such
 * system or application assumes all risk of such use and in doing so agrees to
 * indemnify Cypress against all liability.
 */

#ifndef _UTILS_H_
#define _UTILS_H_

#include <stdint.h>
#include <string.h>

/*****************************************************************************
* MACRO Definition
*****************************************************************************/

#define GET_MAX(a,b)    (((a) > (b)) ? (a) : (b))
/**<  Get the maximum from among two numbers. */

#define GET_MIN(a,b)    (((a) > (b)) ? (b) : (a))
/**<  Get the minimum from among two numbers. */

#define DIV_ROUND_UP(x, y) (((x) + ((y) - 1)) / (y))
/**< Integer division - round up to next integer. */

#define DIV_ROUND_NEAREST(x, y) (((x) + ((y) / 2)) / (y))
/**< Integer division - round to nearest integer. */

#define MAKE_WORD(hi,lo)                        (((uint16_t)(hi) << 8) | ((uint16_t)(lo)))
/**< Combine two bytes to create one 16-bit WORD. */

/**
 * @brief Combine four bytes to create one 32-bit DWORD.
 */
#define MAKE_DWORD(b3,b2,b1,b0)                         \
    (((uint32_t)(b3) << 24) | ((uint32_t)(b2) << 16) |  \
     ((uint32_t)(b1) << 8) | ((uint32_t)(b0)))

#define MAKE_DWORD_FROM_WORD(hi,lo)             (((uint32_t)(hi) << 16) | ((uint32_t)(lo)))
/**< Combine two WORDs to create one DWORD. */

#define WORD_GET_MSB(w)         ((uint8_t)((w) >> 8))
/**< Retrieve the MSB from a WORD. */

#define WORD_GET_LSB(w)         ((uint8_t)((w) & 0xFF))
/**< Retrieve the LSB from a WORD. */
    
#define BYTE_GET_UPPER_NIBBLE(w)         ((uint8_t)(((w) >> 4) & 0xF))
/**< Retrieve the Upper nibble from a BYTE. */

#define BYTE_GET_LOWER_NIBBLE(w)         ((uint8_t)((w) & 0xF))
/**< Retrieve the Lower nibble from a BYTE. */
    
#define DWORD_GET_BYTE0(dw)     ((uint8_t)((dw) & 0xFF))
/**< Retrieve the LSB from a DWORD. */

#define DWORD_GET_BYTE1(dw)     ((uint8_t)(((dw) >> 8) & 0xFF))
/**< Retrieve the bits 15-8 from a DWORD. */

#define DWORD_GET_BYTE2(dw)     ((uint8_t)(((dw) >> 16) & 0xFF))
/**< Retrieve the bits 23-16 from a DWORD. */

#define DWORD_GET_BYTE3(dw)     ((uint8_t)(((dw) >> 24) & 0xFF))
/**< Retrieve the MSB from a DWORD. */

#define MEM_COPY(dest, src, size)               memcpy ((uint8_t *)(dest), (uint8_t *)(src), (size))
/**< memcpy abstraction macro. */
    
#define MEM_CMP(buf1, buf2, size)               memcmp ((uint8_t *)(buf1), (uint8_t *)(buf2), (size))
/**< memcmp abstraction macro. */

#define MEM_SET(dest, byte, size)               memset ((uint8_t *)(dest), (byte), (size))
/**< memset abstraction macro. */

#define BUSY_WAIT_US(us)        CyDelayCycles ((uint32_t)(us))
/**< Insert delay of the desired number of us using a busy spin-loop. */

/**
 * @brief Macro to combine 4 bytes in reverse order to make a 32-bit integer.
 * @param n Final 32 bit number
 * @param b Input array of bytes to be combined.
 * @param i Index into the input byte array.
 */
#define REV_BYTE_ORDER(n,b,i)                           \
    (n) = ( (uint32_t) (b)[(i)    ] << 24 )             \
        | ( (uint32_t) (b)[(i) + 1] << 16 )             \
        | ( (uint32_t) (b)[(i) + 2] <<  8 )             \
        | ( (uint32_t) (b)[(i) + 3]       );            \

/*****************************************************************************
* Global Function Declaration
*****************************************************************************/

/**
 * @brief Calculate the 2's complement binary checksum over a BYTE array.
 *
 * This function calculates the checksum of the specified byte array. The
 * checksum is a simple function calculated as the 2's complement of the binary
 * sum of all bytes in the array. This checksum is used for the firmware binary
 * as well as the configuration table.
 *
 * @param ptr Pointer to the data array.
 * @param size Size of the array in BYTE elements.
 *
 * @return Checksum of the data array.
 */
uint8_t mem_calculate_byte_checksum(uint8_t *ptr, uint32_t  size);

/**
 * @brief Calculate the 2's complement binary checksum over a WORD array.
 *
 * This function calculates the checksum of the specified WORD array. The
 * checksum is a simple function calculated as the 2's complement of the binary
 * sum of all words in the array.
 *
 * @param ptr Pointer to the data array.
 * @param size Size of the array in WORD elements.
 *
 * @return Checksum of the data array.
 */
uint16_t mem_calculate_word_checksum(uint16_t *ptr, uint32_t  size);

/**
 * @brief Calculate the 2's complement binary checksum over a DWORD array.
 *
 * This function calculates the checksum of the specified DWORD array. The
 * checksum is a simple function calculated as the 2's complement of the binary
 * sum of all d-words in the array. This checksum is used for the firmware
 * binary as well as the configuration table.
 *
 * @param ptr Pointer to the data array.
 * @param size Size of the array in DWORD elements.
 *
 * @return Checksum of the data array.
 */
uint32_t mem_calculate_dword_checksum(uint32_t *ptr, uint32_t  size);

/**
 * @brief Function to calculate 16-bit CRC.
 *
 * The function implements CRC-16 with polynomial x^16 + x^15 + x^2 + 1 (0xA001).
 *
 * @param crc Original CRC value.
 * @param data Data byte to be included in CRC computation.
 *
 * @return Updated CRC value including the new data byte.
 */
uint16_t crc16(uint16_t crc, uint8_t data);

/**
 * @brief Function to copy 32-bit data from one location to another.
 *
 * @param dest Pointer to Destination.
 * @param source Pointer to source.
 * @param size Size of data in 32-bit DWORD units.
 * @return None.
 */
void mem_copy_word(uint32_t* dest, const uint32_t* source, uint32_t size);

/**
 * @brief Function to do a byte-by-byte copy of data. Replacement for memcpy function.
 * This function is used in cases where we need a memcpy equivalent that can be placed
 * in a specific memory region (such as ROM).
 *
 * @param dest Pointer to destination buffer.
 * @param source Pointer to source buffer.
 * @param size Size of data to be copied (in bytes).
 * @return None
 */
void mem_copy (uint8_t* dest, const uint8_t* source, uint32_t size);

/**
 * @brief Function to initialize a memory buffer. Replacement for memset function.
 * This function is used in cases where we need a memset equivalent that can be placed
 * in a specific memory region (such as ROM).
 *
 * @param dest Pointer to destination buffer.
 * @param c The data to be copied into each byte of the buffer.
 * @param size Size of the buffer (in bytes).
 * @return None
 */
void mem_set (uint8_t* dest, uint8_t c, uint32_t size);

/**
 * @brief Function to divide two unsigned integers and return the quotient rounded up
 * to the nearest integer.
 *
 * @param x The dividend value.
 * @param y The divisor value. Must be non-zero.
 * @return Quotient rounded up to nearest integer.
 */
uint32_t div_round_up(uint32_t x, uint32_t y);

#endif /* _UTILS_H_ */

/* [] END OF FILE */

