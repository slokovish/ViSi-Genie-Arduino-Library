#ifndef GENIE_STM32_H
#define GENIE_STM32_H

#include <stdint.h>
#include <stddef.h>

#ifdef STM32_HAL
# include "stm32f1xx_hal.h"
#else
typedef struct { int dummy; } UART_HandleTypeDef;
uint32_t HAL_GetTick(void);
int HAL_UART_Transmit(UART_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
int HAL_UART_Receive(UART_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
# define HAL_OK 0
#endif

#ifndef HAL_MAX_DELAY
# define HAL_MAX_DELAY 0xFFFFFFFFU
#endif

/* Genie commands */
#define GENIE_ACK               0x06
#define GENIE_NAK               0x15

#define TIMEOUT_PERIOD          1000
#define RESYNC_PERIOD           100

#define GENIE_READ_OBJ          0
#define GENIE_WRITE_OBJ         1
#define GENIE_WRITE_STR         2
#define GENIE_WRITE_STRU        3
#define GENIE_WRITE_CONTRAST    4
#define GENIE_REPORT_OBJ        5
#define GENIE_REPORT_EVENT      7
#define GENIEM_WRITE_BYTES      8
#define GENIEM_WRITE_DBYTES     9
#define GENIEM_REPORT_BYTES     10
#define GENIEM_REPORT_DBYTES    11
#define GENIE_WRITE_INH_LABEL   12

/* Object constants (subset for brevity) */
#define GENIE_OBJ_FORM          10
#define GENIE_OBJ_GAUGE         11
#define GENIE_OBJ_IMAGE         12
#define GENIE_OBJ_KEYBOARD      13
#define GENIE_OBJ_LED           14
#define GENIE_OBJ_LED_DIGITS    15
#define GENIE_OBJ_STRINGS       17
#define GENIE_OBJ_USER_LED      19
#define GENIE_OBJ_TIMER         23
#define GENIE_OBJ_STATIC_TEXT   21
#define GENIE_OBJ_SMARTGAUGE    35
#define GENIE_OBJ_SMARTSLIDER   36
#define GENIE_OBJ_SMARTKNOB     37
#define GENIE_OBJ_ILED_DIGITS_H 38
#define GENIE_OBJ_IANGULAR_METER 39
#define GENIE_OBJ_IGAUGE        40
#define GENIE_OBJ_ILABELB       41
#define GENIE_OBJ_ILED          45
#define GENIE_OBJ_ILED_DIGITS_L 47

#define GENIE_FRAME_SIZE        6

typedef struct {
    uint8_t  cmd;
    uint8_t  object;
    uint8_t  index;
    uint8_t  data_msb;
    uint8_t  data_lsb;
} genieFrame;

typedef struct {
    uint8_t cmd;
    uint8_t index;
    uint8_t length;
} MagicReportHeader;

typedef union {
    uint8_t     bytes[GENIE_FRAME_SIZE];
    genieFrame  reportObject;
} genieUnionFrame;

#define MAX_GENIE_EVENTS 16
#define MAX_LINK_STATES  20

typedef struct {
    genieUnionFrame frames[MAX_GENIE_EVENTS];
    uint8_t         rd_index;
    uint8_t         wr_index;
    uint8_t         n_events;
} EventQueueStruct;

typedef void (*UserEventHandlerPtr)(void);
typedef void (*UserBytePtr)(uint8_t, uint8_t);
typedef void (*UserDoubleBytePtr)(uint8_t, uint8_t);

typedef struct {
    UART_HandleTypeDef   *huart;
    EventQueueStruct      EventQueue;
    uint8_t               LinkStates[MAX_LINK_STATES];
    uint8_t              *LinkState;
    uint32_t              Timeout;
    int                   Timeouts;
    int                   Error;
    uint8_t               rxframe_count;
    int                   FatalErrors;
    UserEventHandlerPtr   UserHandler;
    UserBytePtr           UserByteReader;
    UserDoubleBytePtr     UserDoubleByteReader;
} GenieSTM32;

/* Error values */
#define ERROR_NONE           0
#define ERROR_TIMEOUT       -1
#define ERROR_NOHANDLER     -2
#define ERROR_NOCHAR        -3
#define ERROR_NAK           -4
#define ERROR_REPLY_OVR     -5
#define ERROR_RESYNC        -6
#define ERROR_NODISPLAY     -7
#define ERROR_BAD_CS        -8

/* Link states */
#define GENIE_LINK_IDLE           0
#define GENIE_LINK_WFAN           1
#define GENIE_LINK_WF_RXREPORT    2
#define GENIE_LINK_RXREPORT       3
#define GENIE_LINK_RXEVENT        4
#define GENIE_LINK_SHDN           5
#define GENIE_LINK_RXMBYTES       6
#define GENIE_LINK_RXMDBYTES      7

#define GENIE_EVENT_NONE    0
#define GENIE_EVENT_RXCHAR  1

/* Public API */
void genieSTM32_Begin(GenieSTM32 *g, UART_HandleTypeDef *huart);
int  genieSTM32_DoEvents(GenieSTM32 *g, int do_handler);
int  genieSTM32_DequeueEvent(GenieSTM32 *g, genieUnionFrame *buff);
int  genieSTM32_ReadObject(GenieSTM32 *g, uint16_t object, uint16_t index);
int  genieSTM32_WriteObject(GenieSTM32 *g, uint16_t object, uint16_t index, uint16_t data);
int  genieSTM32_WriteIntLedDigits(GenieSTM32 *g, uint16_t index, int16_t data);
int  genieSTM32_WriteIntLedDigits32(GenieSTM32 *g, uint16_t index, int32_t data);
int  genieSTM32_WriteIntLedDigitsFloat(GenieSTM32 *g, uint16_t index, float data);
void genieSTM32_WriteContrast(GenieSTM32 *g, uint16_t value);
int  genieSTM32_WriteStr(GenieSTM32 *g, uint16_t index, const char *string);
int  genieSTM32_WriteStrU(GenieSTM32 *g, uint16_t index, const uint16_t *string);
int  genieSTM32_WriteInhLabel(GenieSTM32 *g, uint16_t index, const char *string);
void genieSTM32_AttachEventHandler(GenieSTM32 *g, UserEventHandlerPtr handler);
void genieSTM32_AttachMagicByteReader(GenieSTM32 *g, UserBytePtr handler);
void genieSTM32_AttachMagicDoubleByteReader(GenieSTM32 *g, UserDoubleBytePtr handler);

#endif /* GENIE_STM32_H */

