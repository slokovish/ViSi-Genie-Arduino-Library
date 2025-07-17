#include "genieSTM32.h"
#include <string.h>
#include <stdio.h>

static void send_byte(GenieSTM32 *g, uint8_t b)
{
    HAL_UART_Transmit(g->huart, &b, 1, HAL_MAX_DELAY);
}

static uint8_t recv_byte(GenieSTM32 *g, uint32_t timeout, int *status)
{
    uint8_t b = 0;
    if (HAL_UART_Receive(g->huart, &b, 1, timeout) != HAL_OK) {
        if (status) *status = -1;
    } else if (status) {
        *status = 0;
    }
    return b;
}

static uint32_t millis(void)
{
    return HAL_GetTick();
}

static void push_state(GenieSTM32 *g, uint8_t state)
{
    if ((g->LinkState - g->LinkStates) < (MAX_LINK_STATES - 1)) {
        g->LinkState++;
        *g->LinkState = state;
    }
}

static void pop_state(GenieSTM32 *g)
{
    if (g->LinkState > g->LinkStates) {
        *g->LinkState = 0;
        g->LinkState--;
    }
}

static uint16_t get_link_state(GenieSTM32 *g)
{
    return *g->LinkState;
}

static void set_link_state(GenieSTM32 *g, uint16_t state)
{
    *g->LinkState = state;
    if (state == GENIE_LINK_RXREPORT || state == GENIE_LINK_RXEVENT)
        g->rxframe_count = 0;
}

static uint8_t get_char(GenieSTM32 *g)
{
    uint8_t c;
    int st;
    c = recv_byte(g, g->Timeout, &st);
    if (st != 0) {
        g->Error = ERROR_NOCHAR;
        return 0;
    }
    g->Error = ERROR_NONE;
    return c;
}

static int enqueue_event(GenieSTM32 *g, uint8_t *data)
{
    if (g->EventQueue.n_events >= MAX_GENIE_EVENTS - 2) {
        g->Error = ERROR_REPLY_OVR;
        return -1;
    }
    memcpy(&g->EventQueue.frames[g->EventQueue.wr_index], data, GENIE_FRAME_SIZE);
    g->EventQueue.wr_index++;
    g->EventQueue.wr_index &= (MAX_GENIE_EVENTS - 1);
    g->EventQueue.n_events++;
    return 0;
}

static void flush_event_queue(GenieSTM32 *g)
{
    memset(&g->EventQueue, 0, sizeof(g->EventQueue));
}

static void wait_for_idle(GenieSTM32 *g)
{
    uint32_t timeout = millis() + g->Timeout;
    for (; millis() < timeout;) {
        genieSTM32_DoEvents(g, 0);
        if (get_link_state(g) == GENIE_LINK_IDLE)
            return;
    }
    g->Error = ERROR_TIMEOUT;
}

void genieSTM32_Begin(GenieSTM32 *g, UART_HandleTypeDef *huart)
{
    memset(g, 0, sizeof(*g));
    g->huart = huart;
    g->Timeout = TIMEOUT_PERIOD;
    g->LinkState = &g->LinkStates[0];
    *g->LinkState = GENIE_LINK_IDLE;
    flush_event_queue(g);
}

int genieSTM32_WriteObject(GenieSTM32 *g, uint16_t object, uint16_t index, uint16_t data)
{
    uint8_t buf[GENIE_FRAME_SIZE];
    uint8_t cs = 0;
    wait_for_idle(g);
    buf[0] = GENIE_WRITE_OBJ; cs ^= buf[0];
    buf[1] = object; cs ^= buf[1];
    buf[2] = index; cs ^= buf[2];
    buf[3] = data >> 8; cs ^= buf[3];
    buf[4] = data & 0xFF; cs ^= buf[4];
    buf[5] = cs;
    HAL_UART_Transmit(g->huart, buf, GENIE_FRAME_SIZE, HAL_MAX_DELAY);
    push_state(g, GENIE_LINK_WFAN);
    return 0;
}

int genieSTM32_ReadObject(GenieSTM32 *g, uint16_t object, uint16_t index)
{
    uint8_t buf[4];
    uint8_t cs = GENIE_READ_OBJ ^ object ^ index;
    wait_for_idle(g);
    buf[0] = GENIE_READ_OBJ;
    buf[1] = object;
    buf[2] = index;
    buf[3] = cs;
    HAL_UART_Transmit(g->huart, buf, 4, HAL_MAX_DELAY);
    push_state(g, GENIE_LINK_WF_RXREPORT);
    return 0;
}

static uint8_t get_next_byte(GenieSTM32 *g)
{
    while (1) {
        uint8_t c = 0;
        if (HAL_UART_Receive(g->huart, &c, 1, HAL_MAX_DELAY) == HAL_OK)
            return c;
    }
}

static uint16_t get_next_word(GenieSTM32 *g)
{
    uint16_t out = ((uint16_t)get_next_byte(g)) << 8;
    out |= get_next_byte(g);
    return out;
}

int genieSTM32_DoEvents(GenieSTM32 *g, int do_handler)
{
    uint8_t c = get_char(g);
    static uint8_t rx_data[GENIE_FRAME_SIZE];
    static uint8_t checksum = 0;
    static MagicReportHeader magicHeader;
    static uint8_t magicByte = 0;

    if (g->Error == ERROR_NOCHAR) {
        if (g->EventQueue.n_events > 0 && g->UserHandler && do_handler)
            g->UserHandler();
        return GENIE_EVENT_NONE;
    }

    switch (get_link_state(g)) {
    case GENIE_LINK_IDLE:
        switch (c) {
        case GENIE_REPORT_EVENT:
            push_state(g, GENIE_LINK_RXEVENT);
            break;
        case GENIEM_REPORT_BYTES:
            magicByte = 0;
            push_state(g, GENIE_LINK_RXMBYTES);
            break;
        case GENIEM_REPORT_DBYTES:
            magicByte = 0;
            push_state(g, GENIE_LINK_RXMDBYTES);
            break;
        default:
            return GENIE_EVENT_RXCHAR;
        }
        break;

    case GENIE_LINK_WFAN:
        switch (c) {
        case GENIE_ACK:
            pop_state(g);
            return GENIE_EVENT_RXCHAR;
        case GENIE_NAK:
            pop_state(g);
            g->Error = ERROR_NAK;
            return GENIE_EVENT_RXCHAR;
        case GENIE_REPORT_EVENT:
            push_state(g, GENIE_LINK_RXEVENT);
            break;
        case GENIEM_REPORT_BYTES:
            magicByte = 0;
            push_state(g, GENIE_LINK_RXMBYTES);
            break;
        case GENIEM_REPORT_DBYTES:
            magicByte = 0;
            push_state(g, GENIE_LINK_RXMDBYTES);
            break;
        case GENIE_REPORT_OBJ:
        default:
            return GENIE_EVENT_RXCHAR;
        }
        break;

    case GENIE_LINK_WF_RXREPORT:
        switch (c) {
        case GENIE_REPORT_EVENT:
            push_state(g, GENIE_LINK_RXEVENT);
            break;
        case GENIEM_REPORT_BYTES:
            magicByte = 0;
            push_state(g, GENIE_LINK_RXMBYTES);
            break;
        case GENIEM_REPORT_DBYTES:
            magicByte = 0;
            push_state(g, GENIE_LINK_RXMDBYTES);
            break;
        case GENIE_REPORT_OBJ:
            pop_state(g);
            push_state(g, GENIE_LINK_RXREPORT);
            break;
        default:
            return GENIE_EVENT_RXCHAR;
        }
        break;

    default:
        break;
    }

    if (get_link_state(g) == GENIE_LINK_RXREPORT ||
        get_link_state(g) == GENIE_LINK_RXEVENT) {
        checksum = (g->rxframe_count == 0) ? c : checksum ^ c;
        rx_data[g->rxframe_count] = c;

        if (g->rxframe_count == GENIE_FRAME_SIZE - 1) {
            if (checksum == 0) {
                enqueue_event(g, rx_data);
                g->rxframe_count = 0;
                pop_state(g);
                return GENIE_EVENT_RXCHAR;
            } else {
                g->Error = ERROR_BAD_CS;
            }
        }
        g->rxframe_count++;
        return GENIE_EVENT_RXCHAR;
    }

    if (get_link_state(g) == GENIE_LINK_RXMBYTES ||
        get_link_state(g) == GENIE_LINK_RXMDBYTES) {
        switch (magicByte) {
        case 0:
            magicHeader.cmd = c; magicByte++; break;
        case 1:
            magicHeader.index = c; magicByte++; break;
        case 2:
            magicHeader.length = c; magicByte++;
            if (magicHeader.cmd == GENIEM_REPORT_BYTES) {
                if (g->UserByteReader)
                    g->UserByteReader(magicHeader.index, magicHeader.length);
                else
                    while (--magicHeader.length) (void)get_next_byte(g);
            } else {
                if (g->UserDoubleByteReader)
                    g->UserDoubleByteReader(magicHeader.index, magicHeader.length);
                else
                    while (--magicHeader.length) (void)get_next_word(g);
            }
            (void)get_next_byte(g); /* checksum */
            pop_state(g);
            break;
        }
        return GENIE_EVENT_RXCHAR;
    }

    return GENIE_EVENT_RXCHAR;
}

int genieSTM32_DequeueEvent(GenieSTM32 *g, genieUnionFrame *buff)
{
    if (g->EventQueue.n_events > 0) {
        memcpy(buff, &g->EventQueue.frames[g->EventQueue.rd_index], GENIE_FRAME_SIZE);
        g->EventQueue.rd_index++;
        g->EventQueue.rd_index &= (MAX_GENIE_EVENTS - 1);
        g->EventQueue.n_events--;
        return 1;
    }
    return 0;
}

uint16_t genieSTM32_GetEventData(genieUnionFrame *e)
{
    return (e->reportObject.data_msb << 8) | e->reportObject.data_lsb;
}

int genieSTM32_WriteIntLedDigits(GenieSTM32 *g, uint16_t index, int16_t data)
{
    wait_for_idle(g);
    return genieSTM32_WriteObject(g, GENIE_OBJ_ILED_DIGITS_L, index, (uint16_t)data);
}

int genieSTM32_WriteIntLedDigits32(GenieSTM32 *g, uint16_t index, int32_t data)
{
    wait_for_idle(g);
    genieSTM32_WriteObject(g, GENIE_OBJ_ILED_DIGITS_H, index, (uint16_t)(data >> 16));
    return genieSTM32_WriteObject(g, GENIE_OBJ_ILED_DIGITS_L, index, (uint16_t)(data & 0xFFFF));
}

int genieSTM32_WriteIntLedDigitsFloat(GenieSTM32 *g, uint16_t index, float data)
{
    wait_for_idle(g);
    union { float f; uint32_t u; } u;
    u.f = data;
    genieSTM32_WriteObject(g, GENIE_OBJ_ILED_DIGITS_H, index, (uint16_t)(u.u >> 16));
    return genieSTM32_WriteObject(g, GENIE_OBJ_ILED_DIGITS_L, index, (uint16_t)(u.u & 0xFFFF));
}

void genieSTM32_WriteContrast(GenieSTM32 *g, uint16_t value)
{
    wait_for_idle(g);
    uint8_t cs = GENIE_WRITE_CONTRAST ^ (uint8_t)value;
    uint8_t buf[3] = { GENIE_WRITE_CONTRAST, (uint8_t)value, cs };
    HAL_UART_Transmit(g->huart, buf, 3, HAL_MAX_DELAY);
    push_state(g, GENIE_LINK_WFAN);
}

int genieSTM32_WriteStr(GenieSTM32 *g, uint16_t index, const char *string)
{
    wait_for_idle(g);
    size_t len = strlen(string);
    if (len > 255) return -1;
    uint8_t cs = GENIE_WRITE_STR ^ index ^ (uint8_t)len;
    HAL_UART_Transmit(g->huart, (uint8_t[]){GENIE_WRITE_STR, index, (uint8_t)len}, 3, HAL_MAX_DELAY);
    for (size_t i = 0; i < len; i++) {
        send_byte(g, (uint8_t)string[i]);
        cs ^= (uint8_t)string[i];
    }
    send_byte(g, cs);
    push_state(g, GENIE_LINK_WFAN);
    return 0;
}

int genieSTM32_WriteStrU(GenieSTM32 *g, uint16_t index, const uint16_t *string)
{
    wait_for_idle(g);
    size_t len = 0; while (string[len]) len++; if (len > 255) return -1;
    uint8_t cs = GENIE_WRITE_STRU ^ index ^ (uint8_t)len;
    HAL_UART_Transmit(g->huart, (uint8_t[]){GENIE_WRITE_STRU, index, (uint8_t)len}, 3, HAL_MAX_DELAY);
    for (size_t i = 0; i < len; i++) {
        uint8_t msb = string[i] >> 8; uint8_t lsb = string[i] & 0xFF;
        send_byte(g, msb); cs ^= msb;
        send_byte(g, lsb); cs ^= lsb;
    }
    send_byte(g, cs);
    push_state(g, GENIE_LINK_WFAN);
    return 0;
}

int genieSTM32_WriteInhLabel(GenieSTM32 *g, uint16_t index, const char *string)
{
    wait_for_idle(g);
    size_t len = strlen(string);
    if (len > 255) return -1;
    uint8_t cs = GENIE_WRITE_INH_LABEL ^ index ^ (uint8_t)len;
    HAL_UART_Transmit(g->huart, (uint8_t[]){GENIE_WRITE_INH_LABEL, index, (uint8_t)len}, 3, HAL_MAX_DELAY);
    for (size_t i = 0; i < len; i++) {
        send_byte(g, (uint8_t)string[i]);
        cs ^= (uint8_t)string[i];
    }
    send_byte(g, cs);
    push_state(g, GENIE_LINK_WFAN);
    return 0;
}

void genieSTM32_AttachEventHandler(GenieSTM32 *g, UserEventHandlerPtr handler)
{
    g->UserHandler = handler;
}

void genieSTM32_AttachMagicByteReader(GenieSTM32 *g, UserBytePtr handler)
{
    g->UserByteReader = handler;
}

void genieSTM32_AttachMagicDoubleByteReader(GenieSTM32 *g, UserDoubleBytePtr handler)
{
    g->UserDoubleByteReader = handler;
}

