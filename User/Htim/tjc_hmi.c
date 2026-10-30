#include "tjc_hmi.h"

#include <limits.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static const uint8_t tjc_terminator[3] = {0xFFu, 0xFFu, 0xFFu};

static bool valid_context(const TJC_HMI *hmi)
{
    return hmi != NULL && hmi->write != NULL;
}

static bool valid_identifier(const char *text, bool allow_dot)
{
    const unsigned char *p = (const unsigned char *)text;
    if (p == NULL || *p == '\0') {
        return false;
    }

    while (*p != '\0') {
        const unsigned char c = *p++;
        const bool alpha = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z');
        const bool digit = c >= '0' && c <= '9';
        if (!alpha && !digit && c != '_' && !(allow_dot && c == '.')) {
            return false;
        }
    }
    return true;
}

static bool valid_text_value(const char *text)
{
    const unsigned char *p = (const unsigned char *)text;
    if (p == NULL) {
        return false;
    }
    while (*p != '\0') {
        if (*p == '"' || *p == 0xFFu || *p < 0x20u) {
            return false;
        }
        ++p;
    }
    return true;
}

static TJC_Status send_format(TJC_HMI *hmi, const char *format, ...)
{
    char command[TJC_HMI_MAX_CMD_LEN];
    va_list args;
    int count;

    if (!valid_context(hmi) || format == NULL) {
        return TJC_ERR_ARGUMENT;
    }

    va_start(args, format);
    count = vsnprintf(command, sizeof(command), format, args);
    va_end(args);

    if (count < 0) {
        return TJC_ERR_PARSE;
    }
    if ((size_t)count >= sizeof(command)) {
        return TJC_ERR_TOO_LONG;
    }
    return TJC_HMI_SendCommand(hmi, command);
}

static void dispatch_event(TJC_HMI *hmi, const TJC_Event *event)
{
    if (hmi->event_callback != NULL) {
        hmi->event_callback(hmi, event, hmi->event_user);
    }
}

static TJC_Status parse_and_dispatch(TJC_HMI *hmi)
{
    TJC_Event event;
    const uint8_t *frame = hmi->rx_frame;
    const size_t len = hmi->rx_length;
    size_t payload_len;

    if (len < 4u || frame[len - 1u] != 0xFFu ||
        frame[len - 2u] != 0xFFu || frame[len - 3u] != 0xFFu) {
        return TJC_ERR_PARSE;
    }

    memset(&event, 0, sizeof(event));
    event.type = TJC_EVENT_UNKNOWN;
    event.code = frame[0];
    payload_len = len - 3u;

    switch (event.code) {
    case 0x01u:
        event.type = TJC_EVENT_ACK;
        break;
    case 0x65u:
        if (payload_len >= 4u) {
            event.type = TJC_EVENT_TOUCH;
            event.page_id = frame[1];
            event.component_id = frame[2];
            event.touch_event = frame[3];
        }
        break;
    case 0x66u:
        if (payload_len >= 2u) {
            event.type = TJC_EVENT_PAGE_ID;
            event.page_id = frame[1];
        }
        break;
    case 0x67u:
    case 0x68u:
        if (payload_len >= 6u) {
            event.type = event.code == 0x67u ? TJC_EVENT_TOUCH_COORDINATE
                                             : TJC_EVENT_SLEEP_TOUCH_COORDINATE;
            event.x = (uint16_t)(((uint16_t)frame[1] << 8) | frame[2]);
            event.y = (uint16_t)(((uint16_t)frame[3] << 8) | frame[4]);
            event.touch_event = frame[5];
        }
        break;
    case 0x70u:
        event.type = TJC_EVENT_STRING;
        event.data = payload_len > 1u ? &frame[1] : NULL;
        event.data_len = payload_len > 1u ? payload_len - 1u : 0u;
        break;
    case 0x71u:
        if (payload_len >= 5u) {
            const uint32_t raw = (uint32_t)frame[1]
                               | ((uint32_t)frame[2] << 8)
                               | ((uint32_t)frame[3] << 16)
                               | ((uint32_t)frame[4] << 24);
            event.type = TJC_EVENT_NUMBER;
            event.number = (int32_t)raw;
        }
        break;
    case 0x86u:
        event.type = TJC_EVENT_SLEEP;
        break;
    case 0x87u:
        event.type = TJC_EVENT_WAKE;
        break;
    case 0x88u:
        event.type = TJC_EVENT_READY;
        break;
    case 0x89u:
        event.type = TJC_EVENT_SD_UPGRADE;
        break;
    case 0xFDu:
        event.type = TJC_EVENT_TRANSPARENT_DONE;
        break;
    case 0xFEu:
        event.type = TJC_EVENT_TRANSPARENT_READY;
        hmi->stream_ready = hmi->stream_expected > 0u;
        break;
    default:
        if (event.code <= 0x24u) {
            event.type = TJC_EVENT_ERROR;
            /* An addt command that is rejected never produces 0xFE.  Release
             * the pending state so the caller can correct the command and
             * retry instead of remaining permanently busy. */
            if (hmi->stream_expected != 0u && !hmi->stream_ready) {
                hmi->stream_expected = 0u;
            }
        } else {
            event.type = TJC_EVENT_CUSTOM;
            event.data = payload_len > 1u ? &frame[1] : NULL;
            event.data_len = payload_len > 1u ? payload_len - 1u : 0u;
        }
        break;
    }

    if (event.type == TJC_EVENT_UNKNOWN) {
        event.type = TJC_EVENT_CUSTOM;
        event.data = payload_len > 1u ? &frame[1] : NULL;
        event.data_len = payload_len > 1u ? payload_len - 1u : 0u;
    }

    dispatch_event(hmi, &event);
    return TJC_FRAME_READY;
}

void TJC_HMI_Init(TJC_HMI *hmi, TJC_WriteFn write_fn, void *write_user)
{
    if (hmi == NULL) {
        return;
    }
    memset(hmi, 0, sizeof(*hmi));
    hmi->write = write_fn;
    hmi->write_user = write_user;
}

void TJC_HMI_SetEventCallback(TJC_HMI *hmi, TJC_EventFn callback, void *event_user)
{
    if (hmi == NULL) {
        return;
    }
    hmi->event_callback = callback;
    hmi->event_user = event_user;
}

void TJC_HMI_ResetRx(TJC_HMI *hmi)
{
    if (hmi != NULL) {
        hmi->rx_length = 0u;
    }
}

TJC_Status TJC_HMI_SendRaw(TJC_HMI *hmi, const uint8_t *data, size_t len)
{
    if (!valid_context(hmi) || (data == NULL && len != 0u)) {
        return TJC_ERR_ARGUMENT;
    }
    if (len == 0u) {
        return TJC_OK;
    }
    return hmi->write(data, len, hmi->write_user) == 0 ? TJC_OK : TJC_ERR_WRITE;
}

TJC_Status TJC_HMI_SendCommand(TJC_HMI *hmi, const char *command)
{
    TJC_Status status;
    size_t len;

    if (!valid_context(hmi) || command == NULL || command[0] == '\0') {
        return TJC_ERR_ARGUMENT;
    }
    len = strlen(command);
    if (len >= TJC_HMI_MAX_CMD_LEN) {
        return TJC_ERR_TOO_LONG;
    }

    status = TJC_HMI_SendRaw(hmi, (const uint8_t *)command, len);
    if (status != TJC_OK) {
        return status;
    }
    return TJC_HMI_SendRaw(hmi, tjc_terminator, sizeof(tjc_terminator));
}

TJC_Status TJC_HMI_FeedByte(TJC_HMI *hmi, uint8_t byte)
{
    TJC_Status status;

    if (hmi == NULL) {
        return TJC_ERR_ARGUMENT;
    }
    if (hmi->rx_length >= sizeof(hmi->rx_frame)) {
        hmi->rx_length = 0u;
        return TJC_ERR_OVERFLOW;
    }

    hmi->rx_frame[hmi->rx_length++] = byte;
    if (hmi->rx_length < 3u ||
        hmi->rx_frame[hmi->rx_length - 1u] != 0xFFu ||
        hmi->rx_frame[hmi->rx_length - 2u] != 0xFFu ||
        hmi->rx_frame[hmi->rx_length - 3u] != 0xFFu) {
        return TJC_OK;
    }

    status = parse_and_dispatch(hmi);
    hmi->rx_length = 0u;
    return status;
}

TJC_Status TJC_HMI_Feed(TJC_HMI *hmi, const uint8_t *data, size_t len)
{
    TJC_Status aggregate = TJC_OK;
    size_t i;

    if (hmi == NULL || (data == NULL && len != 0u)) {
        return TJC_ERR_ARGUMENT;
    }
    for (i = 0u; i < len; ++i) {
        const TJC_Status status = TJC_HMI_FeedByte(hmi, data[i]);
        if (status < 0) {
            aggregate = status;
        } else if (status == TJC_FRAME_READY && aggregate == TJC_OK) {
            aggregate = TJC_FRAME_READY;
        }
    }
    return aggregate;
}

TJC_Status TJC_HMI_SetNumberProperty(TJC_HMI *hmi, const char *object,
                                     const char *property, int32_t value)
{
    if (!valid_identifier(object, true) || !valid_identifier(property, false)) {
        return TJC_ERR_ARGUMENT;
    }
    return send_format(hmi, "%s.%s=%ld", object, property, (long)value);
}

TJC_Status TJC_HMI_SetTextProperty(TJC_HMI *hmi, const char *object,
                                   const char *property, const char *value)
{
    if (!valid_identifier(object, true) || !valid_identifier(property, false) ||
        !valid_text_value(value)) {
        return TJC_ERR_ARGUMENT;
    }
    return send_format(hmi, "%s.%s=\"%s\"", object, property, value);
}

TJC_Status TJC_HMI_GetProperty(TJC_HMI *hmi, const char *expression)
{
    if (!valid_identifier(expression, true)) {
        return TJC_ERR_ARGUMENT;
    }
    return send_format(hmi, "get %s", expression);
}

TJC_Status TJC_HMI_ShowPage(TJC_HMI *hmi, const char *page)
{
    if (!valid_identifier(page, false)) {
        return TJC_ERR_ARGUMENT;
    }
    return send_format(hmi, "page %s", page);
}

TJC_Status TJC_HMI_Refresh(TJC_HMI *hmi, const char *object)
{
    if (!valid_identifier(object, true)) {
        return TJC_ERR_ARGUMENT;
    }
    return send_format(hmi, "ref %s", object);
}

TJC_Status TJC_HMI_SetVisible(TJC_HMI *hmi, const char *object, bool visible)
{
    if (!valid_identifier(object, true)) {
        return TJC_ERR_ARGUMENT;
    }
    return send_format(hmi, "vis %s,%u", object, visible ? 1u : 0u);
}

TJC_Status TJC_HMI_SetTouchEnabled(TJC_HMI *hmi, const char *object, bool enabled)
{
    if (!valid_identifier(object, true)) {
        return TJC_ERR_ARGUMENT;
    }
    return send_format(hmi, "tsw %s,%u", object, enabled ? 1u : 0u);
}

TJC_Status TJC_HMI_Click(TJC_HMI *hmi, const char *object, bool pressed)
{
    if (!valid_identifier(object, true)) {
        return TJC_ERR_ARGUMENT;
    }
    return send_format(hmi, "click %s,%u", object, pressed ? 1u : 0u);
}

TJC_Status TJC_HMI_SetBkCmd(TJC_HMI *hmi, uint8_t mode)
{
    if (mode > 3u) {
        return TJC_ERR_ARGUMENT;
    }
    return send_format(hmi, "bkcmd=%u", (unsigned)mode);
}

TJC_Status TJC_HMI_RequestCurrentPage(TJC_HMI *hmi)
{
    return TJC_HMI_SendCommand(hmi, "sendme");
}

TJC_Status TJC_Text_SetText(TJC_HMI *hmi, const char *object, const char *text)
{
    return TJC_HMI_SetTextProperty(hmi, object, "txt", text);
}

TJC_Status TJC_Text_SetFont(TJC_HMI *hmi, const char *object, uint8_t font_id)
{
    return TJC_HMI_SetNumberProperty(hmi, object, "font", font_id);
}

TJC_Status TJC_Text_SetTextColor(TJC_HMI *hmi, const char *object, uint16_t rgb565)
{
    return TJC_HMI_SetNumberProperty(hmi, object, "pco", rgb565);
}

TJC_Status TJC_Text_SetBackgroundColor(TJC_HMI *hmi, const char *object, uint16_t rgb565)
{
    return TJC_HMI_SetNumberProperty(hmi, object, "bco", rgb565);
}

TJC_Status TJC_Text_SetVisible(TJC_HMI *hmi, const char *object, bool visible)
{
    return TJC_HMI_SetVisible(hmi, object, visible);
}

TJC_Status TJC_Text_RequestText(TJC_HMI *hmi, const char *object)
{
    if (!valid_identifier(object, true)) {
        return TJC_ERR_ARGUMENT;
    }
    return send_format(hmi, "get %s.txt", object);
}

TJC_Status TJC_ScrollText_SetText(TJC_HMI *hmi, const char *object, const char *text)
{
    return TJC_HMI_SetTextProperty(hmi, object, "txt", text);
}

TJC_Status TJC_ScrollText_SetEnabled(TJC_HMI *hmi, const char *object, bool enabled)
{
    return TJC_HMI_SetNumberProperty(hmi, object, "en", enabled ? 1 : 0);
}

TJC_Status TJC_ScrollText_SetDirection(TJC_HMI *hmi, const char *object,
                                       TJC_ScrollDirection direction)
{
    if ((unsigned)direction > (unsigned)TJC_SCROLL_BOTTOM_TO_TOP) {
        return TJC_ERR_ARGUMENT;
    }
    return TJC_HMI_SetNumberProperty(hmi, object, "dir", (int32_t)direction);
}

TJC_Status TJC_ScrollText_SetStep(TJC_HMI *hmi, const char *object, uint8_t pixels)
{
    if (pixels < 2u || pixels > 50u) {
        return TJC_ERR_ARGUMENT;
    }
    return TJC_HMI_SetNumberProperty(hmi, object, "dis", pixels);
}

TJC_Status TJC_ScrollText_SetPeriod(TJC_HMI *hmi, const char *object, uint16_t period_ms)
{
    if (period_ms < 80u || period_ms > 65534u) {
        return TJC_ERR_ARGUMENT;
    }
    return TJC_HMI_SetNumberProperty(hmi, object, "tim", period_ms);
}

TJC_Status TJC_ScrollText_SetFont(TJC_HMI *hmi, const char *object, uint8_t font_id)
{
    return TJC_HMI_SetNumberProperty(hmi, object, "font", font_id);
}

TJC_Status TJC_Number_SetValue(TJC_HMI *hmi, const char *object, int32_t value)
{
    return TJC_HMI_SetNumberProperty(hmi, object, "val", value);
}

TJC_Status TJC_Number_RequestValue(TJC_HMI *hmi, const char *object)
{
    if (!valid_identifier(object, true)) {
        return TJC_ERR_ARGUMENT;
    }
    return send_format(hmi, "get %s.val", object);
}

TJC_Status TJC_Number_SetFont(TJC_HMI *hmi, const char *object, uint8_t font_id)
{
    return TJC_HMI_SetNumberProperty(hmi, object, "font", font_id);
}

TJC_Status TJC_XFloat_SetRaw(TJC_HMI *hmi, const char *object, int32_t scaled_value)
{
    return TJC_HMI_SetNumberProperty(hmi, object, "val", scaled_value);
}

TJC_Status TJC_XFloat_SetValue(TJC_HMI *hmi, const char *object,
                               double value, uint8_t decimal_places)
{
    double scaled = value;
    uint8_t i;

    if (decimal_places > 8u) {
        return TJC_ERR_ARGUMENT;
    }
    for (i = 0u; i < decimal_places; ++i) {
        scaled *= 10.0;
    }
    if (scaled > (double)INT32_MAX || scaled < (double)INT32_MIN) {
        return TJC_ERR_ARGUMENT;
    }
    return TJC_XFloat_SetRaw(hmi, object,
                             (int32_t)(scaled >= 0.0 ? scaled + 0.5 : scaled - 0.5));
}

TJC_Status TJC_XFloat_SetFormat(TJC_HMI *hmi, const char *object,
                                uint8_t integer_digits, uint8_t decimal_places)
{
    TJC_Status status;
    if (integer_digits > 10u || decimal_places > 8u) {
        return TJC_ERR_ARGUMENT;
    }
    status = TJC_HMI_SetNumberProperty(hmi, object, "vvs0", integer_digits);
    if (status != TJC_OK) {
        return status;
    }
    return TJC_HMI_SetNumberProperty(hmi, object, "vvs1", decimal_places);
}

TJC_Status TJC_XFloat_RequestRaw(TJC_HMI *hmi, const char *object)
{
    return TJC_Number_RequestValue(hmi, object);
}

TJC_Status TJC_Button_SetText(TJC_HMI *hmi, const char *object, const char *text)
{
    return TJC_HMI_SetTextProperty(hmi, object, "txt", text);
}

TJC_Status TJC_Button_SetEnabled(TJC_HMI *hmi, const char *object, bool enabled)
{
    return TJC_HMI_SetTouchEnabled(hmi, object, enabled);
}

TJC_Status TJC_Button_Press(TJC_HMI *hmi, const char *object)
{
    return TJC_HMI_Click(hmi, object, true);
}

TJC_Status TJC_Button_Release(TJC_HMI *hmi, const char *object)
{
    return TJC_HMI_Click(hmi, object, false);
}

TJC_Status TJC_Button_SetTextColor(TJC_HMI *hmi, const char *object,
                                   uint16_t normal_rgb565, uint16_t pressed_rgb565)
{
    TJC_Status status = TJC_HMI_SetNumberProperty(hmi, object, "pco", normal_rgb565);
    if (status != TJC_OK) {
        return status;
    }
    return TJC_HMI_SetNumberProperty(hmi, object, "pco2", pressed_rgb565);
}

TJC_Status TJC_Progress_SetValue(TJC_HMI *hmi, const char *object, uint8_t percent)
{
    if (percent > 100u) {
        return TJC_ERR_ARGUMENT;
    }
    return TJC_HMI_SetNumberProperty(hmi, object, "val", percent);
}

TJC_Status TJC_Progress_RequestValue(TJC_HMI *hmi, const char *object)
{
    return TJC_Number_RequestValue(hmi, object);
}

TJC_Status TJC_Progress_SetColors(TJC_HMI *hmi, const char *object,
                                  uint16_t background_rgb565, uint16_t foreground_rgb565)
{
    TJC_Status status = TJC_HMI_SetNumberProperty(hmi, object, "bco", background_rgb565);
    if (status != TJC_OK) {
        return status;
    }
    return TJC_HMI_SetNumberProperty(hmi, object, "pco", foreground_rgb565);
}

TJC_Status TJC_Picture_SetImage(TJC_HMI *hmi, const char *object, uint16_t picture_id)
{
    return TJC_HMI_SetNumberProperty(hmi, object, "pic", picture_id);
}

TJC_Status TJC_Crop_SetImage(TJC_HMI *hmi, const char *object, uint16_t full_picture_id)
{
    return TJC_HMI_SetNumberProperty(hmi, object, "picc", full_picture_id);
}

TJC_Status TJC_Gauge_SetValue(TJC_HMI *hmi, const char *object, int32_t value)
{
    return TJC_HMI_SetNumberProperty(hmi, object, "val", value);
}

TJC_Status TJC_Gauge_SetColor(TJC_HMI *hmi, const char *object, uint16_t rgb565)
{
    return TJC_HMI_SetNumberProperty(hmi, object, "pco", rgb565);
}

TJC_Status TJC_Gauge_RequestValue(TJC_HMI *hmi, const char *object)
{
    return TJC_Number_RequestValue(hmi, object);
}

TJC_Status TJC_Hotspot_SetEnabled(TJC_HMI *hmi, const char *object, bool enabled)
{
    return TJC_HMI_SetTouchEnabled(hmi, object, enabled);
}

TJC_Status TJC_Hotspot_Press(TJC_HMI *hmi, const char *object)
{
    return TJC_HMI_Click(hmi, object, true);
}

TJC_Status TJC_Hotspot_Release(TJC_HMI *hmi, const char *object)
{
    return TJC_HMI_Click(hmi, object, false);
}

TJC_Status TJC_TouchCapture_RequestComponentId(TJC_HMI *hmi, const char *object)
{
    if (!valid_identifier(object, true)) {
        return TJC_ERR_ARGUMENT;
    }
    return send_format(hmi, "get %s.val", object);
}

TJC_Status TJC_TouchCapture_SetCoordinateReporting(TJC_HMI *hmi, bool enabled)
{
    return send_format(hmi, "sendxy=%u", enabled ? 1u : 0u);
}

TJC_Status TJC_Slider_SetValue(TJC_HMI *hmi, const char *object, uint16_t value)
{
    return TJC_HMI_SetNumberProperty(hmi, object, "val", value);
}

TJC_Status TJC_Slider_SetRange(TJC_HMI *hmi, const char *object,
                               uint16_t minimum, uint16_t maximum)
{
    TJC_Status status;
    if (minimum > maximum) {
        return TJC_ERR_ARGUMENT;
    }
    status = TJC_HMI_SetNumberProperty(hmi, object, "minval", minimum);
    if (status != TJC_OK) {
        return status;
    }
    return TJC_HMI_SetNumberProperty(hmi, object, "maxval", maximum);
}

TJC_Status TJC_Slider_RequestValue(TJC_HMI *hmi, const char *object)
{
    return TJC_Number_RequestValue(hmi, object);
}

TJC_Status TJC_Slider_SetEnabled(TJC_HMI *hmi, const char *object, bool enabled)
{
    return TJC_HMI_SetTouchEnabled(hmi, object, enabled);
}

TJC_Status TJC_Curve_AddPoint(TJC_HMI *hmi, const char *object,
                              uint8_t channel, uint8_t value)
{
    if (!valid_identifier(object, false) || channel > 3u) {
        return TJC_ERR_ARGUMENT;
    }
    return send_format(hmi, "add %s.id,%u,%u", object,
                       (unsigned)channel, (unsigned)value);
}

TJC_Status TJC_Curve_ClearChannel(TJC_HMI *hmi, const char *object, uint8_t channel)
{
    if (!valid_identifier(object, false) || channel > 3u) {
        return TJC_ERR_ARGUMENT;
    }
    return send_format(hmi, "cle %s.id,%u", object, (unsigned)channel);
}

TJC_Status TJC_Curve_ClearAll(TJC_HMI *hmi, const char *object)
{
    if (!valid_identifier(object, false)) {
        return TJC_ERR_ARGUMENT;
    }
    return send_format(hmi, "cle %s.id,255", object);
}

TJC_Status TJC_Curve_SetChannelColor(TJC_HMI *hmi, const char *object,
                                     uint8_t channel, uint16_t rgb565)
{
    char property[8];
    if (channel > 3u) {
        return TJC_ERR_ARGUMENT;
    }
    (void)snprintf(property, sizeof(property), "pco%u", (unsigned)channel);
    return TJC_HMI_SetNumberProperty(hmi, object, property, rgb565);
}

TJC_Status TJC_Curve_StreamBegin(TJC_HMI *hmi, const char *object,
                                 uint8_t channel, uint16_t point_count)
{
    TJC_Status status;
    if (!valid_context(hmi) || !valid_identifier(object, false) ||
        channel > 3u || point_count == 0u || point_count > 1024u) {
        return TJC_ERR_ARGUMENT;
    }
    if (hmi->stream_expected != 0u) {
        return TJC_ERR_BUSY;
    }

    hmi->stream_expected = point_count;
    hmi->stream_ready = false;
    status = send_format(hmi, "addt %s.id,%u,%u", object,
                         (unsigned)channel, (unsigned)point_count);
    if (status != TJC_OK) {
        hmi->stream_expected = 0u;
    }
    return status;
}

TJC_Status TJC_Curve_StreamWrite(TJC_HMI *hmi, const uint8_t *points, size_t point_count)
{
    TJC_Status status;
    if (!valid_context(hmi) || points == NULL || point_count == 0u) {
        return TJC_ERR_ARGUMENT;
    }
    if (!hmi->stream_ready || hmi->stream_expected == 0u) {
        return TJC_ERR_NOT_READY;
    }
    if (point_count != hmi->stream_expected) {
        return TJC_ERR_ARGUMENT;
    }

    status = TJC_HMI_SendRaw(hmi, points, point_count);
    if (status == TJC_OK) {
        hmi->stream_expected = 0u;
        hmi->stream_ready = false;
    }
    return status;
}

bool TJC_Curve_StreamIsReady(const TJC_HMI *hmi)
{
    return hmi != NULL && hmi->stream_ready;
}

TJC_Status TJC_Timer_SetEnabled(TJC_HMI *hmi, const char *object, bool enabled)
{
    return TJC_HMI_SetNumberProperty(hmi, object, "en", enabled ? 1 : 0);
}

TJC_Status TJC_Timer_SetPeriod(TJC_HMI *hmi, const char *object, uint16_t period_ms)
{
    if (period_ms < 50u || period_ms > 65534u) {
        return TJC_ERR_ARGUMENT;
    }
    return TJC_HMI_SetNumberProperty(hmi, object, "tim", period_ms);
}

TJC_Status TJC_Variable_SetNumber(TJC_HMI *hmi, const char *object, int32_t value)
{
    return TJC_HMI_SetNumberProperty(hmi, object, "val", value);
}

TJC_Status TJC_Variable_SetText(TJC_HMI *hmi, const char *object, const char *text)
{
    return TJC_HMI_SetTextProperty(hmi, object, "txt", text);
}

TJC_Status TJC_Variable_RequestNumber(TJC_HMI *hmi, const char *object)
{
    return TJC_Number_RequestValue(hmi, object);
}

TJC_Status TJC_Variable_RequestText(TJC_HMI *hmi, const char *object)
{
    return TJC_Text_RequestText(hmi, object);
}

TJC_Status TJC_DualStateButton_SetValue(TJC_HMI *hmi, const char *object, bool on)
{
    return TJC_HMI_SetNumberProperty(hmi, object, "val", on ? 1 : 0);
}

TJC_Status TJC_DualStateButton_SetText(TJC_HMI *hmi, const char *object, const char *text)
{
    return TJC_HMI_SetTextProperty(hmi, object, "txt", text);
}

TJC_Status TJC_DualStateButton_RequestValue(TJC_HMI *hmi, const char *object)
{
    return TJC_Number_RequestValue(hmi, object);
}

TJC_Status TJC_DualStateButton_SetEnabled(TJC_HMI *hmi, const char *object, bool enabled)
{
    return TJC_HMI_SetTouchEnabled(hmi, object, enabled);
}

TJC_Status TJC_Checkbox_SetChecked(TJC_HMI *hmi, const char *object, bool checked)
{
    return TJC_HMI_SetNumberProperty(hmi, object, "val", checked ? 1 : 0);
}

TJC_Status TJC_Checkbox_RequestChecked(TJC_HMI *hmi, const char *object)
{
    return TJC_Number_RequestValue(hmi, object);
}

TJC_Status TJC_Radio_SetChecked(TJC_HMI *hmi, const char *object, bool checked)
{
    return TJC_HMI_SetNumberProperty(hmi, object, "val", checked ? 1 : 0);
}

TJC_Status TJC_Radio_RequestChecked(TJC_HMI *hmi, const char *object)
{
    return TJC_Number_RequestValue(hmi, object);
}

TJC_Status TJC_Switch_SetValue(TJC_HMI *hmi, const char *object, bool on)
{
    return TJC_HMI_SetNumberProperty(hmi, object, "val", on ? 1 : 0);
}

TJC_Status TJC_Switch_RequestValue(TJC_HMI *hmi, const char *object)
{
    return TJC_Number_RequestValue(hmi, object);
}

TJC_Status TJC_QRCode_SetText(TJC_HMI *hmi, const char *object, const char *text)
{
    return TJC_HMI_SetTextProperty(hmi, object, "txt", text);
}

TJC_Status TJC_ComboBox_SetPath(TJC_HMI *hmi, const char *object, const char *path)
{
    return TJC_HMI_SetTextProperty(hmi, object, "path", path);
}

TJC_Status TJC_ComboBox_SetSelected(TJC_HMI *hmi, const char *object, uint16_t index)
{
    return TJC_HMI_SetNumberProperty(hmi, object, "val", index);
}

TJC_Status TJC_ComboBox_RequestSelected(TJC_HMI *hmi, const char *object)
{
    return TJC_Number_RequestValue(hmi, object);
}

TJC_Status TJC_ComboBox_RequestText(TJC_HMI *hmi, const char *object)
{
    return TJC_Text_RequestText(hmi, object);
}

TJC_Status TJC_ComboBox_SetExpanded(TJC_HMI *hmi, const char *object, bool expanded)
{
    return TJC_HMI_SetNumberProperty(hmi, object, "down", expanded ? 1 : 0);
}

TJC_Status TJC_TextSelect_SetPath(TJC_HMI *hmi, const char *object, const char *path)
{
    return TJC_HMI_SetTextProperty(hmi, object, "path", path);
}

TJC_Status TJC_TextSelect_SetSelected(TJC_HMI *hmi, const char *object, uint16_t index)
{
    return TJC_HMI_SetNumberProperty(hmi, object, "val", index);
}

TJC_Status TJC_TextSelect_RequestSelected(TJC_HMI *hmi, const char *object)
{
    return TJC_Number_RequestValue(hmi, object);
}

TJC_Status TJC_TextSelect_RequestText(TJC_HMI *hmi, const char *object)
{
    return TJC_Text_RequestText(hmi, object);
}
