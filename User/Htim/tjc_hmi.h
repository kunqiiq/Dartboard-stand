#ifndef TJC_HMI_H
#define TJC_HMI_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef TJC_HMI_MAX_CMD_LEN
#define TJC_HMI_MAX_CMD_LEN 192u
#endif

#ifndef TJC_HMI_RX_FRAME_SIZE
#define TJC_HMI_RX_FRAME_SIZE 256u
#endif

typedef struct TJC_HMI TJC_HMI;

typedef enum {
    TJC_OK = 0,
    TJC_FRAME_READY = 1,
    TJC_ERR_ARGUMENT = -1,
    TJC_ERR_WRITE = -2,
    TJC_ERR_TOO_LONG = -3,
    TJC_ERR_PARSE = -4,
    TJC_ERR_OVERFLOW = -5,
    TJC_ERR_BUSY = -6,
    TJC_ERR_NOT_READY = -7
} TJC_Status;

typedef enum {
    TJC_EVENT_UNKNOWN = 0,
    TJC_EVENT_ACK,
    TJC_EVENT_ERROR,
    TJC_EVENT_TOUCH,
    TJC_EVENT_PAGE_ID,
    TJC_EVENT_TOUCH_COORDINATE,
    TJC_EVENT_SLEEP_TOUCH_COORDINATE,
    TJC_EVENT_STRING,
    TJC_EVENT_NUMBER,
    TJC_EVENT_SLEEP,
    TJC_EVENT_WAKE,
    TJC_EVENT_READY,
    TJC_EVENT_SD_UPGRADE,
    TJC_EVENT_TRANSPARENT_DONE,
    TJC_EVENT_TRANSPARENT_READY,
    TJC_EVENT_CUSTOM
} TJC_EventType;

typedef struct {
    TJC_EventType type;
    uint8_t code;
    uint8_t page_id;
    uint8_t component_id;
    uint8_t touch_event; /* 0 = release, 1 = press */
    uint16_t x;
    uint16_t y;
    int32_t number;
    const uint8_t *data;
    size_t data_len;
} TJC_Event;

typedef int (*TJC_WriteFn)(const uint8_t *data, size_t len, void *user);
typedef void (*TJC_EventFn)(TJC_HMI *hmi, const TJC_Event *event, void *user);

struct TJC_HMI {
    TJC_WriteFn write;
    void *write_user;
    TJC_EventFn event_callback;
    void *event_user;
    uint8_t rx_frame[TJC_HMI_RX_FRAME_SIZE];
    size_t rx_length;
    uint16_t stream_expected;
    bool stream_ready;
};

typedef enum {
    TJC_SCROLL_LEFT_TO_RIGHT = 0,
    TJC_SCROLL_RIGHT_TO_LEFT = 1,
    TJC_SCROLL_TOP_TO_BOTTOM = 2,
    TJC_SCROLL_BOTTOM_TO_TOP = 3
} TJC_ScrollDirection;

/* Core transport and protocol ------------------------------------------------ */
void TJC_HMI_Init(TJC_HMI *hmi, TJC_WriteFn write_fn, void *write_user);
void TJC_HMI_SetEventCallback(TJC_HMI *hmi, TJC_EventFn callback, void *event_user);
void TJC_HMI_ResetRx(TJC_HMI *hmi);

TJC_Status TJC_HMI_SendRaw(TJC_HMI *hmi, const uint8_t *data, size_t len);
TJC_Status TJC_HMI_SendCommand(TJC_HMI *hmi, const char *command);
TJC_Status TJC_HMI_FeedByte(TJC_HMI *hmi, uint8_t byte);
TJC_Status TJC_HMI_Feed(TJC_HMI *hmi, const uint8_t *data, size_t len);

TJC_Status TJC_HMI_SetNumberProperty(TJC_HMI *hmi, const char *object,
                                     const char *property, int32_t value);
TJC_Status TJC_HMI_SetTextProperty(TJC_HMI *hmi, const char *object,
                                   const char *property, const char *value);
TJC_Status TJC_HMI_GetProperty(TJC_HMI *hmi, const char *expression);
TJC_Status TJC_HMI_ShowPage(TJC_HMI *hmi, const char *page);
TJC_Status TJC_HMI_Refresh(TJC_HMI *hmi, const char *object);
TJC_Status TJC_HMI_SetVisible(TJC_HMI *hmi, const char *object, bool visible);
TJC_Status TJC_HMI_SetTouchEnabled(TJC_HMI *hmi, const char *object, bool enabled);
TJC_Status TJC_HMI_Click(TJC_HMI *hmi, const char *object, bool pressed);
TJC_Status TJC_HMI_SetBkCmd(TJC_HMI *hmi, uint8_t mode);
TJC_Status TJC_HMI_RequestCurrentPage(TJC_HMI *hmi);

/* Text control -------------------------------------------------------------- */
TJC_Status TJC_Text_SetText(TJC_HMI *hmi, const char *object, const char *text);
TJC_Status TJC_Text_SetFont(TJC_HMI *hmi, const char *object, uint8_t font_id);
TJC_Status TJC_Text_SetTextColor(TJC_HMI *hmi, const char *object, uint16_t rgb565);
TJC_Status TJC_Text_SetBackgroundColor(TJC_HMI *hmi, const char *object, uint16_t rgb565);
TJC_Status TJC_Text_SetVisible(TJC_HMI *hmi, const char *object, bool visible);
TJC_Status TJC_Text_RequestText(TJC_HMI *hmi, const char *object);

/* Scrolling-text control ---------------------------------------------------- */
TJC_Status TJC_ScrollText_SetText(TJC_HMI *hmi, const char *object, const char *text);
TJC_Status TJC_ScrollText_SetEnabled(TJC_HMI *hmi, const char *object, bool enabled);
TJC_Status TJC_ScrollText_SetDirection(TJC_HMI *hmi, const char *object,
                                       TJC_ScrollDirection direction);
TJC_Status TJC_ScrollText_SetStep(TJC_HMI *hmi, const char *object, uint8_t pixels);
TJC_Status TJC_ScrollText_SetPeriod(TJC_HMI *hmi, const char *object, uint16_t period_ms);
TJC_Status TJC_ScrollText_SetFont(TJC_HMI *hmi, const char *object, uint8_t font_id);

/* Number control ------------------------------------------------------------ */
TJC_Status TJC_Number_SetValue(TJC_HMI *hmi, const char *object, int32_t value);
TJC_Status TJC_Number_RequestValue(TJC_HMI *hmi, const char *object);
TJC_Status TJC_Number_SetFont(TJC_HMI *hmi, const char *object, uint8_t font_id);

/* Virtual floating-point control ------------------------------------------- */
TJC_Status TJC_XFloat_SetRaw(TJC_HMI *hmi, const char *object, int32_t scaled_value);
TJC_Status TJC_XFloat_SetValue(TJC_HMI *hmi, const char *object,
                               double value, uint8_t decimal_places);
TJC_Status TJC_XFloat_SetFormat(TJC_HMI *hmi, const char *object,
                                uint8_t integer_digits, uint8_t decimal_places);
TJC_Status TJC_XFloat_RequestRaw(TJC_HMI *hmi, const char *object);

/* Button control ------------------------------------------------------------ */
TJC_Status TJC_Button_SetText(TJC_HMI *hmi, const char *object, const char *text);
TJC_Status TJC_Button_SetEnabled(TJC_HMI *hmi, const char *object, bool enabled);
TJC_Status TJC_Button_Press(TJC_HMI *hmi, const char *object);
TJC_Status TJC_Button_Release(TJC_HMI *hmi, const char *object);
TJC_Status TJC_Button_SetTextColor(TJC_HMI *hmi, const char *object,
                                   uint16_t normal_rgb565, uint16_t pressed_rgb565);

/* Progress-bar control ------------------------------------------------------ */
TJC_Status TJC_Progress_SetValue(TJC_HMI *hmi, const char *object, uint8_t percent);
TJC_Status TJC_Progress_RequestValue(TJC_HMI *hmi, const char *object);
TJC_Status TJC_Progress_SetColors(TJC_HMI *hmi, const char *object,
                                  uint16_t background_rgb565, uint16_t foreground_rgb565);

/* Picture, crop and gauge controls ----------------------------------------- */
TJC_Status TJC_Picture_SetImage(TJC_HMI *hmi, const char *object, uint16_t picture_id);
TJC_Status TJC_Crop_SetImage(TJC_HMI *hmi, const char *object, uint16_t full_picture_id);
TJC_Status TJC_Gauge_SetValue(TJC_HMI *hmi, const char *object, int32_t value);
TJC_Status TJC_Gauge_SetColor(TJC_HMI *hmi, const char *object, uint16_t rgb565);
TJC_Status TJC_Gauge_RequestValue(TJC_HMI *hmi, const char *object);

/* Hotspot control ----------------------------------------------------------- */
TJC_Status TJC_Hotspot_SetEnabled(TJC_HMI *hmi, const char *object, bool enabled);
TJC_Status TJC_Hotspot_Press(TJC_HMI *hmi, const char *object);
TJC_Status TJC_Hotspot_Release(TJC_HMI *hmi, const char *object);

/* Touch-capture and coordinate reporting ----------------------------------- */
TJC_Status TJC_TouchCapture_RequestComponentId(TJC_HMI *hmi, const char *object);
TJC_Status TJC_TouchCapture_SetCoordinateReporting(TJC_HMI *hmi, bool enabled);

/* Slider control ------------------------------------------------------------ */
TJC_Status TJC_Slider_SetValue(TJC_HMI *hmi, const char *object, uint16_t value);
TJC_Status TJC_Slider_SetRange(TJC_HMI *hmi, const char *object,
                               uint16_t minimum, uint16_t maximum);
TJC_Status TJC_Slider_RequestValue(TJC_HMI *hmi, const char *object);
TJC_Status TJC_Slider_SetEnabled(TJC_HMI *hmi, const char *object, bool enabled);

/* Waveform control. add/addt/cle only work for a waveform on the current page. */
TJC_Status TJC_Curve_AddPoint(TJC_HMI *hmi, const char *object,
                              uint8_t channel, uint8_t value);
TJC_Status TJC_Curve_ClearChannel(TJC_HMI *hmi, const char *object, uint8_t channel);
TJC_Status TJC_Curve_ClearAll(TJC_HMI *hmi, const char *object);
TJC_Status TJC_Curve_SetChannelColor(TJC_HMI *hmi, const char *object,
                                     uint8_t channel, uint16_t rgb565);
TJC_Status TJC_Curve_StreamBegin(TJC_HMI *hmi, const char *object,
                                 uint8_t channel, uint16_t point_count);
TJC_Status TJC_Curve_StreamWrite(TJC_HMI *hmi, const uint8_t *points, size_t point_count);
bool TJC_Curve_StreamIsReady(const TJC_HMI *hmi);

/* Timer control ------------------------------------------------------------- */
TJC_Status TJC_Timer_SetEnabled(TJC_HMI *hmi, const char *object, bool enabled);
TJC_Status TJC_Timer_SetPeriod(TJC_HMI *hmi, const char *object, uint16_t period_ms);

/* Variable control ---------------------------------------------------------- */
TJC_Status TJC_Variable_SetNumber(TJC_HMI *hmi, const char *object, int32_t value);
TJC_Status TJC_Variable_SetText(TJC_HMI *hmi, const char *object, const char *text);
TJC_Status TJC_Variable_RequestNumber(TJC_HMI *hmi, const char *object);
TJC_Status TJC_Variable_RequestText(TJC_HMI *hmi, const char *object);

/* State controls ------------------------------------------------------------ */
TJC_Status TJC_DualStateButton_SetValue(TJC_HMI *hmi, const char *object, bool on);
TJC_Status TJC_DualStateButton_SetText(TJC_HMI *hmi, const char *object, const char *text);
TJC_Status TJC_DualStateButton_RequestValue(TJC_HMI *hmi, const char *object);
TJC_Status TJC_DualStateButton_SetEnabled(TJC_HMI *hmi, const char *object, bool enabled);
TJC_Status TJC_Checkbox_SetChecked(TJC_HMI *hmi, const char *object, bool checked);
TJC_Status TJC_Checkbox_RequestChecked(TJC_HMI *hmi, const char *object);
TJC_Status TJC_Radio_SetChecked(TJC_HMI *hmi, const char *object, bool checked);
TJC_Status TJC_Radio_RequestChecked(TJC_HMI *hmi, const char *object);
TJC_Status TJC_Switch_SetValue(TJC_HMI *hmi, const char *object, bool on);
TJC_Status TJC_Switch_RequestValue(TJC_HMI *hmi, const char *object);

/* QR code, combo box and text-select controls ------------------------------ */
TJC_Status TJC_QRCode_SetText(TJC_HMI *hmi, const char *object, const char *text);
TJC_Status TJC_ComboBox_SetPath(TJC_HMI *hmi, const char *object, const char *path);
TJC_Status TJC_ComboBox_SetSelected(TJC_HMI *hmi, const char *object, uint16_t index);
TJC_Status TJC_ComboBox_RequestSelected(TJC_HMI *hmi, const char *object);
TJC_Status TJC_ComboBox_RequestText(TJC_HMI *hmi, const char *object);
TJC_Status TJC_ComboBox_SetExpanded(TJC_HMI *hmi, const char *object, bool expanded);
TJC_Status TJC_TextSelect_SetPath(TJC_HMI *hmi, const char *object, const char *path);
TJC_Status TJC_TextSelect_SetSelected(TJC_HMI *hmi, const char *object, uint16_t index);
TJC_Status TJC_TextSelect_RequestSelected(TJC_HMI *hmi, const char *object);
TJC_Status TJC_TextSelect_RequestText(TJC_HMI *hmi, const char *object);

#ifdef __cplusplus
}
#endif

#endif
