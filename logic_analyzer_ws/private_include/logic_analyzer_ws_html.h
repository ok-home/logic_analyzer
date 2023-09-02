#pragma once

#define MIN_GPIO LA_HW_MIN_GPIO
#define MAX_GPIO LA_HW_MAX_GPIO

#define END_CFG_MSG "endcfg"
#define END_HW_REQ_MSG "endhwr"
#define REDRAW_MSG "redraw"
#define CLEAR_DIV_MSG "clrdiv"

enum hdrID_name
{
    HDR_CONFIG,
    HDR_START,
    HDR_DATALIST,
    HDR_OPTIONS
};

const char *hdrID[] = {
    [HDR_CONFIG] = "config",
    [HDR_START] = "start",
    [HDR_DATALIST] = "datalist",
    [HDR_OPTIONS] = "options"};
enum rowID_name
{
    ROW_PIN,
    ROW_TRG,
    ROW_EDG,
    ROW_SMP,
    ROW_CLK,
    ROW_TIMEOUT,
    ROW_CHANNELS,
    ROW_PSRAM,
    ROW_LST,
    ROW_START,
    ROW_ZOOM,
    ROW_MSMP,
    ROW_MCLK,
    ROW_SAVE
};
const char *rowID[] = {
    [ROW_PIN] = "pin",
    [ROW_TRG] = "trg",
    [ROW_EDG] = "edg",
    [ROW_SMP] = "smp",
    [ROW_CLK] = "clk",
    [ROW_TIMEOUT] = "tmo",
    [ROW_CHANNELS] = "chn",
    [ROW_PSRAM] = "ram",
    [ROW_LST] = "lst",
    [ROW_START] = "beg",
    [ROW_ZOOM] = "zom",
    [ROW_MSMP] = "smp",
    [ROW_MCLK] = "clk",
    [ROW_SAVE] = "sav"};
enum rowLabel
{
    ROW_LBL_PIN,
    ROW_LBL_TRIG,
    ROW_LBL_EDGE,
    ROW_LBL_SAMPLE,
    ROW_LBL_CLOCK,
    ROW_LBL_TIMEOUT,
    ROW_LBL_CHANNELS,
    ROW_LBL_PSRAM,
    ROW_LBL_START,
    ROW_LBL_ZOOM,
    ROW_LBL_MEASH_SAMPLES,
    ROW_LBL_MEASH_CLOCK,
    ROW_LBL_SAVE
};
const char *rowLbl[] = {
    [ROW_LBL_PIN] = "Сhannel ",
    [ROW_LBL_TRIG] = "Trigger",
    [ROW_LBL_EDGE] = "Trigger Edge",
    [ROW_LBL_SAMPLE] = "Sample Count",
    [ROW_LBL_CLOCK] = "Sample Clock Hz",
    [ROW_LBL_TIMEOUT] = "Timeout Sek",
    [ROW_LBL_CHANNELS] = "Channels",
    [ROW_LBL_PSRAM] = "Psram",
    [ROW_LBL_START] = "Start",
    [ROW_LBL_ZOOM] = "Zoom to fit",
    [ROW_LBL_MEASH_SAMPLES] = "Samples",
    [ROW_LBL_MEASH_CLOCK] = "Clock Hz",
    [ROW_LBL_SAVE] = "Save to RowBin"};

enum rowType
{
    ROW_TEXT,
    ROW_NUMBER,
    ROW_BUTTON
};
const char *rowType[] = {
    [ROW_TEXT] = "text",
    [ROW_NUMBER] = "number",
    [ROW_BUTTON] = "button"};

enum HTML_EVENT_IDX
{
    NO_EVENT = 0,
    CHECK_CFG_DATA_EVENT = 1,
    START_EVENT = 2,
    ZOOM_EVENT = 3,
    SAVE_EVENT = 4,
    GET_HW_PARAM = 5
};

enum DATALIST_ROW
{
    DATALIST_PIN = 0,
    DATALIST_EDGE = 1,
    DATALIST_SAMPLE = 2,
    DATALIST_CLK = 3,
    DATALIST_TIMEOUT = 4,
    DATALIST_CHANNELS = 5,
    DATALIST_PSRAM = 6,
    DATALIST_MAX = 7
};
typedef struct
{
    int32_t value;
    char *label;
    char *datalist;
} options_list_t;

#ifdef CONFIG_IDF_TARGET_ESP32
const options_list_t pin_options[] =
    { // DATALIST_PIN lst00
        {-1, "Disable", "lst00"},
        {0, "GPIO0", "lst00"},
        {1, "GPIO1", "lst00"},
        {2, "GPIO2", "lst00"},
        {3, "GPIO3", "lst00"},
        {4, "GPIO4", "lst00"},
        {5, "GPIO5", "lst00"},
        {6, "GPIO6", "lst00"},
        {7, "GPIO7", "lst00"},
        {8, "GPIO8", "lst00"},
        {9, "GPIO9", "lst00"},
        {10, "GPIO10", "lst00"},
        {11, "GPIO11", "lst00"},
        {12, "GPIO12", "lst00"},
        {13, "GPIO13", "lst00"},
        {14, "GPIO14", "lst00"},
        {15, "GPIO15", "lst00"},
        {16, "GPIO16", "lst00"},
        {17, "GPIO17", "lst00"},
        {18, "GPIO18", "lst00"},
        {19, "GPIO19", "lst00"},
        {21, "GPIO21", "lst00"},
        {22, "GPIO22", "lst00"},
        {23, "GPIO23", "lst00"},
        {25, "GPIO25", "lst00"},
        {26, "GPIO26", "lst00"},
        {27, "GPIO27", "lst00"},
        {32, "GPIO32", "lst00"},
        {33, "GPIO33", "lst00"},
        {34, "GPIO34", "lst00"},
        {35, "GPIO35", "lst00"},
        {36, "GPIO36", "lst00"},
        {37, "GPIO37", "lst00"},
        {38, "GPIO38", "lst00"},
        {39, "GPIO39", "lst00"},
        {0, "", ""}};
const options_list_t edge_options[] =
    { // DATALIST_EDGE lst01
        {1, "POS_EDGE", "lst01"},
        {2, "NEG_EDGE", "lst01"},
        {0, "", ""}};
const options_list_t sample_options[] =
    { // DATALIST_SAMPLE lst02
        {100, "100", "lst02"},
        {200, "200", "lst02"},
        {500, "500", "lst02"},
        {1000, "1 000", "lst02"},
        {2000, "2 000", "lst02"},
        {5000, "5 000", "lst02"},
        {10000, "10 000", "lst02"},
        {20000, "20 000", "lst02"},
        {30000, "30 000", "lst02"},
        {40000, "40 000", "lst02"},
        {50000, "50 000", "lst02"},
        {60000, "60 000", "lst02"},
        {0, "", ""}};
const options_list_t clk_options[] =
    { // DATALIST_CLK lst03
        {5000, "5 kHz", "lst03"},
        {10000, "10 kHz", "lst03"},
        {20000, "20 kHz", "lst03"},
        {50000, "50 kHz", "lst03"},
        {100000, "100 kHz", "lst03"},
        {200000, "200 kHz", "lst03"},
        {500000, "500 kHz", "lst03"},
        {1000000, "1 MHz", "lst03"},
        {2000000, "2 MHz", "lst03"},
        {5000000, "5 MHz", "lst03"},
        {10000000, "10 MHz", "lst03"},
        {20000000, "20 MHz", "lst03"},
        {40000000, "40 MHz", "lst03"},
        {0, "", ""}};
const options_list_t timeout_options[] =
    { // DATALIST_TIMEOUT lst04
        {1, "1 Sek", "lst04"},
        {2, "2 Sek", "lst04"},
        {5, "5 Sek", "lst04"},
        {10, "10 Sek", "lst04"},
        {20, "20 Sek", "lst04"},
        {60, "60 Sek", "lst04"},
        {-1, "No Timeout", "lst04"},
        {0, "", ""}};
const options_list_t channel_options[] =
    { // DATALIST_CHANNELS lst05
        {16, "16 ch only on ESP32", "lst05"},
        {0, "", ""}};
const options_list_t psram_options[] =
    { // DATALIST_PSRAM lst06
        {0, "RAM only on ESP32", "lst06"},
        {0, "", ""}};

#endif
#ifdef CONFIG_IDF_TARGET_ESP32S3
const options_list_t pin_options[] =
    { // DATALIST_PIN lst00
        {-1, "Disable", "lst00"},
        {0, "GPIO0", "lst00"},
        {1, "GPIO1", "lst00"},
        {2, "GPIO2", "lst00"},
        {3, "GPIO3", "lst00"},
        {4, "GPIO4", "lst00"},
        {5, "GPIO5", "lst00"},
        {6, "GPIO6", "lst00"},
        {7, "GPIO7", "lst00"},
        {8, "GPIO8", "lst00"},
        {9, "GPIO9", "lst00"},
        {10, "GPIO10", "lst00"},
        {11, "GPIO11", "lst00"},
        {12, "GPIO12", "lst00"},
        {13, "GPIO13", "lst00"},
        {14, "GPIO14", "lst00"},
        {15, "GPIO15", "lst00"},
        {16, "GPIO16", "lst00"},
        {17, "GPIO17", "lst00"},
        {18, "GPIO18", "lst00"},
        {19, "GPIO19", "lst00"},
        {20, "GPIO20", "lst00"},
        {21, "GPIO21", "lst00"},
        {26, "GPIO26", "lst00"},
        {27, "GPIO27", "lst00"},
        {28, "GPIO28", "lst00"},
        {29, "GPIO29", "lst00"},
        {30, "GPIO30", "lst00"},
        {31, "GPIO31", "lst00"},
        {32, "GPIO32", "lst00"},
        {33, "GPIO33", "lst00"},
        {34, "GPIO34", "lst00"},
        {35, "GPIO35", "lst00"},
        {36, "GPIO36", "lst00"},
        {37, "GPIO37", "lst00"},
        {38, "GPIO38", "lst00"},
        {39, "GPIO39", "lst00"},
        {40, "GPIO40", "lst00"},
        {41, "GPIO41", "lst00"},
        {42, "GPIO42", "lst00"},
        {43, "GPIO43", "lst00"},
        {44, "GPIO44", "lst00"},
        {45, "GPIO45", "lst00"},
        {46, "GPIO46", "lst00"},
        {47, "GPIO47", "lst00"},
        {48, "GPIO48", "lst00"},
        {-2, "", ""}};
const options_list_t edge_options[] =
    { // DATALIST_EDGE lst01
        {1, "POS_EDGE", "lst01"},
        {2, "NEG_EDGE", "lst01"},
        {2, "", ""}};
const options_list_t sample_options[] =
    { // DATALIST_SAMPLE lst02
        {100, "100", "lst02"},
        {200, "200", "lst02"},
        {500, "500", "lst02"},
        {1000, "1k", "lst02"},
        {2000, "2k", "lst02"},
        {5000, "5k", "lst02"},
        {10000, "10k", "lst02"},
        {20000, "20k", "lst02"},
        {30000, "30k", "lst02"},
        {40000, "40k", "lst02"},
        {50000, "50k", "lst02"},
        {60000, "60k", "lst02"},
        {70000, "70k", "lst02"},
        {80000, "80k", "lst02"},
        {90000, "90k", "lst02"},
        {100000, "100k", "lst02"},
        {120000, "120k", "lst02"},
        {140000, "140k", "lst02"},
        {160000, "160k", "lst02"},
        {180000, "180k", "lst02"},
        {200000, "200k", "lst02"},
        {500000, "500k", "lst02"},
        {1000000, "1m", "lst02"},
        {2000000, "2m", "lst02"},
        {3000000, "3m", "lst02"},
        {4000000, "4m", "lst02"},
        {5000000, "5m", "lst02"},
        {6000000, "6m", "lst02"},
        {7000000, "7m", "lst02"},
        {8000000, "8m", "lst02"},
        {-2, "", ""}};
const options_list_t clk_options[] =
    { // DATALIST_CLK lst03
        {5000, "5 kHz", "lst03"},
        {10000, "10 kHz", "lst03"},
        {20000, "20 kHz", "lst03"},
        {50000, "50 kHz", "lst03"},
        {100000, "100 kHz", "lst03"},
        {200000, "200 kHz", "lst03"},
        {500000, "500 kHz", "lst03"},
        {1000000, "1 MHz", "lst03"},
        {2000000, "2 MHz", "lst03"},
        {5000000, "5 MHz", "lst03"},
        {10000000, "10 MHz", "lst03"},
        {20000000, "20 MHz", "lst03"},
        {40000000, "40 MHz", "lst03"},
        {80000000, "80 MHz", "lst03"},
        {-2, "", ""}};
const options_list_t timeout_options[] =
    { // DATALIST_TIMEOUT lst04
        {1, "1 Sek", "lst04"},
        {2, "2 Sek", "lst04"},
        {5, "5 Sek", "lst04"},
        {10, "10 Sek", "lst04"},
        {20, "20 Sek", "lst04"},
        {60, "60 Sek", "lst04"},
        {-1, "No Timeout", "lst04"},
        {-2, "", ""}};
const options_list_t channel_options[] =
    { // DATALIST_CHANNELS lst05
        {8, "8 ch on ESP32S3", "lst05"},
        {16, "16 ch on ESP32S3", "lst05"},
        {-2, "", ""}};
const options_list_t psram_options[] =
    { // DATALIST_PSRAM lst06
        {0, "RAM on ESP32S3", "lst06"},
        {1, "PSRAM on ESP32S3", "lst06"},
        {-2, "", ""}};

#endif
#ifdef CONFIG_IDF_TARGET_ESP32C3
const options_list_t pin_options[] =
    { // DATALIST_PIN lst00
        {-1, "Disable", "lst00"},
        {0, "GPIO0", "lst00"},
        {1, "GPIO1", "lst00"},
        {2, "GPIO2", "lst00"},
        {3, "GPIO3", "lst00"},
        {4, "GPIO4", "lst00"},
        {5, "GPIO5", "lst00"},
        {6, "GPIO6", "lst00"},
        {7, "GPIO7", "lst00"},
        {8, "GPIO8", "lst00"},
        {9, "GPIO9", "lst00"},
        {10, "GPIO10", "lst00"},
        {11, "GPIO11", "lst00"},
        {12, "GPIO12", "lst00"},
        {13, "GPIO13", "lst00"},
        {14, "GPIO14", "lst00"},
        {15, "GPIO15", "lst00"},
        {16, "GPIO16", "lst00"},
        {17, "GPIO17", "lst00"},
        {18, "GPIO18", "lst00"},
        {19, "GPIO19", "lst00"},
        {20, "GPIO20", "lst00"},
        {21, "GPIO21", "lst00"},
        {0, "", ""}};
const options_list_t edge_options[] =
    { // DATALIST_EDGE lst01
        {1, "POS_EDGE", "lst01"},
        {2, "NEG_EDGE", "lst01"},
        {0, "", ""}};
const options_list_t sample_options[] =
    { // DATALIST_SAMPLE lst02
        {100, "100", "lst02"},
        {200, "200", "lst02"},
        {500, "500", "lst02"},
        {1000, "1 000", "lst02"},
        {2000, "2 000", "lst02"},
        {5000, "5 000", "lst02"},
        {10000, "10 000", "lst02"},
        {20000, "20 000", "lst02"},
        {30000, "30 000", "lst02"},
        {40000, "40 000", "lst02"},
        {50000, "50 000", "lst02"},
        {60000, "60 000", "lst02"},
        {0, "", ""}};
const options_list_t clk_options[] =
    { // DATALIST_CLK lst03
        {5000, "5 kHz", "lst03"},
        {10000, "10 kHz", "lst03"},
        {20000, "20 kHz", "lst03"},
        {50000, "50 kHz", "lst03"},
        {100000, "100 kHz", "lst03"},
        {200000, "200 kHz", "lst03"},
        {500000, "500 kHz", "lst03"},
        {1000000, "1 MHz", "lst03"},
        {2000000, "2 MHz", "lst03"},
        {5000000, "5 MHz", "lst03"},
        {10000000, "10 MHz", "lst03"},
        {20000000, "20 MHz", "lst03"},
        {40000000, "40 MHz", "lst03"},
        {80000000, "80 MHz", "lst03"},
        {0, "", ""}};
const options_list_t timeout_options[] =
    { // DATALIST_TIMEOUT lst04
        {1, "1 Sek", "lst04"},
        {2, "2 Sek", "lst04"},
        {5, "5 Sek", "lst04"},
        {10, "10 Sek", "lst04"},
        {20, "20 Sek", "lst04"},
        {60, "60 Sek", "lst04"},
        {-1, "No Timeout", "lst04"},
        {0, "", ""}};
const options_list_t channel_options[] =
    { // DATALIST_CHANNELS lst05
        {4, "4 ch only on ESP32C3", "lst05"},
        {0, "", ""}};
const options_list_t psram_options[] =
    { // DATALIST_PSRAM lst06
        {0, "RAM only on ESP32C3", "lst06"},
        {0, "", ""}};

#endif
