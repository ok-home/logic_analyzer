
#define MAX_PIN 16
/*
#define MIN_GPIO "-1"
#define MAX_GPIO "39"
#define MAX_SAMPLE_CNT "32764"
#define MIN_SAMPLE_CNT "100"
#define SAMPLE_STEP "100"
#define MAX_CLK "20000000"
#define MIN_CLK "5000"
#define CLK_STEP "1000"
*/
#define MIN_GPIO -1
#define MAX_GPIO 39
#define MAX_SAMPLE_CNT 32764
#define MIN_SAMPLE_CNT 100
#define MAX_CLK 20000000
#define MIN_CLK 5000




#define END_CFG_MSG "endcfg"

enum hdrID_name {
    HDR_CONFIG,
    HDR_START,
    HDR_DATALIST,
    HDR_OPTIONS
};

const char *hdrID[] = {
    [HDR_CONFIG] ="config",
    [HDR_START] = "start",
    [HDR_DATALIST]= "datalist",
    [HDR_OPTIONS] = "options"
};
enum rowID_name {
    ROW_PIN,
    ROW_TRG,
    ROW_EDG,
    ROW_SMP,
    ROW_CLK,
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
    [ROW_LST] = "lst",
    [ROW_START] = "beg",
    [ROW_ZOOM] = "zom",
    [ROW_MSMP] = "smp",
    [ROW_MCLK] = "clk",
    [ROW_SAVE] = "sav"
};
enum rowLabel{
    ROW_LBL_PIN,
    ROW_LBL_TRIG,
    ROW_LBL_EDGE,
    ROW_LBL_SAMPLE,
    ROW_LBL_CLOCK,
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
    [ROW_LBL_CLOCK] = "Sample Clock",
    [ROW_LBL_START] = "Start",
    [ROW_LBL_ZOOM] = "Zoom to fit",
    [ROW_LBL_MEASH_SAMPLES] = "Samples",
    [ROW_LBL_MEASH_CLOCK] = "Clock",
    [ROW_LBL_SAVE] = "Save to RowBin"
};

enum rowType {
    ROW_TEXT,
    ROW_NUMBER,
    ROW_BUTTON
};
const char *rowType[] = {
    [ROW_TEXT] = "text",
    [ROW_NUMBER] = "number",
    [ROW_BUTTON] = "button"
};

enum HTML_EVENT_IDX {
    NO_EVENT                = 0,
    CHECK_CFG_DATA_EVENT    = 1,
    START_EVENT             = 2,
    ZOOM_EVENT              = 3,
    SAVE_EVENT              = 4
};

/*
#define     NO_EVENT                = "0"
#define     CHECK_CFG_DATA_EVENT    = "1"
#define     START_EVENT             = "2"
#define     ZOOM_EVENT              = "3"
*/

enum DATALIST_ROW{
    DATALIST_PIN = 0,
    DATALIST_EDGE = 1,
    DATALIST_SAMPLE = 2,
    DATALIST_CLK = 3,
    DATALIST_MAX = 4
};
typedef struct {
    int32_t     value;
    char*       label;
    char*       datalist;
} options_list_t;

const options_list_t options[]={
    {-1,"Disable","lst00"},
    {0,"GPIO0","lst00"},
    {1,"GPIO1","lst00"},
    {2,"GPIO2","lst00"},
    {3,"GPIO3","lst00"},
    {4,"GPIO4","lst00"},
    {5,"GPIO5","lst00"},
    {6,"GPIO6","lst00"},
    {7,"GPIO7","lst00"},
    {8,"GPIO8","lst00"},
    {9,"GPIO9","lst00"},
    {10,"GPIO10","lst00"},
    {11,"GPIO11","lst00"},
    {12,"GPIO12","lst00"},
    {13,"GPIO13","lst00"},
    {14,"GPIO14","lst00"},
    {15,"GPIO15","lst00"},
    {16,"GPIO16","lst00"},
    {17,"GPIO17","lst00"},
    {18,"GPIO18","lst00"},
    {19,"GPIO19","lst00"},
    {21,"GPIO21","lst00"},
    {22,"GPIO22","lst00"},
    {23,"GPIO23","lst00"},
    {25,"GPIO25","lst00"},
    {26,"GPIO26","lst00"},
    {27,"GPIO27","lst00"},
    {32,"GPIO32","lst00"},
    {33,"GPIO33","lst00"},
    {34,"GPIO34","lst00"},
    {35,"GPIO35","lst00"},
    {36,"GPIO36","lst00"},
    {37,"GPIO37","lst00"},
    {38,"GPIO38","lst00"},
    {39,"GPIO39","lst00"},
    {1,"POS_EDGE","lst01"},
    {2,"NEG_EDGE","lst01"},
    {100,"","lst02"},
    {200,"","lst02"},
    {500,"","lst02"},
    {1000,"","lst02"},
    {2000,"","lst02"},
    {5000,"","lst02"},
    {10000,"","lst02"},
    {20000,"","lst02"},
    {30000,"","lst02"},
    {32764,"","lst02"},
    {5000,"5 kHz","lst03"},
    {10000,"10 kHz","lst03"},
    {20000,"20 kHz","lst03"},
    {50000,"50 kHz","lst03"},
    {100000,"100 kHz","lst03"},
    {200000,"200 kHz","lst03"},
    {500000,"500 kHz","lst03"},
    {1000000,"1 MHz","lst03"},
    {2000000,"2 MHz","lst03"},
    {5000000,"5 MHz","lst03"},
    {10000000,"10 MHz","lst03"},
    {20000000,"20 MHz","lst03"}
};