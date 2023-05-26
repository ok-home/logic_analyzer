#include "soc/dport_reg.h"
#include "soc/gpio_reg.h"
#include "soc/soc.h"
#include "rom/ets_sys.h"
#include "logic_analyzer_ll.h"
#include "logic_analyzer_hi_lewel_interrupt.h"
#include "esp_log.h"


hi_interrupt_state_t la_hi_interrupt_state = {
    .cpu = 2,
    .dport_int_map_data_disable = 6,
};

// _DPORT_REG_READ(DPORT_PRO_GPIO_INTERRUPT_MAP_REG), _DPORT_REG_READ(DPORT_APP_GPIO_INTERRUPT_MAP_REG)
void ll_triggered_isr_alloc(void *p)
{
    ESP_LOGI("AISR","start");
    ESP_INTR_DISABLE(HI_INTERRUPT_NUMBER);
    intr_matrix_set(la_hi_interrupt_state.cpu, ETS_GPIO_INTR_SOURCE, HI_INTERRUPT_NUMBER);   
    ESP_INTR_ENABLE(HI_INTERRUPT_NUMBER);
    REG_WRITE(la_hi_interrupt_state.gpio_pin_cfg_reg,REG_READ(la_hi_interrupt_state.gpio_pin_cfg_reg)|(~la_hi_interrupt_state.gpio_pin_cfg_int_ena_core_bit));
    ESP_LOGI("AISR","cfg after %lx",REG_READ(la_hi_interrupt_state.gpio_pin_cfg_reg));
    ESP_LOGI("AISR","exit");
    vTaskDelay(10); // debug delay
    vTaskDelete(NULL);
}

void ll_hi_lewel_triggered_isr_start(int pin_trigger,int trigger_edge)
{
    int int_free_pro = 0;
    int int_free_app = 0;
    if (pin_trigger<0) return;
    la_hi_interrupt_state.cpu = 2;
    switch (_DPORT_REG_READ(DPORT_PRO_GPIO_INTERRUPT_MAP_REG))
    {
        case 6:
        case 7:
        case 11:
        case 15:
        case 16:
        case 29:
        int_free_pro = 1;
        la_hi_interrupt_state.cpu = 0; // use hi-level int on pro cpu (0)
        break;
        default:
        int_free_pro = 0;
        break;
    }
    switch (_DPORT_REG_READ(DPORT_APP_GPIO_INTERRUPT_MAP_REG))
    {
        case 6:
        case 7:
        case 11:
        case 15:
        case 16:
        case 29:
        int_free_app = 1;
        la_hi_interrupt_state.cpu = 1; // use hi-level int on app cpu (1)
        break;
        default:
        int_free_app = 0;
        break;
    }
    ESP_LOGI("TISR","pro=%ld app=%ld cpu=%ld",_DPORT_REG_READ(DPORT_PRO_GPIO_INTERRUPT_MAP_REG),_DPORT_REG_READ(DPORT_APP_GPIO_INTERRUPT_MAP_REG),la_hi_interrupt_state.cpu);
    if((int_free_app|int_free_pro) == 0) // all gpio int ( app&pro ) predefined - slow gpio int
    {
        ESP_LOGI("TISR","slow gpio interrupt");
        gpio_install_isr_service(0); // default
        gpio_set_intr_type(pin_trigger, trigger_edge);
        gpio_isr_handler_add(pin_trigger, la_ll_trigger_isr, (void *)pin_trigger);
        gpio_intr_enable(pin_trigger);
        return;
    } 
    else
    {
        ESP_LOGI("TISR","fast gpio interrupt");
        la_hi_interrupt_state.dport_int_map_data_disable = 6; // soft interrupt - disable gpio interrupt
        la_hi_interrupt_state.dport_int_map_reg = (la_hi_interrupt_state.cpu == 0) ? DPORT_PRO_GPIO_INTERRUPT_MAP_REG  : DPORT_APP_GPIO_INTERRUPT_MAP_REG; // app/pro map register
        la_hi_interrupt_state.dport_int_stat_reg = (la_hi_interrupt_state.cpu == 0) ? DPORT_PRO_INTR_STATUS_0_REG : DPORT_APP_INTR_STATUS_0_REG; // app/pro int status dport register
        la_hi_interrupt_state.gpio_mask = (pin_trigger < 32) ? 1<<pin_trigger : 1<<(pin_trigger-32); // hi/low interupt mask ( 0-31 )( 32-39 )
        la_hi_interrupt_state.gpio_stat_reg = (pin_trigger < 32) ? GPIO_STATUS_REG : GPIO_STATUS1_REG ; // hi/low interupt status register ( 0-31 )( 32-39 )
        la_hi_interrupt_state.gpio_stat_clr_reg = (pin_trigger < 32) ? GPIO_STATUS_W1TC_REG : GPIO_STATUS1_W1TC_REG ; // hi/low interupt status clear register ( 0-31 )( 32-39 )
        la_hi_interrupt_state.gpio_pin_cfg_reg = GPIO_PIN0_REG+(4*pin_trigger); // gpio config register corresponded with trigger pin
        la_hi_interrupt_state.gpio_pin_cfg_int_ena_core_bit = (la_hi_interrupt_state.cpu == 0) ? ~(1<<15) : ~(1<<13); // app/pro enable interrupt in cfg gpio register - 0 for fast clear
        la_hi_interrupt_state.i2s_set_vsync_reg = GPIO_FUNC191_IN_SEL_CFG_REG;  // i2s0/i2s1
        la_hi_interrupt_state.i2s_set_vsync_bit = HI_INTERRUPT_SET_VSYNC;

        ESP_LOGI("TISR","v_sync check bit=%lx vsync=%lx hsync=%lx",la_hi_interrupt_state.i2s_set_vsync_bit,REG_READ(la_hi_interrupt_state.i2s_set_vsync_reg),REG_READ(GPIO_FUNC190_IN_SEL_CFG_REG));
        ESP_LOGI("TISR","cfg mapreg=%lx dstatreg=%lx mask=%lx gstatreg=%lx sclrreg=%lx cfgreg=%lx intena=%lx",
            la_hi_interrupt_state.dport_int_map_reg,
            la_hi_interrupt_state.dport_int_stat_reg,
            la_hi_interrupt_state.gpio_mask,
            la_hi_interrupt_state.gpio_stat_reg,
            la_hi_interrupt_state.gpio_stat_clr_reg,
            la_hi_interrupt_state.gpio_pin_cfg_reg,
            la_hi_interrupt_state.gpio_pin_cfg_int_ena_core_bit);

        // for shared interrupt (gpio pin interrupt defined on both cores )  ( simultaneously on 2 cores )
        // if interrupt edge predefined and interrupt enable on core
        // interrupt edge not change
        
        // mask ????????????????????????????? clear ???????????????

        
    ESP_LOGI("TISR","cfg before %lx",REG_READ(la_hi_interrupt_state.gpio_pin_cfg_reg));
        uint32_t reg_cfg = REG_READ(la_hi_interrupt_state.gpio_pin_cfg_reg);
        if((reg_cfg & 0x1f) == 0) // all gpio  interrupt  disable
        {
            reg_cfg &= ~(0x7<<7); // clear edge
        }
        if((reg_cfg & ( 0x7<<7)) == 0 ) // intr edge not defined
            {
                reg_cfg |= trigger_edge<<7; // set edge
            }

        REG_WRITE(la_hi_interrupt_state.gpio_pin_cfg_reg,reg_cfg);
        ESP_LOGI("TISR","cfg after %lx",REG_READ(la_hi_interrupt_state.gpio_pin_cfg_reg));

    // alloc hi level int on free core        
        xTaskCreatePinnedToCore(ll_triggered_isr_alloc, "trigg_alloc", 4096, NULL, /*uxTaskPriorityGet(NULL)*/20, NULL, la_hi_interrupt_state.cpu);
        vTaskDelay(10); // debug delay
        ESP_LOGI("TISR","exit isr cfg");
    }

}

