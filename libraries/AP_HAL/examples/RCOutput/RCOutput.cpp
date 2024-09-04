/*
  simple test of RC output interface
  Attention: If your board has safety switch,
  don't forget to push it to enable the PWM output.
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

// we need a boardconfig created so that the io processor's enable
// parameter is available

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_IOMCU/AP_IOMCU.h>
AP_BoardConfig BoardConfig;

///////////////////////////////////////////////////////////////////////////////////////

#define ESC_HZ 490
#define SERVO_HZ 50
///////////////////////////////////////////////////////////////////////////////////////


void setup();
void loop();
void singleSignal(uint16_t _pwm);

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

void setup (void)
{
    hal.console->printf("Starting AP_HAL::RCOutput test\n");
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    BoardConfig.init();
#endif

    // float SET_PWM_PARAMETER = 14;
    // AP_Param::set_object_value(&BoardConfig, BoardConfig.var_info, "PWM_COUNT", SET_PWM_PARAMETER);
    // AP_Param::set_object_value(&BoardConfig, BoardConfig.var_info, "SAFETYENABLE", 0);
    // AP_Param::set_object_value(&BoardConfig, BoardConfig.var_info, "SAFETYOPTION", 0);// 0 1 2
    // AP_Param::set_object_value(&BoardConfig, BoardConfig.var_info, "SAFETY_MASK", 8191);

    hal.rcout->set_freq(0xFF, SERVO_HZ);
    hal.util->set_soft_armed(true);

    // enable
    for (uint8_t i=0; i < 16; i++) {
        hal.rcout->enable_ch(i);
    }

    hal.scheduler->delay(1000);
}


static uint32_t hop_loop_cnt;

void loop (void)
{

    hal.console->printf(" loop cnt : %lu \n",hop_loop_cnt++);

    hal.rcout->force_safety_off();

    // 1. 
    // upDownSignal();

    // 2. 
    singleSignal(1100);
    singleSignal(1000);

    // if(hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_ARMED){
    //     hal.console->printf("SAFETY_ARMED \n");
    // }
    // if(hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED){
    //     hal.console->printf("SAFETY_DISARMED \n");
    // }
    // if(hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_NONE){
    //     hal.console->printf("SAFETY_NONE \n");
    // }

    // hal.console->printf("ARMED?: %d\n",hal.util->get_soft_armed());

    hal.scheduler->delay(5); //was 5

}

static uint16_t pwm = 1100;
static int8_t delta = 1;

void upDownSignal()
{
    for (uint8_t i=0; i < 16; i++) {
        hal.rcout->write(i, pwm);
        pwm += delta;
        if (delta > 0 && pwm >= 1200) {
            delta = -1;
            hal.console->printf("decreasing | pwm: %u\n",pwm);
        } else if (delta < 0 && pwm <= 1000) {
            delta = 1;
            hal.console->printf("increasing | pwm: %u\n",pwm);
            
        }
        hal.scheduler->delay(10);
    }
}

void singleSignal(uint16_t _pwm)
{
    for(uint8_t j = 0 ; j < 30 ; j++){
        for(uint8_t i = 0 ; i < 16 ; i++){
            hal.rcout->enable_ch(i);
            hal.rcout->write(i, _pwm);
            hal.scheduler->delay(5);
        }
    }

}


AP_HAL_MAIN();
