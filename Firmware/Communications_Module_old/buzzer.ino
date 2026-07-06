// Code for configuring hardware timer to drive piezo buzzer in complementary mode
// Author: Kennan Bays

// #### GLOBAL ####
HardwareTimer *buzzerTimer;
const uint8_t BUZZER_DUTY = 50;   // percent
const uint8_t BUZZER_DEADTIME = 72; // timer ticks (1uS @ 72MHz)

/*
* Initializes the buzzer system via remapping
* TIM1_CH1N & setting differential drive
*/
void initBuzzer() {
  // Enable AFIO clock for remapping
  __HAL_RCC_AFIO_CLK_ENABLE();

  // Remap TIM1_CH1N to PA7
  __HAL_AFIO_REMAP_TIM1_PARTIAL();

  buzzerTimer = new HardwareTimer(TIM1);
  //Overflow first, LL_TIM* last.
  buzzerTimer->setOverflow(1000, HERTZ_FORMAT); //1kHz default
  buzzerTimer->setCaptureCompare(3, BUZZER_DUTY, PERCENT_COMPARE_FORMAT);
  // Look into configuring TIM_OC manually?
  buzzerTimer->setMode(3, TIMER_OUTPUT_COMPARE_PWM1, BUZZER_A_PIN);
  buzzerTimer->setMode(3, TIMER_OUTPUT_COMPARE_PWM1, BUZZER_B_PIN);
  // Add deadtime gap to prevent shootthrough in h-bridge
  TIM1->BDTR = (TIM1->BDTR & ~TIM_BDTR_DTG_Msk) | (BUZZER_DEADTIME << TIM_BDTR_DTG_Pos);

  //LL_TIM_CC_EnableChannel(buzzerTimer->getHandle()->Instance, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N);
  //buzzerTimer->resume();
}//initBuzzer()


void stopBuzzer() {
  // Disables buzzer PWM
  LL_TIM_CC_DisableChannel(buzzerTimer->getHandle()->Instance, LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
}//stopBuzzer()

void startBuzzer() {
  // Enables buzzer PWM
  LL_TIM_CC_EnableChannel(buzzerTimer->getHandle()->Instance, LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
  buzzerTimer->resume();
}//startBuzzer()

void setBuzzerFreq(uint32_t f) {
  // Sets the PWM frequency for the buzzer driver
  buzzerTimer->setOverflow(f, HERTZ_FORMAT); 
}//setBuzzerFreq()