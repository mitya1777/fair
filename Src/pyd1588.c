#include "pyd1588.h"
#include "main.h"

//#define MSP430                                                                     //  uncomment the string if really use msp430

/*   PYD 1588 configuration parameters   */

#define FORCED_READOUT                 0x00
#define INTERRUPTED_READOUT            0x01
#define WAKE_UP_MODE                   0x02

#define PIR_BPF                        0x00
#define PIR_LPF                        0x01
#define TEMPERATURE_SENSOR             0x03


#define SI_LINE_LOW_TO_HIGH              asm("BIS.B #04h, &P6OUT");                  //  low to high state the serial in interface line switching
#define SI_LINE_HIGH_TO_LOW              asm("BIC.B #04h, &P6OUT");                  //  high to low state the serial in interface line switching
#define DL_LINE_LOW_TO_HIGH              asm("BIS.B #01h, &P1OUT");                  //  low to high state the direct link interface line switching
#define DL_LINE_HIGH_TO_LOW              asm("BIC.B #01h, &P1OUT");                  //  high to low state the direct link interface line switching
#define DL_LINE_INPUT_MODE               asm("BIC.B #04h, &P6DIR");                  //  configuration pin mode as input
#define DL_LINE_OUTPUT_MODE              asm("BIS.B #04h, &P6DIR");                  //  configuration pin mode as output



/*     Serial In interface realization     */

void load_parameters(uint8_t counting_mode,                                          //  pulse counting method with the signal sign accounting
                     uint8_t cut_off_frequency,                                      //  the high pass cut-off frequency in the band pass 1lter
                     uint8_t signal_source,                                          //  a choice of three data sources is available (PIR BPF, PIR LPF, temperature sensor)
                     uint8_t operation_mode,                                         //  three sensor operation modes are available
                     uint8_t window_time,                                            //  the pulse counter increment is performed during the selected time
                     uint8_t pulse_counter,                                          //  the amount of pulses above the threshold
                     uint8_t blind_time,                                             //  for avoid immediate re-triggering after a motion event detection
                     uint8_t threschold,                                             //  threshold voltage
                     uint16_t t_shd,                                                 //  data bit time (not less than 80 us)
                     uint16_t t_slt)                                                 //  hold time for detector data latching
{

    DL_LINE_HIGH_TO_LOW;

    SI_LINE_HIGH_TO_LOW;                                                             //      initialization impulse
    SI_LINE_LOW_TO_HIGH;
    SI_LINE_HIGH_TO_LOW;

    /*	parameters setting	*/

    uint8_t p_counting_mode     = counting_mode;
    uint8_t p_cut_off_frequency = cut_off_frequency;
    uint8_t p_signal_source     = signal_source;
    uint8_t p_operation_mode    = operation_mode;
    uint8_t p_window_time       = window_time;
    uint8_t p_pulse_counter     = pulse_counte;
    uint8_t p_blind_time        = blind_time;
    uint8_t p_threschold        = threschold;

    uint16_t loaded_data[0x02] = {0x00};
    uint8_t *bit_field[0x04] = {0x00};

    bit_field[0x00] = &p_pulse_counter;
    bit_field[0x01] = &p_window_time;
    bit_field[0x02] = &p_operation_mode;
    bit_field[0x03] = &p_signal_source;


    #ifdef MSP430                                                                   //  lsb (msp430) to msb (pyd1588) data format converting

    p_blind_time = ((p_blind_time & 0x0A) >> 0x01) | ((p_blind_time & 0x05) << 0x01);
    p_blind_time = ((p_blind_time & 0x0C) >> 0x02) | ((p_blind_time & 0x03) << 0x02);
    
    for(uint16_t i = 0x00; i < 0x04; i ++)    {
      *bit_field[i] = ((*bit_field[i] & 0xAA) >> 0x01) | ((*bit_field[i] & 0x55) << 0x01);
    }
    
    lsb_to_msb(&p_threschold);

    #endif
        
    loaded_data[0] |= p_threschold;
    loaded_data[0] |= (p_blind_time        << 0x08);
    loaded_data[0] |= (p_pulse_counter     << 0x0C);
    loaded_data[0] |= (p_window_time       << 0x0E);
    loaded_data[1] |= p_operation_mode;
    loaded_data[1] |= (p_signal_source     << 0x02);
    loaded_data[1] |= (p_cut_off_frequency << 0x06);
    loaded_data[1] |= (p_counting_mode     << 0x08);
                 
    for(uint8_t i = 0x00; i < 0x02; i ++)     {
          
      for(uint8_t k = 0x00; k < 0x10; k ++)     {

        if((k == 0x09) & (i == 0x01))     {
         
          SI_LINE_HIGH_TO_LOW;
          break;
        }
        
        if((loaded_data[i] & (0x01 << k)) == 0x00) {
          
          SI_LINE_HIGH_TO_LOW;
          SI_LINE_LOW_TO_HIGH
          SI_LINE_HIGH_TO_LOW;
        }

        else {
          
          SI_LINE_HIGH_TO_LOW;
          SI_LINE_LOW_TO_HIGH;
        }
                    
        bit_time(t_shd);
      }
    }
    SI_LINE_HIGH_TO_LOW;
    bit_time(t_slt);
    DL_LINE_INPUT_MODE;
}

        
uint16_t get_pyd_data(uint16_t t_ds,                                            //  data set-up time
                      uint16_t t_bit,                                           //  bit time
                      uint16_t t_ra)                                            //  abortion time
{
  uint16_t returned_data = 0x00;
  uint8_t  buffer[0x05] = {0x00};
  uint8_t  bit = 0x00;

  DL_LINE_OUTPUT_MODE;                                                          //      initialization impulse
  DL_LINE_HIGH_TO_LOW;
  DL_LINE_LOW_TO_HIGH;

  bit_time(t_ds);                                                               //      initialization impulse duration

  for(uint8_t i = 0x00; i <= 0x04; i ++)  {

    for(uint8_t k = 0x08; k > 0x00; k --)    {
     DL_LINE_HIGH_TO_LOW;
     DL_LINE_LOW_TO_HIGH;
     DL_LINE_INPUT_MODE;
      
     bit_time(t_bit);
      
    ((P6IN & 0x04) == 0x00) ? (bit = 0x00) : (bit = 0x01);
      buffer[i] |= (bit << (k-1));

      DL_LINE_HIGH_TO_LOW;
      DL_LINE_OUTPUT_MODE;
    }
  }

  DL_LINE_HIGH_TO_LOW;
  DL_LINE_OUTPUT_MODE;  
  
  if((buffer[0] & 0x80) != 0x00)        {                                       //      correct data checking

    returned_data = (((buffer[0] & 0x7F) << 0x07) |
                      (buffer[1] >> 0x01));
    
    if((buffer[0] & 0x40) != 0x00)        {                                     //      data sign definition
      buffer[0] = (~buffer[0]);
      buffer[1] = ((~buffer[1]));
    
     returned_data = (((buffer[0] & 0x7F) << 0x07) |                            //      negative ADC sensor data
                      (buffer[1] >> 0x01));
    }
  }

  else  {
    returned_data = 0x00;
  }

  bit_time(t_ra);
  
  return returned_data;
}        


void lsb_to_msb(uint8_t *x)
{
  uint16_t y = 0x00;

  *x = ((*x & 0xAA) >> 0x01) | ((*x & 0x55) << 0x01);
   y = ((*x & 0x30) >> 0x02) | ((*x & 0x0C) << 0x02);
  *x = y | (*x & 0xC3);
   y = ((*x & 0xC0) >> 0x06) | ((*x & 0x03) << 0x06);
  *x = y | (*x & 0x3C);
}


void bit_time(uint16_t cycles_quantity)                                         //  should be better a free timer usage
{
  while(cycles_quantity)  {
    cycles_quantity --;
  }
}


#ifdef MSP430

/*    the maximum msp430 clock frequency setting    */
void burst_frequency(void)
{
  DCOCTL  |= 0xE0;                                                              //      maximum Digitally Controlled Oscillator frequency
  BCSCTL1 &= 0xCF;                                                              //      divider reset
  BCSCTL1 |= 0x87;                                                              //      the hoghest nominal frequency is selected (XT2 is off)
  BCSCTL2 |= 0x01;                                                              //      using externel resistor Rosc
}

#endif