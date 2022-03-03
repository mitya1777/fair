#ifndef	__PYD1588_H
#define __PYD1588_H

void load_parameters(uint8_t counting_mode,                                          //  pulse counting method with the signal sign accounting
                     uint8_t cut_off_frequency,                                      //  the high pass cut-off frequency in the band pass 1lter
                     uint8_t signal_source,                                          //  a choice of three data sources is available (PIR BPF, PIR LPF, temperature sensor)
                     uint8_t operation_mode,                                         //  three sensor operation modes are available
                     uint8_t window_time,                                            //  the pulse counter increment is performed during the selected time
                     uint8_t pulse_counter,                                          //  the amount of pulses above the threshold
                     uint8_t blind_time,                                             //  for avoid immediate re-triggering after a motion event detection
                     uint8_t threschold,                                             //  threshold voltage
                     uint16_t t_shd,                                                 //  data bit time (not less than 80 us)
                     uint16_t t_slt);                                                //  hold time for detector data latching

uint16_t get_pyd_data(uint16_t t_ds,                                                 //  data set-up time
                      uint16_t t_bit,                                                //  bit time
                      uint16_t t_ra);                                                //  abortion time

void bit_time(uint16_t cycles_quantity);

#ifdef MSP430

void burst_frequency(void);
void lsb_to_msb(uint8_t *x);

#endif










#endif	/*	__PYD1588_H	*/