/*
 * Stuff relating specifically to build using BT 8746 phone
 * 
 * Wiring gutted and re-made:
 * Dial grey / pink  -> T16  (ground; switches common)
 * Dial blue / brown -> T15  (SW1+SW2; NO; closed during wind-up and -down; SW1 closes first)
 * Dial orange       -> T8   (SW3; NC; pulse-opens during wind-down for loop-disconnect dialling)
 * 
 * Hook switch common -> T6  
 * Hook switch NC     -> T5
 * Hook switch NO     -> T2
 * 
 * T11-15 are (originally) floating
 * T16-19 are jumpered
 */
#include "phone8746.h"
void PulseDial::update(void)
{
  switch (state)
  {
    case idle:
      if (0 == digitalRead(SW1) // wind-up started
       && debounce >= debounce_time)
      {
        count = 0;
        state = windup;
        newNumber = false;
        seenPulse = false;
      }
      break;

    case high:
      if (debounce >= debounce_time) // pulse end
      {
        if (0 == digitalRead(SW3))
        {
          state = low;
          debounce = 0;
        }
      }
      break;

    case windup:      
    case low:
      if (debounce >= debounce_time)
      {
        if (1 == digitalRead(SW3)) // pulse start: cout it
        {
          state = high;
          debounce = 0;
          count++;
          seenPulse = true;
        }
      }
      if (1 == digitalRead(SW1)) // wind-down complete
      {
        state = idle; 
        newNumber = seenPulse;
        debounce = 0;

        // 10 pulses means 0 was dialled
        if (10 == count)
          count = 0;
      }
      break;
  }
}
