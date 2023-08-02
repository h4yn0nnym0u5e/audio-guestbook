#include <Arduino.h>

class PulseDial 
{
    enum {idle,windup,high,low} state; 
    elapsedMillis debounce;
    int SW1,SW3;
    const int debounce_time = 4;
  public: 
    PulseDial(int s1, int s3) : SW1(s1), SW3(s3) 
    {
      pinMode(s1,INPUT_PULLUP);
      pinMode(s3,INPUT_PULLUP);
    };
    int count; 
    bool newNumber; 
    void update(void);
    bool dialling(void) {return state != idle; }
};
