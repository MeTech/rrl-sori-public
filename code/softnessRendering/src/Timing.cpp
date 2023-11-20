#include "Arduino.h"

class Timer
{
  private:
    unsigned long st_time;              // Time when the program started
    unsigned long Ct1_prev;             // Time in the previous iteration
    unsigned long Now;                  // Global time
    unsigned long CT1_Ts;               // Sampling time in miliseconds
    unsigned long Ct1_dt;               // Measured sampling time in miliseconds
  
  public:
    Timer(void) 
    {
     this->st_time=0;
     this->Ct1_prev=0;
     this->Now=0;
     this->CT1_Ts=0;
     this->Ct1_dt=0;
    }

    void initialize(unsigned long CT1_Ts)
    {
      this->st_time=millis();
      this->Ct1_prev=0;
      this->Now=0;
      this->CT1_Ts=CT1_Ts;
      this->Ct1_dt=0;
    }

    void reset(void)
    {
      this->st_time=millis();
      this->Ct1_prev=0;
      this->Now=0;
      this->Ct1_dt=0;
    }

    boolean Tick(void)
    {
      this->Now=millis()-(this->st_time);
      if (this->Now-this->Ct1_prev > this->CT1_Ts)
      {
        this->Ct1_dt=this->Now-this->Ct1_prev;
        this->Ct1_prev=this->Now;
        return true;
      }
      else
      {
        return false;
      }
    }
    
    float T_Now(void)
    {
      return (float) this->Now/1000;
    }
    
    float dt(void)
    {
      return (float) this->Ct1_dt/1000;
    }
        
};
