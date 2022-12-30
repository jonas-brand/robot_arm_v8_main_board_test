/*
Programm for testing robot arm mainboard
*/

#include "Arduino.h"

struct gpio_t
{
    volatile uint8_t pinx;   //input
    volatile uint8_t ddrx;   //data direction
    volatile uint8_t portx;  //port
};

//function for testing connections within port
uint8_t self_test(gpio_t* self, const String names[8])
{
  //number of errors to be teturned out of function
  uint8_t errors = 0;

  //loop over every connection
  for(uint8_t i = 0; i < 8; i++)
  {
    //set all test pins to input and turn on pullup resistors
    self->ddrx = 0;
    self->portx = 0xFF;
    
    //set active testing pin as output and pull it low
    self->ddrx |= _BV(i);
    self->portx &= ~_BV(i);

    for(uint8_t j = i + 1; j < 8; j++)
    {
      if(!(self->pinx & _BV(j)))
      {
          Serial.print(names[i]);
          Serial.print(" - ");
          Serial.print(names[j]);
          Serial.println("\t------ERROR CONNECTION------");
          errors++;
      }      
    }
  }

  return errors;
}

//function for testing connections between ports
uint8_t test(gpio_t* port1, gpio_t* port2, const bool connections[8][8], const String names_1[8], const String names_2[8])
{
  //number of errors to be teturned out of function
  uint8_t errors;

  //loop over every connection
  for(uint8_t i = 0; i < 8; i++)
  {
    //set all test pins to input and turn on pullup resistors
    port1->ddrx = 0;
    port1->portx = 0xFF;
    port2->ddrx = 0;
    port2->portx = 0xFF;

    //set active testing pin as output and pull it low
    port1->ddrx |= _BV(i);
    port1->portx &= ~_BV(i);

    for(uint8_t j = 0; j < 8; j++)
    {
      //if there schould be a connection between pins
      if(connections[i][j])
      {
        Serial.print(names_1[i]);
        Serial.print(" - ");
        Serial.print(names_2[j]);

        if(!(port2->pinx & _BV(j)))
          Serial.println("\tok");
        else
        {
          Serial.println("\t------OPEN------");
          errors++;
        }
      }else
      //if there should not be a connection
      {
        if(!(port2->pinx & _BV(j)))
        {
          Serial.print(names_1[i]);
          Serial.print(" - ");
          Serial.print(names_2[j]);
          Serial.println("\t------ERROR CONNECTION------");
          errors++;
        }
      }
    }
  }

  return errors;
}

#define x true
#define _ false

#define PORT_POW (gpio_t*)&PINL
#define PORT_BUS_LIM (gpio_t*)&PINA
#define PORT_STP_DIR (gpio_t*)&PINC
#define PORT_CS_EN (gpio_t*)&PINK
#define PORT_TEST (gpio_t*)&PINF

//function for testing whole board
uint16_t board_test(void){
  //names
  const String pow[8] =       {"VCC", "VCCG", "-", "-", "VMG", "VM", "-", "-"};
  const String bus_lim[8] =   {"lim0", "lim1", "lim2", "sdo", "sck", "sdi", "-", "-"};
  const String stp_dir[8] =   {"stp0", "stp1", "stp2", "dir0", "dir1", "dir2", "-", "-"};
  const String cs_en[8] =     {"en0", "en1", "en2", "cs0", "cs1", "cs2", "-", "-"};
  const String drv_sig[8] =   {"drv:dir", "drv:stp", "-", "drv:sdo", "drv:csn", "drv:sck", "drv:sdi", "drv:en"};
  const String drv_pow[8] =   {"DRV:VCCG", "drv:VCC", "-", "-", "-", "-", "drv:VMG", "drv:VM"};
  const String out[8] =       {"VCCG", "lim", "-", "-", "-", "-", "-", "-"};

  //connections
  const bool con_pow[8][8] =       {{_, x, _, _, _, _, _, _},   //VCC
                                    {x, _, _, _, _, _, _, _},   //VCC_GND
                                    {_, _, _, _, _, _, _, _},
                                    {_, _, _, _, _, _, _, _},
                                    {_, _, _, _, _, _, x, _},   //VM_GND
                                    {_, _, _, _, _, _, _, x},   //VM
                                    {_, _, _, _, _, _, _, _},   
                                    {_, _, _, _, _, _, _, _}};  

  const bool con_bus_lim[8][8] =   {{_, _, _, _, _, _, _, _},   //lim0
                                    {_, _, _, _, _, _, _, _},   //lim1
                                    {_, _, _, _, _, _, _, _},   //lim2
                                    {_, _, _, x, _, _, _, _},   //sdo
                                    {_, _, _, _, _, x, _, _},   //sck
                                    {_, _, _, _, _, _, x, _},   //sdi
                                    {_, _, _, _, _, _, _, _},
                                    {_, _, _, _, _, _, _, _}};
  
  const bool con_stp_dir_0[8][8] = {{_, x, _, _, _, _, _, _},   //stp0
                                    {_, _, _, _, _, _, _, _},   //stp1
                                    {_, _, _, _, _, _, _, _},   //stp2
                                    {x, _, _, _, _, _, _, _},   //dir0
                                    {_, _, _, _, _, _, _, _},   //dir1
                                    {_, _, _, _, _, _, _, _},   //dir2
                                    {_, _, _, _, _, _, _, _},
                                    {_, _, _, _, _, _, _, _}};
  
  const bool con_stp_dir_1[8][8] = {{_, _, _, _, _, _, _, _},   //stp0
                                    {_, x, _, _, _, _, _, _},   //stp1
                                    {_, _, _, _, _, _, _, _},   //stp2
                                    {_, _, _, _, _, _, _, _},   //dir0
                                    {x, _, _, _, _, _, _, _},   //dir1
                                    {_, _, _, _, _, _, _, _},   //dir2
                                    {_, _, _, _, _, _, _, _},
                                    {_, _, _, _, _, _, _, _}};

  const bool con_stp_dir_2[8][8] = {{_, _, _, _, _, _, _, _},   //stp0
                                    {_, _, _, _, _, _, _, _},   //stp1
                                    {_, x, _, _, _, _, _, _},   //stp2
                                    {_, _, _, _, _, _, _, _},   //dir0
                                    {_, _, _, _, _, _, _, _},   //dir1
                                    {x, _, _, _, _, _, _, _},   //dir2
                                    {_, _, _, _, _, _, _, _},
                                    {_, _, _, _, _, _, _, _}};

  const bool con_cs_en_0[8][8] =   {{_, _, _, _, _, _, _, x},   //en0
                                    {_, _, _, _, _, _, _, _},   //en1
                                    {_, _, _, _, _, _, _, _},   //en2
                                    {_, _, _, _, x, _, _, _},   //cs0
                                    {_, _, _, _, _, _, _, _},   //cs1
                                    {_, _, _, _, _, _, _, _},   //cs2
                                    {_, _, _, _, _, _, _, _},
                                    {_, _, _, _, _, _, _, _}};

  const bool con_cs_en_1[8][8] =   {{_, _, _, _, _, _, _, _},   //en0
                                    {_, _, _, _, _, _, _, x},   //en1
                                    {_, _, _, _, _, _, _, _},   //en2
                                    {_, _, _, _, _, _, _, _},   //cs0
                                    {_, _, _, _, x, _, _, _},   //cs1
                                    {_, _, _, _, _, _, _, _},   //cs2
                                    {_, _, _, _, _, _, _, _},
                                    {_, _, _, _, _, _, _, _}};

  const bool con_cs_en_2[8][8] =   {{_, _, _, _, _, _, _, _},   //en0
                                    {_, _, _, _, _, _, _, _},   //en1
                                    {_, _, _, _, _, _, _, x},   //en2
                                    {_, _, _, _, _, _, _, _},   //cs0
                                    {_, _, _, _, _, _, _, _},   //cs1
                                    {_, _, _, _, x, _, _, _},   //cs2
                                    {_, _, _, _, _, _, _, _},
                                    {_, _, _, _, _, _, _, _}};

  const bool con_out_0[8][8] =     {{_, x, _, _, _, _, _, _},   //lim0
                                    {_, _, _, _, _, _, _, _},   //lim1
                                    {_, _, _, _, _, _, _, _},   //lim2
                                    {_, _, _, _, _, _, _, _},   //sdo
                                    {_, _, _, _, _, _, _, _},   //sck
                                    {_, _, _, _, _, _, _, _},   //sdi
                                    {_, _, _, _, _, _, _, _},
                                    {_, _, _, _, _, _, _, _}};

  const bool con_out_1[8][8] =     {{_, _, _, _, _, _, _, _},   //lim0
                                    {_, x, _, _, _, _, _, _},   //lim1
                                    {_, _, _, _, _, _, _, _},   //lim2
                                    {_, _, _, _, _, _, _, _},   //sdo
                                    {_, _, _, _, _, _, _, _},   //sck
                                    {_, _, _, _, _, _, _, _},   //sdi
                                    {_, _, _, _, _, _, _, _},
                                    {_, _, _, _, _, _, _, _}};

  const bool con_out_2[8][8] =     {{_, _, _, _, _, _, _, _},   //lim0
                                    {_, _, _, _, _, _, _, _},   //lim1
                                    {_, x, _, _, _, _, _, _},   //lim2
                                    {_, _, _, _, _, _, _, _},   //sdo
                                    {_, _, _, _, _, _, _, _},   //sck
                                    {_, _, _, _, _, _, _, _},   //sdi
                                    {_, _, _, _, _, _, _, _},
                                    {_, _, _, _, _, _, _, _}};

  const bool con_out[8][8] =       {{_, _, _, _, _, _, _, _},   //VCC
                                    {x, _, _, _, _, _, _, _},   //VCC_GND
                                    {_, _, _, _, _, _, _, _},
                                    {_, _, _, _, _, _, _, _},
                                    {_, _, _, _, _, _, _, _},   //VM_GND
                                    {_, _, _, _, _, _, _, _},   //VM
                                    {_, _, _, _, _, _, _, _},   
                                    {_, _, _, _, _, _, _, _}};

  const bool con_none[8][8] =      {{_}, {_}, {_}, {_}, {_}, {_}, {_}, {_}};

  //total number of errors
  uint16_t errors = 0;

  //tests
  Serial.println("====================connect to board====================");
  while(! Serial.available());
  while(Serial.available())
    Serial.read();

  errors += self_test(PORT_POW, pow);
  errors += self_test(PORT_BUS_LIM, bus_lim);
  errors += self_test(PORT_STP_DIR, stp_dir);
  errors += self_test(PORT_CS_EN, cs_en);

  errors += test(PORT_POW, PORT_BUS_LIM, con_none, pow, bus_lim);
  errors += test(PORT_POW, PORT_STP_DIR, con_none, pow, stp_dir);
  errors += test(PORT_POW, PORT_CS_EN, con_none, pow, cs_en);
  errors += test(PORT_BUS_LIM, PORT_STP_DIR, con_none, bus_lim, stp_dir);
  errors += test(PORT_BUS_LIM, PORT_CS_EN, con_none, bus_lim, cs_en);
  errors += test(PORT_STP_DIR, PORT_CS_EN, con_none, stp_dir, cs_en);

  if(errors == 0)
    Serial.println("ok");

  Serial.println("\n====================driver 0 signal====================");
  while(! Serial.available());
  while(Serial.available())
    Serial.read();

  errors += test(PORT_BUS_LIM, PORT_TEST, con_bus_lim, bus_lim, drv_sig);
  errors += test(PORT_STP_DIR, PORT_TEST, con_stp_dir_0, stp_dir, drv_sig);
  errors += test(PORT_CS_EN, PORT_TEST, con_cs_en_0, cs_en, drv_sig);

  Serial.println("\n====================driver 1 signal====================");
  while(! Serial.available());
  while(Serial.available())
    Serial.read();

  errors += test(PORT_BUS_LIM, PORT_TEST, con_bus_lim, bus_lim, drv_sig);
  errors += test(PORT_STP_DIR, PORT_TEST, con_stp_dir_1, stp_dir, drv_sig);
  errors += test(PORT_CS_EN, PORT_TEST, con_cs_en_1, cs_en, drv_sig);

  Serial.println("\n====================driver 2 signal====================");
  while(! Serial.available());
  while(Serial.available())
    Serial.read();

  errors += test(PORT_BUS_LIM, PORT_TEST, con_bus_lim, bus_lim, drv_sig);
  errors += test(PORT_STP_DIR, PORT_TEST, con_stp_dir_2, stp_dir, drv_sig);
  errors += test(PORT_CS_EN, PORT_TEST, con_cs_en_2, cs_en, drv_sig);

  Serial.println("\n====================driver 0 power====================");
  while(! Serial.available());
  while(Serial.available())
    Serial.read();
  
  errors += test(PORT_POW, PORT_TEST, con_pow, pow, drv_pow);

  Serial.println("\n====================driver 1 power====================");
  while(! Serial.available());
  while(Serial.available())
    Serial.read();
  
  errors += test(PORT_POW, PORT_TEST, con_pow, pow, drv_pow);

  Serial.println("\n====================driver 2 power====================");
  while(! Serial.available());
  while(Serial.available())
    Serial.read();
  
  errors += test(PORT_POW, PORT_TEST, con_pow, pow, drv_pow);

  Serial.println("\n====================driver 0 output====================");
  while(! Serial.available());
  while(Serial.available())
    Serial.read();

  errors += test(PORT_POW, PORT_TEST, con_out, pow, out);
  errors += test(PORT_BUS_LIM, PORT_TEST, con_out_0, bus_lim, out);

  Serial.println("\n====================driver 1 output====================");
  while(! Serial.available());
  while(Serial.available())
    Serial.read();

  errors += test(PORT_POW, PORT_TEST, con_out, pow, out);
  errors += test(PORT_BUS_LIM, PORT_TEST, con_out_1, bus_lim, out);

  Serial.println("\n====================driver 2 output====================");
  while(! Serial.available());
  while(Serial.available())
    Serial.read();

  errors += test(PORT_POW, PORT_TEST, con_out, pow, out);
  errors += test(PORT_BUS_LIM, PORT_TEST, con_out_2, bus_lim, out);

  return errors;
}

void setup()
{
  Serial.begin(9600);
  Serial.println("testing:");
  uint16_t errors = board_test();
  Serial.println();
  Serial.print(errors);
  Serial.println(" errors");

  if(errors == 0)
    Serial.println("\n================================PASSED================================");
  else
    Serial.println("\n================================FAILED================================");
}

void loop()
{

}