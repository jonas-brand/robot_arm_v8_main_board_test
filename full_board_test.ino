/*
Programm for testing robot arm mainboard
*/

//first test port
#define TEST_PORT_1 PORTL
#define TEST_DDR_1  DDRL
#define TEST_PIN_1  PINL

//second test port
#define TEST_PORT_2 PORTA
#define TEST_DDR_2  DDRA
#define TEST_PIN_2  PINA

//function for testing connections within port
uint8_t self_test_1(const String names[8])
{
  //number of errors to be teturned out of function
  uint8_t errors = 0;

  //loop over every connection
  for(uint8_t i = 0; i < 8; i++)
  {
    //set all test pins to input and turn on pullup resistors
    TEST_DDR_1 = 0;
    TEST_PORT_1 = 0xFF;
    
    //set active testing pin as output and pull it low
    TEST_DDR_1 |= _BV(i);
    TEST_PORT_1 &= ~_BV(i);

    for(uint8_t j = i + 1; j < 8; j++)
    {
      if(!(TEST_PIN_1 & _BV(j)))
      {
         Serial.print("------ERROR CONNECTION------\t");
          Serial.print(names[i]);
          Serial.print(" - ");
          Serial.println(names[j]);
          errors++;
      }      
    }
  }

  return errors;
}

//function for testing connections within port
uint8_t self_test_2(const String names[8])
{
  //number of errors to be teturned out of function
  uint8_t errors = 0;

  //loop over every connection
  for(uint8_t i = 0; i < 8; i++)
  {
    //set all test pins to input and turn on pullup resistors
    TEST_DDR_2 = 0;
    TEST_PORT_2 = 0xFF;
    
    //set active testing pin as output and pull it low
    TEST_DDR_2 |= _BV(i);
    TEST_PORT_2 &= ~_BV(i);

    for(uint8_t j = i + 1; j < 8; j++)
    {
      if(!(TEST_PIN_2 & _BV(j)))
      {
         Serial.print("------ERROR CONNECTION------\t");
          Serial.print(names[i]);
          Serial.print(" - ");
          Serial.println(names[j]);
          errors++;
      }      
    }
  }

  return errors;
}

//function for testing connections between ports
uint8_t test(const bool connections[8][8], const String names_1[8], const String names_2[8])
{
  //number of errors to be teturned out of function
  uint8_t errors = self_test_1(names_1) + self_test_2(names_2);

  //loop over every connection
  for(uint8_t i = 0; i < 8; i++)
  {
    //set all test pins to input and turn on pullup resistors
    TEST_DDR_1 = 0;
    TEST_PORT_1 = 0xFF;
    TEST_DDR_2 = 0;
    TEST_PORT_2 = 0xFF;

    //set active testing pin as output and pull it low
    TEST_DDR_1 |= _BV(i);
    TEST_PORT_1 &= ~_BV(i);

    for(uint8_t j = 0; j < 8; j++)
    {
      //if there schould be a connection between pins
      if(connections[i][j])
      {
        Serial.print(names_1[i]);
        Serial.print(" - ");
        Serial.print(names_2[j]);

        if(!(TEST_PIN_2 & _BV(j)))
          Serial.println("\tok");
        else
        {
          Serial.println("\t------OPEN------");
          errors++;
        }
      }else
      //if there should not be a connection
      {
        if(!(TEST_PIN_2 & _BV(j)))
        {
          Serial.print("------ERROR CONNECTION------\t");
          Serial.print(names_1[i]);
          Serial.print(" - ");
          Serial.println(names_2[j]);
          errors++;
        }
      }
    }
  }

  return errors;
}

//function for testing whole board
uint16_t board_test(void){
  //names
  const String bus_names[8] = {"MISO", "SCK", "MOSI", "_", "_", "_", "_", "_"};
  const String cs_names[8] = {"cs0", "cs1", "cs2", "cs3", "cs4", "cs5" , "_", "_"};
  const String en_names[8] = {"en0", "en1", "en2", "en3", "en4", "en5" , "_", "_"};
  const String step_names[8] = {"stp0", "stp1", "stp2", "stp3", "stp4", "stp5" , "_", "_"};
  const String dir_names[8] = {"dir0", "dir1", "dir2", "dir3", "dir4", "dir5" , "_", "_"};
  const String llim_names[8] = {"llim0", "llim1", "llim2", "llim3", "llim4", "llim5" , "_", "_"};
  const String ulim_names[8] = {"ulim0", "ulim1", "ulim2", "ulim3", "ulim4", "ulim5" , "_", "_"};
  const String vcc_names[8] = {"vcc", "gnd", "_", "_", "_", "_", "_", "_"};
  const String drv_signal_names[8] = {"dir", "stp", "_", "sdo", "csn", "sck", "sdi", "en"};
  const String drv_power_names[8] = {"gnd", "vcc", "_", "_", "_", "_", "gnd", "vm"};
  const String mot_plug_names[8] = {"gnd", "ulim", "_", "_", "_", "_", "gnd", "llim"};

  //connections
  //                                                       sdo           sck    sdi
  const bool bus_drv_signal[8][8] = {{false, false, false, true,  false, false, false, false}, //MISO
                                     {false, false, false, false, false, true,  false, false}, //SCK
                                     {false, false, false, false, false, false, true,  false}, //MOSI
                                     {false, false, false, false, false, false, false, false},
                                     {false, false, false, false, false, false, false, false},
                                     {false, false, false, false, false, false, false, false},
                                     {false, false, false, false, false, false, false, false},
                                     {false, false, false, false, false, false, false, false}};

  //total number of errors
  uint16_t errors = 0;

  //tests
  Serial.println("test port 1 -> bus, test port 2 -> driver 0 signal");
  while(! Serial.available());
  while(Serial.available())
    Serial.read();

  errors += test(bus_drv_signal, bus_names, drv_signal_names);

  return errors;
}

void setup()
{
  Serial.begin(9600);
  Serial.println("testing:");
  uint16_t errors = board_test();
  Serial.print(errors);
  Serial.print(" errors");
}

void loop()
{

}
