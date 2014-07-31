// inslude the SPI library:
#include <SPI.h>

// setup pins
int nReset = 8;
int nCS = 9;

void setup(){
  //set pin I/O direction
  pinMode (nReset, OUTPUT);
  pinMode (nCS, OUTPUT);
  
  //put the device in reset not chip selected
  digitalWrite(nReset,LOW);
  digitalWrite(nCS,HIGH);
  
  //initialize SPI
  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
  //start a serial port for debugging
  Serial.begin(115200);
  delay(1000);
  Serial.println("startup complete");
}

void loop(){
  unsigned int read_data;
  
  //enable ADNS
  digitalWrite(nReset,HIGH);
  delay(1000);
  
//  pixel_grab();
//  delay(1000);  

//  while(1){
//    delay(100);
//    Serial.println(String(squal()));
//  }

  while(1){
    unsigned int motion = 0;
    //check for movement
    //read motion register, 0x02
    motion = ADNS_read(0x02);
    if(motion > 127){
      //there has been a motion!
      //print the x and y movements
      
      /* 
          The original code provided by the author of this library included this line
          for displaying results, but casting the results with String(...) mishandles
          the twos complement results for positive or negative displacement in x and y:
          
            Serial.println("X:" + String(ADNS_read(0x03)) + " Y:" + String(ADNS_read(0x04)));
      */
      
      /*
          So I added this block of code which gets it right
      */
              
      //int8_t dx = convert_from_unsigned(dx_raw);
      
      uint8_t dy_raw = ADNS_read(0x04);
      int8_t dy = int8_t(dy_raw);
      //int8_t dy = convert_from_signed(dy_raw);
      
      char buf[80];
      sprintf(buf, "X: %d\tY: %d", dx, dy);
      Serial.println(buf);
    }
    
  }
}


/*
  conversion function to change unsigned bytes into signed int8_t's
  based on two complement format in the unsigned byte.
  See wikipedio for explantion of 2's complement conversion formula
    http://en.wikipedia.org/wiki/Two%27s_complement#Converting_from_two.27s_complement_representation
*/
int8_t convert_from_unsigned(uint8_t input_byte) {
  int8_t result = input_byte;
  
  if(bitRead(result, 7)) {  
     result = -1 * (~result + 1);     
  }
  
  return result;
}

unsigned int squal(){
  //squal is address 0x05
  return(ADNS_read(0x05));
}

void pixel_grab(){
  //address = 0x0b
  int xcount=0;
  int count=0;
  
  //reset the pixel grab counter
  ADNS_write(0x0B,0x00);

  Serial.println("");  
  for (count=0;count < 225; count++){
    if (count%15 == 14){
      Serial.println(String(ADNS_read(0x0B)));
    }
    else{
      Serial.print(String(ADNS_read(0x0B)) + ",");
    }
  }
  Serial.println("");
}

void ADNS_write(unsigned int address, unsigned int data){
  // take the CS pin low to select the chip:
  digitalWrite(nCS,LOW);
  //  send in the address and value via SPI:
  SPI.transfer(address);
  SPI.transfer(data);
  // take the SS pin high to de-select the chip:
  digitalWrite(nCS,HIGH); 
}

/* Zac changed the type to guarantee an 8 bit representation */
uint8_t ADNS_read(unsigned int address){
  uint8_t data;
  // take the CS pin low to select the chip:
  digitalWrite(nCS,LOW);
  //  send in the address and value via SPI:
  SPI.transfer(address);

  pinMode(MOSI, INPUT);
  data = SPI.transfer(0x00);
  pinMode(MOSI, OUTPUT);
  // take the SS pin high to de-select the chip:
  digitalWrite(nCS,HIGH); 
  return(data);
}
