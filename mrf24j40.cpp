#include <Arduino.h>
#include <mrf24j40.h>

void MRFClass::setSS(void) {
  PORTB &= ~_BV(_pin_cs);
}

void MRFClass::resetSS(void) {
  PORTB |=  _BV(_pin_cs);
}

MRFClass::MRFClass(int pin_cs, int pin_int, int PC){
  if(pin_cs == 9) {
	_portb_cs = 0b00000010;
	_pin_cs = 1;
  } else if(pin_cs == 10) {
    _portb_cs = 0b00000100;
	_pin_cs = 2;
  }
  _macBSN = random(sizeof(unsigned long));
  _seq_num =  1;

  pinMode(pin_cs,OUTPUT);
  pinMode(pin_int,INPUT);

  resetSS();

   _PC=PC;
}

byte MRFClass::read_short(byte address) {
  setSS();
  // 0 top for short addressing, 0 bottom for read
  SPI.transfer(address<<1 & 0b01111110);
  byte ret = SPI.transfer(0x0);
  resetSS();
  return ret;
}

byte MRFClass::read_long(word address) {
  setSS();
  byte ahigh = address >> 3;
  byte alow = address << 5;
  SPI.transfer(0x80 | ahigh); // high bit for long
  SPI.transfer(alow);
  byte ret = SPI.transfer(0);
  resetSS();
  return ret;
}

void MRFClass::write_short(byte address, byte data) {
    // 0 for top address, 1 bottom for write
	setSS();
    SPI.transfer((address<<1 & 0b01111110) | 0x01);
    SPI.transfer(data);
	resetSS();
}

void MRFClass::write_long(word address, byte data) {
  setSS();
  byte ahigh = address >> 3;
  byte alow = address << 5;
  SPI.transfer(0x80 | ahigh); // high bit for long
  SPI.transfer(alow | 0x10); // last bit for write
  SPI.transfer(data);
  resetSS();
}

word MRFClass::read_pan(void) {
  byte panh = read_short(MRF_PANIDH);
  return panh << 8 | read_short(MRF_PANIDL);
}

void MRFClass::write_pan(word panid) {
  mrf_panid[0] = panid & 0xff;
  mrf_panid[1] = panid >> 8;

  write_short(MRF_PANIDL, mrf_panid[0]);
  write_short(MRF_PANIDH, mrf_panid[1]);

  Serial.print("panid: ");
  Serial.print(read_short(MRF_PANIDH), HEX);
  Serial.println(read_short(MRF_PANIDL), HEX);
}

void MRFClass::write_addr16(word address16) {
  write_short(MRF_SADRH, address16 >> 8);
  write_short(MRF_SADRL, address16 & 0xff);
  
  Serial.print("addr16: ");
  Serial.print(read_short(MRF_SADRH), HEX);
  Serial.println(read_short(MRF_SADRL), HEX);
}

word MRFClass::read_addr16(void) {
  byte a16h = read_short(MRF_SADRH);
  return a16h << 8 | read_short(MRF_SADRL);
}

void MRFClass::set_addr64(void) {
  byte temp;
  //load EEPROM into local variable
  
  mrf_mac[0]  = EEPROM.read(EEPROM_MADR7);
  mrf_mac[1]  = EEPROM.read(EEPROM_MADR6);
  mrf_mac[2]  = EEPROM.read(EEPROM_MADR5);
  mrf_mac[3]  = EEPROM.read(EEPROM_MADR4);
  mrf_mac[4]  = EEPROM.read(EEPROM_MADR3);
  mrf_mac[5]  = EEPROM.read(EEPROM_MADR2);
  mrf_mac[6]  = EEPROM.read(EEPROM_MADR1);
  mrf_mac[7]  = EEPROM.read(EEPROM_MADR0);

  //write address to mrf transiever
  write_short(MRF_EADR0,mrf_mac[0]);
  write_short(MRF_EADR1,mrf_mac[1]);
  write_short(MRF_EADR2,mrf_mac[2]);
  write_short(MRF_EADR3,mrf_mac[3]);
  write_short(MRF_EADR4,mrf_mac[4]);
  write_short(MRF_EADR5,mrf_mac[5]);
  write_short(MRF_EADR6,mrf_mac[6]);
  write_short(MRF_EADR7,mrf_mac[7]);

  //print the address so we know the mrf got it
  Serial.print("addr64: ");
  temp = read_short(MRF_EADR7);
  if (temp < 16) Serial.print("0");
  Serial.print(temp, HEX);
  temp = read_short(MRF_EADR6);
  if (temp < 16) Serial.print("0");
  Serial.print(temp, HEX);
  temp = read_short(MRF_EADR5);
  if (temp < 16) Serial.print("0");
  Serial.print(temp, HEX);
  temp = read_short(MRF_EADR4);
  if (temp < 16) Serial.print("0");
  Serial.print(temp, HEX);
  temp = read_short(MRF_EADR3);
  if (temp < 16) Serial.print("0");
  Serial.print(temp, HEX);
  temp = read_short(MRF_EADR2);
  if (temp < 16) Serial.print("0");
  Serial.print(temp, HEX);
  temp = read_short(MRF_EADR1);
  if (temp < 16) Serial.print("0");
  Serial.print(temp, HEX);
  temp = read_short(MRF_EADR0);
  if (temp < 16) Serial.print("0");
  Serial.println(temp, HEX);
  
  //Serial.print(read_short(MRF_EADR7), HEX);
  //Serial.print(read_short(MRF_EADR6), HEX);
  //Serial.print(read_short(MRF_EADR5), HEX);
  //Serial.print(read_short(MRF_EADR4), HEX);
  //Serial.print(read_short(MRF_EADR3), HEX);
  //Serial.print(read_short(MRF_EADR2), HEX);
  //Serial.print(read_short(MRF_EADR1), HEX);
  //Serial.println(read_short(MRF_EADR0), HEX);
}

void MRFClass::set_channel(byte channel) {
  write_long(MRF_RFCON0, (((channel - 11) << 4) | 0x03));
}

void MRFClass::set_AES_key(void) {
  uint16_t i;
  uint16_t j;

  i=0x280;  //0x280 to 0x28F
  j=0x2B0; //0x2B0 to 0x2BF
  
  write_long(i++,MRF_AES0);
  write_long(i++,MRF_AES1);
  write_long(i++,MRF_AES2);
  write_long(i++,MRF_AES3);
  write_long(i++,MRF_AES4);
  write_long(i++,MRF_AES5);
  write_long(i++,MRF_AES6);
  write_long(i++,MRF_AES7);
  write_long(i++,MRF_AES8);
  write_long(i++,MRF_AES9);
  write_long(i++,MRF_AES10);
  write_long(i++,MRF_AES11);
  write_long(i++,MRF_AES12);
  write_long(i++,MRF_AES13);
  write_long(i++,MRF_AES14);
  write_long(i++,MRF_AES15);
 
  write_long(j++,MRF_AES0);
  write_long(j++,MRF_AES1);
  write_long(j++,MRF_AES2);
  write_long(j++,MRF_AES3);
  write_long(j++,MRF_AES4);
  write_long(j++,MRF_AES5);
  write_long(j++,MRF_AES6);
  write_long(j++,MRF_AES7);
  write_long(j++,MRF_AES8);
  write_long(j++,MRF_AES9);
  write_long(j++,MRF_AES10);
  write_long(j++,MRF_AES11);
  write_long(j++,MRF_AES12);
  write_long(j++,MRF_AES13);
  write_long(j++,MRF_AES14);
  write_long(j++,MRF_AES15);
}

void MRFClass::reset(void) {
  write_short(MRF_SOFTRST, 0x07); // Perform a software reset
}

void MRFClass::EnergyDetect(void) {
  byte rssi_stat;
  uint8_t i;
  
  write_long(MRF_TESTMODE, 0x08);	// Disable automatic switch on PA/LNA
  write_short(MRF_TRISGPIO, 0x0F);	// Set GPIO direction to OUTPUT (control PA/LNA)
  write_short(MRF_GPIO, 0x0C);		// Enable LNA, disable
 
  for(i=0;i<16;i++) {
    set_channel(i); //Select channel
	
	write_short(MRF_BBREG6, 0x80);	// set RSSIMODE1 to initiate RSSI measurement
	
    rssi_stat = read_short(MRF_BBREG6);
	while((rssi_stat & 0x01) != 0x01) {
      rssi_stat = read_short(MRF_BBREG6);
    }
 
    rssi_stat = read_long(MRF_RSSI);
	
	Serial.print("Chan: ");
	Serial.print(i, HEX);
	Serial.print(" RSSI: ");
	Serial.println(rssi_stat, HEX);
  }
 
  write_short(MRF_BBREG6, 0x80);	// enable RSSI on received packets again after energy scan is finished
  write_short(MRF_GPIO, 0x00);
  write_short(MRF_TRISGPIO, 0x00);	// Set GPIO direction to INPUT
  write_long(MRF_TESTMODE, 0x0F);	// setup for automatic PA/LNA control
}

void MRFClass::init(void){
  write_short(MRF_GPIO, 0x00);
  write_short(MRF_TRISGPIO, 0x00);
  write_long(MRF_TESTMODE, 0x0F);
  
  write_short(MRF_PACON0, 0x29);
  write_short(MRF_PACON1, 0x02);
  write_short(MRF_PACON2, 0x98); // Initialize FIFOEN = 1 and TXONTS = 0x6.
  write_short(MRF_TXSTBL, 0x95); // Initialize RFSTBL = 0x9.

  write_long(MRF_RFCON0, 0x03); // Initialize RFOPT = 0x03.
  write_long(MRF_RFCON1, 0x02); // Initialize VCOOPT = 0x02.
  write_long(MRF_RFCON2, 0x80); // Enable PLL (PLLEN = 1).
  write_long(MRF_RFCON6, 0x90); // Initialize TXFIL = 1 and 20MRECVR = 1.
  write_long(MRF_RFCON7, 0x80); // Initialize SLPCLKSEL = 0x2 (100 kHz Internal oscillator).
  write_long(MRF_RFCON8, 0x10); // Initialize RFVCO = 1.
  write_long(MRF_SLPCON1, 0x21); // Initialize CLKOUTEN = 1 and SLPCLKDIV = 0x01.

  // Configuration for nonbeacon-enabled devices (see Section 3.8 Beacon-Enabled and
  // Nonbeacon-Enabled Networks)
  write_short(MRF_TXMCR, 0x1C);  //Clear Slotted mode
  if(_PC == 1) {
    EnergyDetect();
    write_short(MRF_RXMCR, 0x08); //Set as PC
    //write_short(MRF_RXMCR, 0x09); //PC, PROMI
    write_short(MRF_ORDER, 0xFF); //BO=15 and SO=15
  } else {
    write_short(MRF_RXMCR, 0x00);
  }

  //Security
  set_AES_key(); //install the tx and rx security key
  //write_short(MRF_SECCON0,0x12); //enable AES-CCM-128 on the TXFIFO and RXFIFO
  //write_short(MRF_SECCON0,0x09); //AES-CTR
  write_short(MRF_SECCON0,0); //NONE

  write_short(MRF_BBREG2, 0x80); // Set CCA mode to ED
  write_short(MRF_CCAEDTH, 0x60); // Set CCA ED threshold.
  write_short(MRF_BBREG6, 0x40); // Set appended RSSI value to RXFIFO.

  // Initialize interrupts
  write_long(MRF_SLPCON0, 0x01); //Interrupt on falling edge and disable sleep clock
  write_short(MRF_INTCON, 0xE6); //Enable SEC, RX, and TX Interrupts

  set_channel(7);
  write_long(MRF_RFCON3, 0x40); //Select TX Power
  write_short(MRF_RFCTL, 0x04); //Reset RF state machine.
  write_short(MRF_RFCTL, 0x00);

  delay(1); //delay at least 192usec

  set_addr64();

  if(_PC) {
    memset(_parent_addr, 0, 8);
    write_pan(0xCBA1);
    write_addr16(0xFFFE);
    client_cnt = 0;
    _join_stat = 4;
    _hops = 0;
    _beacon_timer = millis();
  }
  else {
    init_node();
  }

  _tx_busy = 0;
  _tx_count = 0;
  _rx_count = 0;
  cmd_count = 0;
  udp_packet_cnt = 0;
}

void MRFClass::init_node(void) {
  memset(_parent_addr, 0, 8);
  write_pan(0xFFFF);
  write_addr16(0xFFFF);
  _hops = 0xFF;
  _join_stat = 1;
}

void MRFClass::tx_ready(void) {
  byte header;
  byte txncon;
  
  // ??? <5> | sec en <1> | ack req <1> | tx rdy <1>
  txncon = 0x01;
  header = read_long(0x02);
  
  if((header & 0x20) == 0x20) {
    txncon = txncon | 0x02;
  }
  
  if((header & 0x04) == 0x04) {
    txncon = txncon | 0x04;
  }
  
  write_short(MRF_TXNCON,txncon);
}

void MRFClass::tx_beacon_req(void) {
  int i;
  i=0;

  write_long(i++,7); //header length
  write_long(i++,8); //frame length

  // 0 | PANID comp | ack| frame pending | security | frame type <3>
  //write_long(i++,0b01001011);
  write_long(i++,0b01000011);

  // src mode <2> | frame ver <2> | dest mode <2> | 00
  write_long(i++,0b00001000);

  write_long(i++,_seq_num); //sequence number
  write_long(i++,0xFF); //broadcast PANID low
  write_long(i++,0xFF); //broadcast PANID high
  write_long(i++,0xFF); //broadcast address low
  write_long(i++,0xFF); //broadcast address high
  write_long(i++,0x07); //MAC Command Frame (beacon request)

  tx_ready();

  //Serial.println("TX BEACON REQ");
}

void MRFClass::tx_beacon(void) {
  int i;
  int j;

  i=0;
  write_long(i++,13); //header length
  write_long(i++,16); //frame length

  // 0 | PANID comp | ack| frame pending | security | frame type <3>
  //write_long(i++,0b00001000);
  write_long(i++,0b00000000);

  // src mode <2> | frame ver <2> | dest mode <2> | 00
  write_long(i++,0b11000000);

  write_long(i++,_seq_num); //sequence number

  //write panid
  for(j=0;j<2;j++) {
    write_long(i++,mrf_panid[j]);
  }

  //write src addr
  for(j=0;j<8;j++) {
    write_long(i++,mrf_mac[j]);
  }

  // superframe order <4> | beacon order <4>
  write_long(i++,0b00000000); //this field is ignored
  // assoc permit | PC | 0 | bat life ext | final CAP <4>
  write_long(i++,0b11000000); //most of this is ignored
  write_long(i++,0x00); //Hops to PC

  tx_ready();

  //Serial.println("TX BEACON");
  _beacon_timer = millis();
}

void MRFClass::tx_assoc_req(void) {
  int i;
  int j;

  i=0;
  write_long(i++,23); //header length
  write_long(i++,25); //frame lengt

  // 0 | PANID comp | ack| frame pending | security | frame type <3>
  //write_long(i++,0b00101011);
  write_long(i++,0b00100011);

  // src mode <2> | frame ver <2> | dest mode <2> | 00
  write_long(i++,0b11001100);

  write_long(i++,_seq_num); //sequence number

  //write panid
  for(j=0;j<2;j++) {
    write_long(i++,mrf_panid[j]);
  }

  //write dest addr
  for(j=0;j<8;j++) {
	  write_long(i++,_parent_addr[j]);
  }

  write_long(i++,0xFF); //src PANID low (broadcast)
  write_long(i++,0xFF); //src PANID high (broadcast)

  //write src addr
  for(j=0;j<8;j++) {
    write_long(i++,mrf_mac[j]);
  }
  write_long(i++,0x01); //MAC Command Frame (association request)

  // alloc addr | sec cap | 00 | rx idle | pwr src | dev type | PC
  write_long(i++,0b00000000); //ignored

  tx_ready();

  //Serial.println("TX ASSOC REQ");
}

void MRFClass::tx_assoc_resp(DeviceAddress dest_addr) {
  int i;
  int j;

  i=0;
  write_long(i++,23); //header length
  write_long(i++,27); //frame length

  // 0 | PANID comp | ack| frame pending | security | frame type <3>
  //write_long(i++,0b00101011);
  write_long(i++,0b00100011);

  // src mode <2> | frame ver <2> | dest mode <2> | 00
  write_long(i++,0b11001100);

  write_long(i++,_seq_num);  // sequence number 1
  write_long(i++,0xFF); //dest PANID low (broadcast)
  write_long(i++,0xFF); //dest PANID high (broadcast)

  for(j=0;j<8;j++) {
    write_long(i++,dest_addr[j]);
  }

  //write panid
  for(j=0;j<2;j++) {
    write_long(i++,mrf_panid[j]);
  }

  //write src addr
  for(j=0;j<8;j++) {
    write_long(i++,mrf_mac[j]);
  }

  write_long(i++,0x02); //MAC Command Frame (association request)
  write_long(i++,0xFE); //short addr low
  write_long(i++,0xFF); //short addr high
  write_long(i++,0x00); //assoc status (0x00 is success)

  tx_ready();

  Serial.print("Join: ");
  printAddress(dest_addr);
  //memcpy(client_list[client_cnt], dest_addr, 8);
  setAddress(&client_list[client_cnt].addr[0], &dest_addr);
  client_list[client_cnt].time=millis();
  client_cnt++;

  //Serial.println("TX ASSOC RESP");
}

void MRFClass::tx_data_cmd(DeviceAddress dest_addr, byte data_cmd) {
  int i;
  int j;

  i=0;
  write_long(i++,21); //header length
  write_long(i++,22); //frame length

  // 0 | PANID comp | ack| frame pending | security | frame type <3>
  //write_long(i++,0b01101001);
  write_long(i++,0b01100001);
  // src mode <2> | frame ver <2> | dest mode <2> | 00
  write_long(i++,0b11001100);

  write_long(i++,_seq_num); //sequence number

  //write panid
  for(j=0;j<2;j++) {
    write_long(i++,mrf_panid[j]);
  }

  //write dest addr
  for(j=0;j<8;j++){
	  write_long(i++,dest_addr[j]);
  }

  //write src addr
  for(j=0;j<8;j++) {
    write_long(i++,mrf_mac[j]);
  }
  //end of header (21 bytes)

  write_long(i++,data_cmd); //cmd id

  tx_ready();

  //Serial.print("TX DATA CMD: ");
  //Serial.println(data_cmd, HEX);
}

void MRFClass::tx_data_queue(uint8_t len) {
  int i;
  int j;

  i=0;
  write_long(i++,21); //header length
  write_long(i++,22+len); //frame length

  // 0 | PANID comp | ack| frame pending | security | frame type <3>
  if(tx_cmd_buffer[0] == 102) {
    write_long(i++,0b01000001);
  } else {
    write_long(i++,0b01100001);
  }
  // src mode <2> | frame ver <2> | dest mode <2> | 00
  write_long(i++,0b11001100);

  write_long(i++,_seq_num); //sequence number

  //write panid
  for(j=0;j<2;j++) {
    write_long(i++,mrf_panid[j]);
  }

  //write dest addr
 for(j=0;j<8;j++) {
    write_long(i++,_parent_addr[j]);
  }

  //write src addr
  for(j=0;j<8;j++) {
    write_long(i++,mrf_mac[j]);
  }
  //end of header (21 bytes)

  write_long(i++,0x09); //mrf to udp command

  for(j=0;j<len;j++) {
    write_long(i++,tx_cmd_buffer[j]);
  }

  tx_ready();

  //Serial.println("TX DATA QUEUE");
}

void MRFClass::udp_to_mrf(int packetSize, byte* packetBuffer) {
  DeviceAddress dest_addr;
  uint8_t mrf_cmd;
  uint8_t rims_cmd;
  int i;
  int j;

  for(j=0;j<8;j++) {
    dest_addr[j] = packetBuffer[8-j];
  }
  mrf_cmd = packetBuffer[9];
  rims_cmd = packetBuffer[10];

  //printAddress(dest_addr);
  //Serial.print("mrf_cmd: ");
  //Serial.println(mrf_cmd, DEC);
  //Serial.print("rims_cmd: ");
  //Serial.println(rims_cmd, DEC);

  i=0;
  write_long(i++,21); //header length
  write_long(i++,21+packetSize-9); //frame length

  // 0 | PANID comp | ack| frame pending | security | frame type <3>
  //write_long(i++,0b01101001);
  write_long(i++,0b01100001);

  // src mode <2> | frame ver <2> | dest mode <2> | 00
  write_long(i++,0b11001100);

  write_long(i++,_seq_num); //sequence number

  //write panid
  for(j=0;j<2;j++) {
    write_long(i++,mrf_panid[j]);
  }

  //write dest addr
  for(j=0;j<8;j++){
	  write_long(i++,dest_addr[j]);
  }

  //write src addr
  for(j=0;j<8;j++) {
    write_long(i++,mrf_mac[j]);
  }
  //end of header (21 bytes)

  write_long(i++,mrf_cmd);
  write_long(i++,rims_cmd);

  for(j=11;j<packetSize;j++) {
    write_long(i++,packetBuffer[j]); //load packet data
    //Serial.println(packetBuffer[j], HEX);
  }
  
  //Serial.print("mrf cnt: ");
  //Serial.println(i);
  
  tx_ready();
}

void MRFClass::tx_status(void) {
  byte txstat;

  txstat = read_short(MRF_TXSTAT);

  if((txstat & 0x01) == 0x01){
    Serial.print("ERR: TX RETRIES: ");
    Serial.println((txstat & 0xC0) >> 6);
	
	if((txstat & 0x20) == 0x20) {
      Serial.println("ERR: TX CHAN BUSY");
    }
  }
}

void MRFClass::rx_disable(void) {
    write_short(MRF_BBREG1, 0x04); // RXDECINV - disable receiver
}

void MRFClass::rx_enable(void) {
    write_short(MRF_BBREG1, 0x00); // RXDECINV - enable receiver
}

void MRFClass::rx_flush(void) {
  write_short(MRF_RXFLUSH, 0x01);
}

void MRFClass::rx_toBuffer(void) {
  byte i;
  byte len;
  uint16_t ptr;

  if(_rx_count > 0) {
    Serial.println("rx !clr");
  }
  
  if((read_short(MRF_RXSR) & 0x04) == 0x04) {
    Serial.println("DECRYPTION ERROR");
    rx_flush();
  }
  else {
    noInterrupts();
    rx_disable();
	//setSS();
	
	ptr=0x300;
    len = read_long(ptr);
	for(i=0;i<=len+2;i++) {
	  rx_buff[i] = read_long(ptr++);
	}

	//resetSS();
	rx_enable();
    interrupts();
	
	//Serial.print("len: ");
	//Serial.println(len);
	_rx_count++;
  }
}

void MRFClass::rx_packet(void) {
  byte len;
  byte frm_ctrl1;
  byte crc1;
  byte crc2;
  
  rx_ptr=0;
  len = rx_buff[rx_ptr++];
 
  //Serial.print("len2: ");
  //Serial.println(len);
 
  frm_ctrl1 = rx_buff[rx_ptr++];
	
  switch(frm_ctrl1 & 0x07) {
    case 0x00:
      rx_beacon();
      break;
    case 0x01:
      rx_data(len, frm_ctrl1);
      break;
    case 0x02:
      Serial.println("ack frame");
      rx_ptr = rx_ptr+2;
	  break;
    case 0x03:
      rx_mac(frm_ctrl1);
      break;
    default:
      Serial.print("invalid frame type: ");
      Serial.println(frm_ctrl1, HEX);
  }
  
  //clear checksum
  crc1 = rx_buff[rx_ptr++];
  crc2= rx_buff[rx_ptr++];
  //rx_ptr = rx_ptr+2;
  
  _LQI = rx_buff[rx_ptr++];
  _RSSI = rx_buff[rx_ptr++];
	
  //Serial.print(crc1, HEX);
  //Serial.print(" ");
  //Serial.print(crc2, HEX);
  //Serial.println("");
}

void MRFClass::rx_data(byte len, byte frm_ctrl1) {
  uint8_t i;
  uint8_t j;
  uint8_t frm_ctrl2;
  uint8_t seq_num;
  DeviceAddress src_addr;
  DeviceAddress dest_addr;

  uint8_t data_cmd;
  uint8_t sensor_cnt;
  uint8_t data_len;
  uint8_t data[64];

//  uint8_t temp;
//  uint16_t raw;
//  float var;

//  union {
//    byte asBytes[4];
//    double asDouble;
//    uint32_t asUINT32;
//  } byte2var;

  frm_ctrl2 = rx_buff[rx_ptr++];
  seq_num = rx_buff[rx_ptr++];

  //Destination PAN ID
  rx_ptr=rx_ptr+2;

  //Destintation Address
  switch(frm_ctrl2 & 0b11000000) {
    case 0x0:  //dest addr not included
      break;
    case 0x80:  // short addr
      rx_ptr=rx_ptr+2;
      break;
    case 0xC0:  //long addr
      for(i=0;i<8;i++) {
		  dest_addr[i] = rx_buff[rx_ptr++];
	  }
      break;
  }

  //Source PAN ID
  switch(frm_ctrl1 & 0b01000000) {
    case 0x00:  //no pan compression
      rx_ptr=rx_ptr+2;
      break;
  }

  //Source Address
  switch(frm_ctrl2 & 0b00001100) {
    case 0x0:  //dest addr not included
      break;
    case 0x08:  //short addr
      rx_ptr=rx_ptr+2;
      break;
    case 0x0C:  //long addr
      for(i=0;i<8;i++) {
        src_addr[i] = rx_buff[rx_ptr++];
	  }
      break;
    default:
      memset(src_addr, 0, 8);
  }

  data_cmd = rx_buff[rx_ptr++];

  //Serial.print("data cmd: ");
  //Serial.println(data_cmd);

  switch(data_cmd) {
    case 0x02:
      rx_ptr=rx_ptr+64;
      break;
    case 0x03:
      sensor_cnt = rx_buff[rx_ptr++];
      data_len = sensor_cnt*10;

      for(i=0;i<data_len;i++) {
        data[i] = rx_buff[rx_ptr++];
      }

      //write out data
      Serial.write(9);  //header length
      Serial.write(10+data_len);  //packet length

      for(i=0;i<8;i++) {
		  Serial.write(src_addr[i]);
      }

      Serial.write(data_cmd);  //command id
      //end of serial header (9 bytes)

      Serial.write(sensor_cnt);

      for (i=0;i<data_len;i++) {
        Serial.write(data[i]);
      }
      break;
    case 0x04:  //Set temperature
      //temp sensor address
      break;
    case 0x05:  //heartbeat
      Serial.print("heartbeat: ");
      printAddress(src_addr);
      if(!memcmp(src_addr, _parent_addr, 8)) {
        //Serial.println("valid");
        _join_stat = 4;
      } else {
        Serial.println("not valid");
      }
      break;
    case 0x06:  //heartbeat request
      Serial.print("heartbeat REQ: ");
      printAddress(src_addr);
      tx_data_cmd(src_addr, 0x05);
      break;
   case 0x09:  //mrf to udp
      //mrf_to_udp();
	  i=0;
	  
	  //printAddress(src_addr);
	  
	  //Serial.print("len: ");
	  //Serial.println(len, DEC);
	  //Serial.print("ptr: ");
	  //Serial.println(rx_ptr, DEC);
	  
	  udp_buffer[i++] = 0x02; //udp cmd type (rims cmd)

	  for(j=0;j<8;j++) {
        udp_buffer[i++] = src_addr[7-j];
	  }

	  //log troubleshooting
	  //for(j=5;j<9;j++) {
        //byte2var.asBytes[j-5] = read_long(rx_ptr+j);
		//Serial.print(byte2var.asBytes[j-5], HEX);
		//Serial.print(" ");
      //}
	  //Serial.println("");
      //Serial.print("log temp: ");
      //Serial.println(byte2var.asDouble);
	  
	  for(j=0;j<(len-24);j++) {
	    udp_buffer[i++] = rx_buff[rx_ptr++];
		//if(j<9) {
		//  Serial.print(udp_buffer[i-1]);
		//  Serial.print(" ");
		//}
      }
	  //Serial.println("");
	  
	  _udp_pending = i;
	  break;
   case 0x10:  //rims command
      Serial.println("rims command");
      for(i=0;i<len-24;i++) {
		cmd_buffer[i] = rx_buff[rx_ptr++];
      }
	  cmd_count++;
      break;
	case 0x11:  //lights command
	  Serial.println("lights command");
      for(i=0;i<len-24;i++) {
		cmd_buffer[i] = rx_buff[rx_ptr++];
      }
	  cmd_count++;
	  break;
    default:
      Serial.print("Data Command ");
      Serial.print(data_cmd, HEX);
      Serial.println(" is not valid");
  }
}

void MRFClass::rx_mac(uint8_t frm_ctrl1) {
  uint8_t frm_ctrl2;
  uint8_t seq_num;
  uint8_t mac_cmd;

  uint16_t addr;
  DeviceAddress src_addr;
  uint16_t src_pan;

  frm_ctrl2 = rx_buff[rx_ptr++];;
  seq_num = rx_buff[rx_ptr++];;

  uint8_t i;

  //Destination PAN ID
  src_pan = rx_buff[rx_ptr++];;
  src_pan = (rx_buff[rx_ptr++] << 8) | src_pan;

  //Destintation Address
  switch(frm_ctrl2 & 0b11000000) {
    case 0x0:  //dest addr not included
      break;
    case 0x80:  // short addr
      rx_ptr = rx_ptr+2;
      break;
    case 0xC0:  //long addr
      rx_ptr = rx_ptr+8;
      break;
  }

  //Source PAN ID
  switch(frm_ctrl1 & 0b01000000) {
    case 0x00:  //no pan compression
      src_pan = rx_buff[rx_ptr++];
      src_pan = (rx_buff[rx_ptr++] << 8) | src_pan;
      break;
    case 0x40:  //pand compression
      break;
  }

  //Source Address
  switch(frm_ctrl2 & 0b00001100) {
    case 0x0:  //dest addr not included
      break;
    case 0x08:  //short addr
      rx_ptr = rx_ptr+2;
      break;
    case 0x0C:  //long addr
      for(i=0;i<8;i++) {
        src_addr[i] = rx_buff[rx_ptr++];;
      }
      break;
    default:
      memset(src_addr, 0, 8);
  }

  mac_cmd = rx_buff[rx_ptr++];;

  //printAddress(src_addr);

  switch(mac_cmd) {
    case 0x01:  //assoc req
      //Serial.println("RX ASSOC REQ");
      rx_ptr++;
      tx_assoc_resp(src_addr);
      break;
    case 0x02:  //assoc resp
      //Serial.println("RX ASSOC RESP");
      addr = rx_buff[rx_ptr++];;
      addr = (rx_buff[rx_ptr++] << 8) | addr;
      rx_ptr++;
      if(!memcmp(src_addr, _parent_addr, 8) && (src_pan == _panid)) {
        write_addr16(addr);
        write_pan(src_pan);
        _join_stat = 4;
        //Serial.print("ADDR SET TO: ");
        //Serial.println(addr, HEX);
        //Serial.print("PANID SET TO: ");
        //Serial.println(src_pan, HEX);
      }
      break;
    //case 0x03:  //dissoc not
    //case 0x04:  //data req
    //case 0x05:  //PAN ID Conflict
    //case 0x06:  //orphan not
    case 0x07:  //beacon req
      //Serial.println("RX BEACON REQ/TX BEACON");
      tx_beacon();
      break;
    //case 0x08:  //coord realign
    //case 0x09:  //GTS req
    default:
      Serial.print("MAC COMMAND ");
      Serial.print(mac_cmd, HEX);
      Serial.println(" NOT PROGRAMMED");
  }
}

void MRFClass::rx_beacon(void) {
  DeviceAddress parent_addr;
  uint16_t panid;
  uint8_t frm_ctrl2;
  uint8_t seq_num;
  uint8_t superframe1;
  uint8_t superframe2;
  uint8_t hops;
  int i;

  frm_ctrl2 = rx_buff[rx_ptr++];
  seq_num = rx_buff[rx_ptr++];

  panid = rx_buff[rx_ptr++];
  panid = (rx_buff[rx_ptr++] << 8) | panid;

  for(i=0;i<8;i++) {
	  parent_addr[i]=rx_buff[rx_ptr++];
  }

  superframe1 = rx_buff[rx_ptr++];; //ignore this byte
  superframe2 = rx_buff[rx_ptr++];; //
  hops = rx_buff[rx_ptr++];;

  if(_join_stat == 1) {
    if(superframe2 & 0x80) {
      memcpy(_parent_addr,parent_addr,8);
      _panid = panid;
	  _join_stat = 2;
    }
  }
  
  //Serial.println("rx beacon");
  
  _beacon_timer = millis();
}

void MRFClass::decrypt(void) {
  byte temp;
  
  setSS();
  temp = read_short(MRF_SECCON0);
  resetSS();
  
  write_short(MRF_SECCON0, temp |= 0x40);
}

byte MRFClass::get_interrupts(void) {
  byte interupts;
  
  setSS();
  interupts = read_short(MRF_INTSTAT);
  resetSS();
  return interupts;
}

// function to print a device address
void MRFClass::printAddress(DeviceAddress a) {
  uint8_t i;

  for(i=0;i<8;i++) {
    // zero pad the address if necessary
    if (a[7-i] < 16) Serial.print("0");
    Serial.print(a[7-i], HEX);
  }
  Serial.println("");
}

// function to print a device address
void MRFClass::setAddress(DeviceAddress* a, DeviceAddress* b) {
  //memcpy(client_list[client_cnt], dest_addr, 8);
  //memcpy(a, b, 8);
}

void MRFClass::PC_loop(void) {
  uint32_t current_time = millis();
  
  if(current_time - _beacon_timer > 30000) {
    tx_beacon();
  }
}

void MRFClass::node_loop(void) {
  uint32_t current_time = millis();

  switch(_join_stat) {
    case 1:  //transmitting beacon requests every 10 seconds
      if(current_time - _last_time > 10000) {
        //Serial.println("beacon req");
        tx_beacon_req();
        _last_time = current_time;
      }
      break;
    case 2: //received a beacon, tx assoc req
      tx_assoc_req();
      _join_stat = 3;
    case 3: //waiting for assoc resp
      if(current_time - _last_time > 20000) {
        init_node();
      }
      break;
    case 4: //node is active tx heartbeat
      if(current_time - _beacon_timer > 95000) {
        tx_data_cmd(_parent_addr, 0x06);
		_join_stat = 5;
      }
      break;
    case 5: //waiting for heartbeat resp
      if(current_time - _beacon_timer > 105000) {
        Serial.println("lost PC");
        init_node();
      }
      break;
    default:
      init_node();
  }
}