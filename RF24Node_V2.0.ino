#include <SPI.h>
#include "RF24.h"            // https://github.com/nRF24/RF24/
#include <printf.h>          // look in Arduino/libraries/RF24 for the printf.h file
#include <TimeLib.h>         // https://github.com/PaulStoffregen/Time

// Node ID has nothing to do with the nRF24 Radio addressing, it is used in my protocol
#define Gateway_ID   00        // Node 0 = Gateway ID
#define Broadcast_ID 01        // Node 1 = Broadcast ID
#define my_Node_ID   20        // Node ID 20 to 255 is for the clients
//#define my_Node_ID 28        // Test Node 2
//#define my_Node_ID 56        // Test Node 3      

// I used the following libraries and code to construct my programme
// https://sandervandevelde.wordpress.com/2016/03/06/using-nrf24l01-modules-between-arduinos/
// https://github.com/SensorsIot/NRF24L01-Bi-Directional-Communication/blob/master/NRF24L01-Bi-Directional-Communication.ino
// https://tmrh20.blogspot.com/
// https://www.instructables.com/Coding-Timers-and-Delays-in-Arduino/
// https://nrf24.github.io/RF24/index.html
// https://www.pjrc.com/teensy/td_libs_Time.html

// on Nano or Mini Pro
// |--------------------|
// |    UPSIDE      8 7 |
// |     PINS       6 5 |\
// |      ON        4 3 |\\
// |  OTHER SIDE    2 1 |\\
// |--------------------|\\
//                     \  \
// # DESC COLOR  Arduino port
// 1 GND  BLACK           GND
// 2 VCC  RED           3.3V!
// 3 CE   ORANGE           10
// 4 CSN  YELLOW            9
// 5 SCK  GREEN            13
// 6 MOSI BLUE             11
// 7 MISO VIOLET           12
// 8 LED  --|<|--[|||]-----Vss/GND Optional

// Select one of these, must match Gateway settings
//#define DATARATE RF24_2MBPS
#define DATARATE RF24_1MBPS
//#define DATARATE RF24_250KBPS

// Select one of these
#define RF_POWER RF24_PA_MIN
//#define RF_POWER RF24_PA_LOW
//#define RF_POWER RF24_PA_MED
//#define RF_POWER RF24_PA_HIGH

// User Defines
#define SERIAL_DEBUG
//#define SERIAL_DEBUG_TIMES
//#define SERIAL_MESSAGE_STATS
//#define POLL_DEBUG
//#define TIMES_DEBUG
//#define TELEMETRY_DEBUG
//#define PRINT_STATS
//#define PRINT_RX
//#define PRINT_TX
 
/* Hardware configuration: */
/* Set up nRF24L01 radio on SPI bus plus pins 10 & 9 */
RF24 radio(10, 9);

// These addresses work like a point to point connection 
uint8_t addresses[2][6] = {"10101","20202"};

#define   LED_1        8           // Red LED (Data RX/TX)

// Timer & LED's
#define   LED_ON        0              // Current sink, connect cathode to arduino pin
#define   LED_OFF       1
#define   ON            1
#define   OFF           0
bool      LED_State   = OFF;
#define   INTERVAL_LED  250
uint32_t  time_1      = 0;

// Radio TX Interval
#define   INTERVAL_TX     5000      // For testing, send data every x000ms
uint32_t  time_2        = 0;

// Create two data buffers for radio rx and tx data
#define BUFFER_LEN   32               
uint8_t radioRXbuffer[BUFFER_LEN];   // RX Buffer
uint8_t radioTXbuffer[BUFFER_LEN];   // TX Buffer


// Protocol Header Data TX
struct protoCol_TX_HEADER{ 
  uint8_t   node_id     = my_Node_ID;
  uint8_t   id_to       = 0;
  uint8_t   type        = 0;
  uint16_t  message_id  = 0;  
};
protoCol_TX_HEADER messageHEADER_RX;

// Protocol Header Data RX
struct protoCol_RX_HEADER{ 
  uint8_t   node_id     = 0;
  uint8_t   id_to       = 0;
  uint8_t   type        = 0;
  uint16_t  message_id  = 0;  
};
protoCol_RX_HEADER messageHEADER_TX;

struct pollTXData{
  uint32_t  txData    = 0;
  uint32_t  rxData    = 0;
};
pollTXData pollTX;

struct pollRXData{
  uint32_t  txData    = 0;
  uint32_t  rxData    = 0;
};
pollRXData pollRX;


struct timeData{
  time_t  rx_time    = now();
};
timeData nt_Time;               // new time value

struct message_Stats{ 
  uint16_t  tx_vol         = 0;
  uint16_t  rx_vol         = 0;
  uint16_t  mesg_failures  = 0;
  uint16_t  mesg_delivered = 0;  
  uint16_t  mesg_retries   = 0;
  uint16_t  tx_time        = 0;
  uint16_t  tx_time_end    = 0;
  uint16_t  tx_time_start  = 0;
  uint16_t  rx_time        = 0;
  uint16_t  rx_time_end    = 0;
  uint16_t  rx_time_start  = 0;
  uint16_t  msg_time       = 0;
};
message_Stats nodeStats;
// Print Message Stats Timer
#define   INTERVAL_STATS     10000      // For testing, send data every x000ms
uint32_t  time_3           = 0;

// Time request interval
//#define   INTERVAL_TIME      5000
uint32_t  INTERVAL_TIME    = (1UL*60UL*60UL*1000UL);   // 1 hour
uint32_t  time_4           = 0;
bool      time_request     = true;

uint32_t broadcast_interval  = 5000;
uint32_t time_5              = 0;

// Define variables for messages
bool       msgComplete      = false;
bool       msgTimeOutFlag   = false;
#define    MESSAGE_TIMEOUT    10         // If no Ack is Received in the time the message delivery has failed
uint32_t   time_mesg        = 0;

// Define Protocol type requests 
#define POLL_REQUEST            10       // System testing, sends number and message id
#define POLL_RESPONSE           11       // System testing, receives the same number and message id
#define DATA_MESSAGE_SEND       12       
#define DATA_MESSAGE_DELIVERY   13       
#define TIME_REQUEST            14       // Time Request for nRF24 time sync message
#define TIME_RESPONSE           15       // Time sync message response
#define TX_BROADCAST            0xFF       // All node Broadcast Message

// Declare protocol variables
uint16_t  tx_poll_message_id  = 0;
uint16_t  tx_time_message_id  = 0;
uint16_t  tx_broadcast_message_id  = 0;

/* ************************************************************************************************************ */
void setup() {

    
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }  
  printf_begin();
  printf_P(PSTR("RF24 Node ID:%02u\n\r"),my_Node_ID);

  pinMode(LED_1, OUTPUT);
  digitalWrite(LED_1, LED_OFF);
 
  radio.begin();
  
  // Addressing
  radio.setAutoAck(false);
  radio.openReadingPipe(1,addresses[0]);

  // Radio Settings
  radio.setPALevel(RF_POWER);
  radio.setDataRate(DATARATE);
  radio.setChannel(0x7F);

  // Setup the sensors
  randomSeed(my_Node_ID);
  
  radio.startListening();


}

/* ************************************************************************************************************ */
void loop() {

  // LED Control, switch the LED off after the on Interval
  if (LED_State && ((millis() - time_1) >= INTERVAL_LED)) {
      LED_State = OFF;
      digitalWrite(LED_1, LED_OFF);
    }

  // Radio - TX Poll Request
  if(millis() >= time_2 + INTERVAL_TX){
      // Check if we need to set the time
      if (year() == 1970){
        tx_time_request();
      }
      // Send a poll request     
      time_2 += INTERVAL_TX;
      tx_poll_request();
    }

  // Show the stats
  #if defined (SERIAL_MESSAGE_STATS)
    if(millis() >= time_3 + INTERVAL_STATS){
        time_3 += INTERVAL_STATS;
        print_stats();                  // Display Message Stats
      }
  #endif

  // If the Node Reboots, get the Time
  if ((timeStatus()== 0) && (time_request == true)){
     tx_time_request();
     time_request = false;
    }

  // Sync the time with the Gateway
  if(millis() >= time_4 + INTERVAL_TIME){
      time_4 += INTERVAL_TIME;
      tx_time_request();
    }

  // Send an random all Node Broadcast Message
  if(millis() >= time_5 + broadcast_interval){
    broadcast_interval = random(10000, 15000); 
    time_5 += broadcast_interval;
    tx_broadcast();
  }

  if( radio.available()){
    
    nodeStats.rx_time_start = micros();
    nodeStats.rx_vol++;                   // Update the Stats
    
    while (radio.available()) {
      radio.read(&radioRXbuffer, sizeof(radioRXbuffer) + 1);
    }

    // Protocol Header Information
    memcpy (&messageHEADER_RX, &radioRXbuffer, sizeof(messageHEADER_RX) + 1);

    // Did we receive a broadcast message?
    if (messageHEADER_RX.type == TX_BROADCAST){

        // Switch on the Broadcast Type
        switch (messageHEADER_RX.node_id) {
            case Gateway_ID:
              printf_P(PSTR("GATEWAY "));
              break;
            default: 
              printf_P(PSTR("NODE "));    
              break;
          }

        printf_P(PSTR("BROADCAST RECEIVED ["));
        printf_P(PSTR("%04X]\n\r"),messageHEADER_RX.message_id);
     
    }

   
    if (my_Node_ID == messageHEADER_RX.id_to){          // If the message is for this Node, deal with it
        // Switch on the Request Type
        switch (messageHEADER_RX.type) {
            case POLL_RESPONSE:
              rx_poll_response();
              break;
            case DATA_MESSAGE_SEND:    
              break;
            case TIME_RESPONSE:
              rx_time_response();
              break;
          }
      }
    
  }

  // Check if we have a message timeout
  if (msgTimeOutFlag && ((millis() - time_mesg) >= MESSAGE_TIMEOUT)) {
      msgTimeOutFlag = false;
      #if defined (SERIAL_DEBUG)
        // printf_P(PSTR("TX: Message ID: %04X not delivered.\n\r"),messageHEADER_TX.message_id);
      #endif
      nodeStats.mesg_failures++;      // Update the Stats
    }

  if (msgComplete) {

    msgComplete = false;

    // This is a good place to print debug messages
    #if defined (SERIAL_DEBUG_TIMES)
      times_debug();
    #endif
 
    #if defined (PRINT_STATS)
      print_stats();
    #endif

    }

}

/* ************************************************************************************************************ */
void tx_poll_request()
{

    //printf_P(PSTR("tx_poll_request()\n\r"));
    
    tx_led();                 // Blink the LED
    nodeStats.tx_time_start = micros();
    radio.stopListening();
    
    // Set the Header information
    messageHEADER_TX.node_id = my_Node_ID;             // Our ID
    messageHEADER_TX.id_to = Gateway_ID;               // Who must respond?
    messageHEADER_TX.type = POLL_REQUEST;              // Set the message type
    messageHEADER_TX.message_id = random(0xFFFF);      // Create a funky ID
    tx_poll_message_id = messageHEADER_TX.message_id;  // Save the message id
    
    // Copy the protocol structure to the TX buffer
    memcpy (&radioTXbuffer, &messageHEADER_TX, sizeof(messageHEADER_TX));
    
    // Create and set some message data
    pollTX.txData = millis();                       // We send our millis()
    pollTX.rxData = 0;                              // Then we will receive is back
    
    // Add the Data
    memcpy (&radioTXbuffer[sizeof(messageHEADER_TX)], &pollTX, sizeof(pollTX));
    
    print_tx_message_header();
    
    radio.openWritingPipe(addresses[0]);
    radio.write(&radioTXbuffer, sizeof(radioTXbuffer) + 1);    // Publish the message, if we get the ack we know it was delivered

    // Start the message timeout timer
    msgTimeOutFlag = true;                  // If set the timer will run until the message timesout
    time_mesg = millis();
    radio.startListening();
    nodeStats.tx_vol++;                      // Update the Stats
    nodeStats.tx_time_end = micros();
 
  }

/* ************************************************************************************************************ */
void tx_time_request()
{

    // printf_P(PSTR("tx_time_request()\n\r"));
    
    tx_led();       // Blink LED
    nodeStats.tx_time_start = micros();
    radio.stopListening();
    
    // Set the Header information
    messageHEADER_TX.node_id = my_Node_ID;             // Our ID
    messageHEADER_TX.id_to = Gateway_ID;               // Who must respond?
    messageHEADER_TX.type = TIME_REQUEST;              // Set the message type
    messageHEADER_TX.message_id = random(0xFFFF);      // Create a funky ID
    tx_time_message_id = messageHEADER_TX.message_id;  // Save the message id
    
    // Copy the protocol structure to the TX buffer
    memcpy (&radioTXbuffer, &messageHEADER_TX, sizeof(messageHEADER_TX));
    
    print_tx_message_header();
    
    radio.openWritingPipe(addresses[0]);
    radio.write(&radioTXbuffer, sizeof(radioTXbuffer) + 1);    // Publish the message, if we get the ack we know it was delivered

    // Start the message timeout timer
    msgTimeOutFlag = true;                            // If set the timer will run until the message timesout
    time_mesg = millis();
    radio.startListening();
    nodeStats.tx_vol++;                               // Update the Stats
    nodeStats.tx_time_end = micros();

}

/* ************************************************************************************************************ */
void tx_broadcast()
{

    //printf_P(PSTR("tx_broadcast()\n\r"));
    clear_radio_buffers();
   
    nodeStats.tx_time_start = micros();
    radio.stopListening();

    // Set the Header information
    messageHEADER_TX.id_to = Gateway_ID;                    // Set our ID here as well
    messageHEADER_TX.type = TX_BROADCAST;                   // TX_BROADCAST
    messageHEADER_TX.message_id = random(0xFFFF);           // Create a funky ID
    tx_broadcast_message_id = messageHEADER_TX.message_id;  // Save the message id

    // Copy the protocol structure to the TX buffer
    memcpy (&radioTXbuffer, &messageHEADER_TX, sizeof(messageHEADER_TX));

    radio.openWritingPipe(addresses[0]);
    radio.write(&radioTXbuffer, sizeof(radioTXbuffer) + 1);
    
    nodeStats.tx_vol++;                   // Update the Stats
    nodeStats.tx_time_end = micros();
    msgComplete = true;

    radio.startListening();

    // Show the outgoing message header
    print_tx_message_header();

}

/* ************************************************************************************************************ */
void rx_poll_response(){

      //printf_P(PSTR("rx_poll_response()\n\r"));
           
      // Copy the protocol structure from the RX buffer
      memcpy (&messageHEADER_RX, &radioRXbuffer, sizeof(messageHEADER_RX));

      // Copy the data out of the Buffer
      memcpy (&pollRX, &radioRXbuffer[sizeof(messageHEADER_RX)], sizeof(pollRX));
      
      print_rx_message_header();
      
      if (messageHEADER_RX.message_id == tx_poll_message_id){
        
        msgComplete = true;                          // else, drop the message and wait for the next one.
        msgTimeOutFlag = false;                      // Stop the timeout timer
        nodeStats.mesg_delivered++;                  // Update the Stats
        nodeStats.rx_time_end = micros();            // Update the Stats
        
      }
              
  }

/* ************************************************************************************************************ */
void rx_time_response(){
  
      //printf_P(PSTR("rx_time_response()\n\r"));

      // Copy the protocol structure from the RX buffer
      memcpy(&pollRX, &radioRXbuffer[sizeof(messageHEADER_RX)], sizeof(pollRX));

      if (messageHEADER_RX.message_id == tx_time_message_id){
     
        // Copy the Time from the RX buffer
        memcpy(&nt_Time, &radioRXbuffer[sizeof(messageHEADER_RX)], sizeof(nt_Time));
  
        // Adjust our Time
        setTime(nt_Time.rx_time);
  
        print_rx_message_header();
      
        msgComplete = true;                          // else, drop the message and wait for the next one.
        msgTimeOutFlag = false;                      // Stop the timeout timer
        nodeStats.mesg_delivered++;                  // Update the Stats
        nodeStats.rx_time_end = micros();            // Update the Stats
      }
  
  }


/* ************************************************************************************************************ */
void clear_radio_buffers()
{

  // printf_P(PSTR("clear_radio_buffers()\n\r"));
   
  for (uint8_t i = 0; i <= sizeof(radioTXbuffer); i++) {
    radioTXbuffer[i] = 0x00;
  }
  for (uint8_t i = 0; i <= sizeof(radioRXbuffer); i++) {
    radioRXbuffer[i] = 0x00;
  }

}

/* ************************************************************************************************************ */
void dump_radio_buffers()
{

  print_tx_buffer();
  print_rx_buffer();
  
}

/* ************************************************************************************************************ */
void print_rx_buffer()
{
    
  printf_P(PSTR("RX Buffer: "));
  for (uint8_t i = 0; i <= (sizeof(radioRXbuffer)-1); i++) {
    printf_P(PSTR("%02X "), radioRXbuffer[i]);
  }
  printf_P(PSTR("\n\r"));
  
}

/* ************************************************************************************************************ */
void print_tx_buffer()
{
  printf_P(PSTR("TX Buffer: "));
  for (uint8_t i = 0; i <= (sizeof(radioTXbuffer)-1); i++) {
    printf_P(PSTR("%02X "), radioTXbuffer[i]);
  }
  printf_P(PSTR("\n\r"));
}

/* ************************************************************************************************************ */
void print_rx_message_header()
{
  printf_P(PSTR("RX MESG_ID: %04X "),messageHEADER_RX.message_id);
  printf_P(PSTR("FROM: %02u "),messageHEADER_RX.node_id);
  printf_P(PSTR("TO: %02u "),messageHEADER_RX.id_to);
  printf_P(PSTR("TYPE: %02X "),messageHEADER_RX.type);
  switch (messageHEADER_RX.type) {
      case POLL_RESPONSE:
        printf_P(PSTR("(POLL_RESPONSE) DATA ["));
        printf_P(PSTR("txData: %04X "), pollRX.txData);
        printf_P(PSTR("rxData: %04X"), pollRX.rxData);
        break;
      case TIME_RESPONSE:
        printf_P(PSTR("(TIME_RESPONSE) ["));
        //printf_P(PSTR("Time: %lu "), nt_Time.rx_time );
        printf_P(PSTR("Time: "));
        printTime(nt_Time.rx_time);
        break;
      case TX_BROADCAST:
        printf_P(PSTR("(TX_BROADCAST) ["));
        break;
    }
  printf_P(PSTR("]\n\r"));
    
}

/* ************************************************************************************************************ */
void print_tx_message_header()
{
  printf_P(PSTR("TX MESG_ID: %04X "),messageHEADER_TX.message_id);
  printf_P(PSTR("FROM: %02u "),messageHEADER_TX.node_id);
  printf_P(PSTR("TO: %02u "),messageHEADER_TX.id_to);
  printf_P(PSTR("TYPE: %02X "),messageHEADER_TX.type);
  switch (messageHEADER_TX.type) {
      case POLL_REQUEST:
        printf_P(PSTR("(POLL_REQUEST) DATA ["));
        printf_P(PSTR("txData: %04X "), pollTX.txData);
        printf_P(PSTR("rxData: %04X"), pollTX.rxData);
        break;
      case TIME_REQUEST:
        printf_P(PSTR("(TIME_REQUEST) ["));
        break;
      case TX_BROADCAST:
        printf_P(PSTR("(TX_BROADCAST) ["));
        printf_P(PSTR("%04X"),messageHEADER_TX.message_id);
        break;
    }    
  printf_P(PSTR("]\n\r"));
}

/* ************************************************************************************************************ */
void tx_led(){
 
    // Blink the LED
    digitalWrite(LED_1, LED_ON);
    LED_State = ON;
    time_1 = millis();

    return;
  }

/* ************************************************************************************************************ */
void printTime(time_t t){
  
  // YYYY-MM-DD 
  printf_P(PSTR("%04d"),year(t));
  printf_P(PSTR("-%02d"),month(t));
  printf_P(PSTR("-%02d"),day(t));
  // HH:mm:ss 
  printf_P(PSTR(" %02d"),hour(t));
  printf_P(PSTR(":%02d"),minute(t));
  printf_P(PSTR(":%02d"),second(t));
   
}

/* ************************************************************************************************************ */
void poll_debug()
{
   
  // Protocol as sending
  printf_P(PSTR("POLL TX: FROM: %02u "),messageHEADER_TX.node_id);
  printf_P(PSTR("TO: %02u "),messageHEADER_TX.id_to);
  printf_P(PSTR("TYP: %02X "),messageHEADER_TX.type);
  printf_P(PSTR("MESG_ID: %04X "),messageHEADER_TX.message_id);
  printf_P(PSTR("TXD: %04X "),pollTX.txData);
  printf_P(PSTR("RXD: %04X "),pollTX.rxData);
  printf_P(PSTR("TIME: %04u uS\n\r"),nodeStats.tx_time);

  // Protocol as received
  printf_P(PSTR("POLL RX: FROM: %02u "),messageHEADER_RX.node_id);
  printf_P(PSTR("TO: %02u "),messageHEADER_RX.id_to);
  printf_P(PSTR("TYP: %02X "),messageHEADER_RX.type);
  printf_P(PSTR("MESG_ID: %04X "),messageHEADER_RX.message_id);
  printf_P(PSTR("TXD: %04X "),pollRX.txData);
  printf_P(PSTR("RXD: %04X "),pollRX.rxData);
  printf_P(PSTR("TIME: %04u uS\n\r"),nodeStats.msg_time);
  
}

/* ************************************************************************************************************ */
void time_calcs(){
  
  nodeStats.tx_time = nodeStats.tx_time_end - nodeStats.tx_time_start;
  nodeStats.rx_time = nodeStats.rx_time_end - nodeStats.rx_time_start;
  // nodeStats.msg_time = nodeStats.tx_time + nodeStats.rx_time;
  nodeStats.msg_time = nodeStats.rx_time_end - nodeStats.tx_time_start;

  }

/* ************************************************************************************************************ */
void times_debug()
{

  time_calcs();

  printf_P(PSTR("  TX time: %u us\n\r"),nodeStats.tx_time);
  printf_P(PSTR("  RX time: %u us\n\r"),nodeStats.rx_time);
  printf_P(PSTR("MESG time: %u us\n\r"),nodeStats.msg_time);

}

/* ************************************************************************************************************ */
void print_stats()
{

   printf_P(PSTR("STATS: TX: %d"),nodeStats.tx_vol);
   printf_P(PSTR(" RX: %d"),nodeStats.rx_vol);
   printf_P(PSTR(" DEL: %u"),nodeStats.mesg_delivered);
   printf_P(PSTR(" FAIL: %u"),nodeStats.mesg_failures);
   printf_P(PSTR(" Time: "));
   printTime(nt_Time.rx_time);
   printf_P(PSTR("\n\r"));
     
}
