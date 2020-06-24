/* This sketch uses a ESP8266 wemos D1 mini and a PCB Hallard with RFM95 
/* Please note that lmic pin mapping in line 74-80 are different from Arduino board.
/* I also attached a DHT11
/* need to improve the payload format sent to the Gateway


/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses ABP (Activation-by-personalisation), where a DevAddr and
 * Session keys are preconfigured (unlike OTAA, where a DevEUI and
 * application key is configured, while the DevAddr and session keys are
 * assigned/generated in the over-the-air-activation procedure).
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!
 *
 * To use this sketch, first register your application and device with
 * the things network, to set or generate a DevAddr, NwkSKey and
 * AppSKey. Each device should have their own unique values for these
 * fields.
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "DHT.h"
#define DHTPIN 2   // DHTPIN2 --> occorre collegare il PIN DATA del DHT11 al PIN D4 del Wemos D1 mini
#define DHTTYPE DHT11
float h;
float t;
DHT dht(DHTPIN, DHTTYPE);

// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const PROGMEM u1_t NWKSKEY[16] = { 0xC1, 0x2E, 0x39, 0x54, 0xBE, 0x86, 0x9E, 0xB4, 0x08, 0xBD, 0x3F, 0x4A, 0xD4, 0xCF, 0xFC, 0x01 };

// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const u1_t PROGMEM APPSKEY[16] = { 0x36, 0x52, 0x37, 0x83, 0xF5, 0xBB, 0xE4, 0xB7, 0xAC, 0xD3, 0x6E, 0xDD, 0x2D, 0x91, 0x91, 0x6F };

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x260114E4 ; // <-- Change this address for every node!

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

//static uint8_t mydata[] = "Hello, world!";
unsigned char mydata[32];
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// Questa configurazione è valida per la trasmissione WEMOS D1 mini + scheda PCB DIYCON con RFM95 semtech.
const lmic_pinmap lmic_pins = {
//    .nss = 6,
    .nss = 15,    
    .rxtx = LMIC_UNUSED_PIN,
//    .rst = 5,
    .rst = LMIC_UNUSED_PIN,
//    .dio = {2, 3, 4},
      .dio = {4, 5, LMIC_UNUSED_PIN},
};


// procedura di lettura dati da DHT11
void getData() {
  h = dht.readHumidity();
  t = (dht.readTemperature());
   if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return; }
  Serial.print("Humidity: ");
  Serial.print(h);
  Serial.print(" %\t");
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.println(" *C ");
 
 // Payload configuration
const int PayloadSizeMaximum = 64 ;
char payload[PayloadSizeMaximum] = "";
const byte SensorReadingSeperator = ',' ;
int payloadLength = 0 ;

/* Copy the temperature into the payload
  payload[payloadLength] = 't'; */
//  payloadLength += 1 ;  //aggiunge un carattere NULL che crea casini col BLYNK
//  payloadLength += strlen( dtostrf(t, -1, 1, &payload[payloadLength]));

/*  NODE-RED node (da copiare nel clipboard)
N[{"id":"463c4c30.adee24","type":"tab","label":"TTN1","disabled":false,"info":""},{"id":"4b2d65f2.6028ec","type":"mqtt in","z":"463c4c30.adee24","name":"TTN real input (string)","topic":"+/devices/+/up","qos":"2","datatype":"auto","broker":"f249f1c.56a011","x":120,"y":100,"wires":[["27f0608f.55941"]]},{"id":"33761d6c.0b0582","type":"blynk-ws-out-write","z":"463c4c30.adee24","name":"BLYNK","pin":"3","pinmode":0,"client":"88231f92.c054b","x":540,"y":100,"wires":[]},{"id":"27f0608f.55941","type":"function","z":"463c4c30.adee24","name":"Decrypt Payload","func":"// msg2 - Decrypt to Text\nvar msg2 = { payload: msg.payload.length };\nmsg2.payload = JSON.parse(msg.payload);\nmsg2.payload = new Buffer(msg2.payload.payload_raw, 'base64').toString('ascii');\nmsg.payload = msg2.payload;\n//msg.payload = msg.payload.toString()\n//msg.payload = String.fromCharCode(msg.payload);\nreturn [ msg ];","outputs":1,"noerr":0,"x":340,"y":100,"wires":[["33761d6c.0b0582"]]},{"id":"f249f1c.56a011","type":"mqtt-broker","z":"","name":"moschella-app2","broker":"eu.thethings.network","port":"1883","clientid":"","usetls":false,"compatmode":true,"keepalive":"60","cleansession":true,"birthTopic":"","birthQos":"0","birthPayload":"","closeTopic":"","closePayload":"","willTopic":"","willQos":"0","willPayload":""},{"id":"88231f92.c054b","type":"blynk-ws-client","z":"","name":"BLYNK write","path":"ws://blynk-cloud.com/websockets","key":"0xOqmGrQmfDc2hP9yUGR5X7Z4ZdMe6hq","dbg_all":true,"dbg_read":false,"dbg_write":false,"dbg_notify":false,"dbg_mail":false,"dbg_prop":false,"dbg_sync":false,"dbg_bridge":false,"dbg_low":false,"dbg_pins":"3","multi_cmd":true,"proxy_type":"no","proxy_url":"","enabled":true}]
 */

  // Copy the temperature into the payload
 // payload[payloadLength] = 't';
 // payloadLength += 1 ;
 // payload[payloadLength] = ' ';
 // payloadLength += 1 ;
  payloadLength += strlen( dtostrf(t, -1, 1, &payload[payloadLength]));
  payload[ payloadLength] = SensorReadingSeperator;
  payloadLength += sizeof(SensorReadingSeperator) ;   //aggiunge il separatore "," dopo la temperatura
 
  // Copy the humidity into the payload
  //payload[ payloadLength] = 'h';
  //payloadLength += 1 ;
  //payload[ payloadLength] = ' ';
  //payloadLength += 1 ;
  payloadLength += strlen( dtostrf(h, -1, 0, &payload[payloadLength]));  
  
  // display info about payload then send it (No ACK) with LoRa 
  Serial.print( "RFM9X/SX127X Payload length:");
  Serial.print( payloadLength );
  Serial.println( " bytes" );   

  mydata[0] = payload[0];
  mydata[1] = payload[1];
  mydata[2] = payload[2];
  mydata[3] = payload[3];
  mydata[4] = payload[4];
  mydata[5] = payload[5];
  mydata[6] = payload[6];
  mydata[7] = payload[7];
  mydata[8] = payload[8];
  mydata[9] = payload[9];
  mydata[10] = payload[10];
  mydata[11] = payload[11];
  mydata[12] = payload[12];
  mydata[13] = payload[13];
  mydata[14] = payload[14];
  mydata[15] = payload[15];

  Serial.println( payload );
}


void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}




void do_send(osjob_t* j){
    
    // read temperature from DHT11 on LORA event  
    getData();
    
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    Serial.begin(115200);
    Serial.println(F("Starting"));
    dht.begin();

    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    // NA-US channels 0-71 are configured automatically
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    #elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);

    // Start job
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}
