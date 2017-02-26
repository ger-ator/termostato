#include <ArduinoOTA.h>
#include "SoulissFramework.h"
#include "bconf/MCU_ESP8266.h"
#include "conf/IPBroadcast.h"
#include <ESP8266WiFi.h>
#include <EEPROM.h>

/*** All configuration includes should be above this line ***/
#include "Souliss.h"

/*
 * Configuracion de red
 */
 #define Gateway_Address 0xAB01
 #define SalonPeer_Address 0xAB02
 
/*
 * Configuracion de SLOTS locales
 */
#define RELE         0
#define SP_POTENCIA     1

#define RELEPIN D0

void setup()
{
  /*
   * Configurar pin del rele como salida
   */
  pinMode(RELEPIN, OUTPUT);

  /*
   * Inicializacion de Souliss
   */
  Initialize();
  GetIPAddress();
  SetAddress(SalonPeer_Address, 0xFF00, Gateway_Address);

  Set_AutoLight(RELE);
  Set_T61(SP_POTENCIA);
  
  ArduinoOTA.setHostname("rele-prueba");
  ArduinoOTA.begin();
}

void loop()
{
  EXECUTEFAST() {
    UPDATEFAST();

    FAST_50ms() {
      Logic_AutoLight(RELE);
      Timer_AutoLight(RELE);
      nDigOut(RELEPIN, Souliss_T1n_Coil, RELE);
    }
    FAST_2110ms() {
      Logic_T61(SP_POTENCIA);
    }
    FAST_PeerComms();
  }

  EXECUTESLOW() {
    UPDATESLOW();

    SLOW_10s() {
      if (mOutput(RELE) & 0xF0) { //Si RELE en AUTO
        U8 ciclos = (U8) (2 * mOutputAsFloat(SP_POTENCIA));
        SetInput(RELE, Souliss_T1n_AutoCmd + ciclos);
      }
    }
  }
  ArduinoOTA.handle();
}
