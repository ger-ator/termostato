/*
 * Configuracion del PID
 * Poner antes de importar Souliss para que el 
 * preprocesador no la lie con la macro Initialize
 */
#include <PID_v1.h>
#define KP 45
#define KI 0.05
#define KD 0
double sp, pv, out;
PID myPID(&pv, &out, &sp, KP, KI, KD, DIRECT);

/*
 *  Configuracion del DHT22
 */
#include "DHT.h"
#define DHTPIN D2
DHT dht;

#include <ArduinoOTA.h>
#include "SoulissFramework.h"
#include "bconf/MCU_ESP8266.h"
#include "conf/Gateway.h"
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
#define SETPOINT            0
#define PID_MODE            2
#define TEMPERATURE         4
#define HUMIDITY            6
#define DEMANDA             8

/*
 * Configuracion de SLOTS remotos
 */
#define RELE         0
#define SP_POTENCIA  1

void setup()
{
  /*  
   * Iniciar controlador PID
   */
  myPID.SetOutputLimits(0, 100);
  myPID.SetMode(MANUAL);
  myPID.SetSampleTime(10000);

  /*
   * Iniciar sensor DHT22
   */
  dht.setup(DHTPIN);

  /*
   * Inicializacion de Souliss
   */
  Initialize();
  GetIPAddress();
  SetAsGateway(myvNet_dhcp);
  SetAddress(Gateway_Address, 0xFF00, 0x0000);
  SetAsPeerNode(SalonPeer_Address, 1);

  Set_Temperature(TEMPERATURE);
  Set_Humidity(HUMIDITY);
  Souliss_SetT51(memory_map, DEMANDA);
  Set_T11(PID_MODE);
  Set_Temperature_Setpoint(SETPOINT);
  float sp_inicial = 22.0;
  ImportAnalog(SETPOINT, &sp_inicial);
  ArduinoOTA.setHostname("termostato-prueba");
  ArduinoOTA.begin();
}

void loop()
{
  EXECUTEFAST() {
    UPDATEFAST();

    FAST_2110ms() {
      float h = dht.getHumidity();
      float t = dht.getTemperature();
      if (!isnan(t)) {
        ImportAnalog(TEMPERATURE, &t);
        Souliss_Logic_T52(memory_map, TEMPERATURE, 0.004, &data_changed);
      }
      if (!isnan(h)) {
        ImportAnalog(HUMIDITY, &h);
        Logic_Humidity(HUMIDITY);
      }
    }
    FAST_1110ms() {
      Logic_Temperature_Setpoint(SETPOINT);
      Logic_T11(PID_MODE);
      if (isTrigger()) {
        if (mOutput(PID_MODE) == Souliss_T1n_OnCoil) {
          myPID.SetMode(AUTOMATIC);
          Send(SalonPeer_Address, RELE, Souliss_T1n_AutoCmd);
        }
        else {
          myPID.SetMode(MANUAL);
          Send(SalonPeer_Address, RELE, Souliss_T1n_OffCmd);
        }       
      }
    }
    FAST_GatewayComms();
  }
  EXECUTESLOW() {
    UPDATESLOW();

    SLOW_10s() {
      pv = mOutputAsFloat(TEMPERATURE);      
      sp = mOutputAsFloat(SETPOINT);
      if(!isnan(pv) && !isnan(sp)) {
        if (myPID.Compute()) {
          float demanda = (float) out;
          ImportAnalog(DEMANDA, &demanda);
          Souliss_Logic_T52(memory_map, DEMANDA, 0.004, &data_changed);
          if (isTrigger()) SendData(SalonPeer_Address, SP_POTENCIA, &mOutput(DEMANDA), 2);
        }
      }
    }
  }
  ArduinoOTA.handle();
}
