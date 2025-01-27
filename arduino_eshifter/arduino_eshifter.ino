/******************************************************************
Created with PROGRAMINO IDE for Arduino - 26.01.2025 | 13:38:17
Update      :
Author      : Chris74 pour Kiki85 (Cyclurba)
Description : e-shifter ebike Vanmoof s5, Arduino Nano

Guithub (codes et doc) : https://github.com/Chris741233/ebike_arduino_eshifter

forum Cyclurba : https://cyclurba.fr/forum/757849/programme-arduino-schifter.html?discussionID=31918


INA219 :
https://how2electronics.com/how-to-use-ina219-dc-current-sensor-module-with-arduino/
https://wiki.dfrobot.com/Gravity:%20I2C%20Digital%20Wattmeter%20SKU:%20SEN0291

L298N:
https://www.robotique.tech/tutoriel/commander-un-moteur-par-arduino-et-le-module-l298n/

conso montage a vide (Arduino nano + l298n + ina219) sous 8V = 29mA (sans USB, sans diode alim)

Voir aussi schema et script de calibration moteur et intensite

******************************************************************/


// -------- INCLUDE -------------------------------


#include <Wire.h>          // I2C
#include "INA219.h"        // from Rob.Tillaart, voir https://github.com/RobTillaart/INA219

// simplification envois Serial et screen serial, expl: Serial << "A:" << var << endl;
#include <Streaming.h>     // Peter Polidoro, https://github.com/janelia-arduino/Streaming    

// duree d'appui bouton et gestion bouton
// librairie du membre bricoleau, voir https://forum.arduino.cc/t/partage-boutons-poussoirs/361656
// pour info n'est pas dispo dans le depot officiel Arduino
#include "simpleBouton.h"  


#include <Timeout.h>       // timer, tfeldmann  https://github.com/tfeldmann/Arduino-Timeout 
// ne pas confondre avec TimeOut.h !


// -------- ARDUINO PIN, selon schema joint --------------

// butons
const int BUTON_UP_PIN = 3;    // bouton poussoir Plus (UP, Monter vers v3), interupt 3 possible  
const int BUTON_DOWN_PIN = 2;  // bouton poussoir Moins (DOWN, Descendre vers v1), interupt 2 possible

// L298N, motor driver
const int ENA_PIN = 9;   // enable A, PWM
const int IN1_PIN = 8;   // in1
const int IN2_PIN = 7;   // in2 

// LED interne Uno/Nano
const int LED_PIN = 13;  // LED_BUILTIN


// INA219 curent sensor  
// I2C PIN SDA A4, SCL A5


// --------  CONSTANTES SYSTEME, MODIFIABLES ------------------------


#define ADRESSE_INA 0x40     // adresse I2C INA219, generalement 0x40 (DFRobot=0x45) 
#define RSHUNT_INA 0.100     // valeur shunt, generalement R100 = 0.100 Ohm  (DFRobot=0.010) 

const int SPEED_MOTOR = 255; // speed moteur par defaut, 0 to 255 max (PWM 8bit)

const int MA_RUN = 360;      // (300) mA, conso en utilisation (sous ~6.5V au moteur)
const int MA_MARGE = 200;    // (150) mA, + marge de butee a pas depasser pour securite (stale = ? mA sous 8V)
const int MA_MAX = MA_RUN + MA_MARGE; 

// -- ces parametres influencent sur le controle intensite max moteur
const int SUP_PIC = 30;   // (30) filtrer les premieres ms du pic demarage moteur, env. 30-50=Ok ou 0 pour desactive
const int MIN_STAT = 20;  // 20mA minimum pour etre affiche en direct et ajoute aux stats (laisser comme ca !)

const int TIMEOUT_1to3 = 2400;  // ms, temps de reference pour passer de v1 à v3 (butee basse a haute)
const int MARGE_HOME = 100;     // ms, petite marge a ajouter a TIMEOUT_1to3 pour etre sur de la position home v1 au boot
const int TIMEOUT_STEP = TIMEOUT_1to3/2; // ms, etape (1to2, 2to3, etc..), en principe TIMEOUT_1to3 / 2


const int DELAY_PUSH_BTN = 800; // ms, lorsqu'appui sur bouton et position atteinte (stop) = le delais d'attente avant reprise
// empeche relance moteur indesirable

const bool DOUBLE_CLIC_REINIT = false;  // reinit position home sur v1 si double-clic bouton down (descendre), true ou false



// -------- INIT OBJETS (ne pas modifier) ----------------------------

INA219 INA(ADRESSE_INA);   

simpleBouton boutonUp(BUTON_UP_PIN);      // objet bouton Plus  (UP, monter vers v3)
simpleBouton boutonDown(BUTON_DOWN_PIN);  // objet bouton Moins (DOWN, descendre vers v1)

boutonAction boutonD(BUTON_DOWN_PIN);


Timeout timer_motor; // objet timer pour gestion duree moteur


// --------  VARIABLES GLOBALES (ne pas modifier) ----------------------

// INA
float v_motor  = 0.0;
float mA_motor = 0.0;
float mW_motor = 0.0;
float shuntMv  = 0.0;
float pic_mA   = 0.0;  // stock val max


// etat position shifter
unsigned int state_shifter = 0;  // 0=inconnu, 1=V1, 2=V2, 3=V3

// etat moteur tourne, vrai/faux
bool motor_turn = false;

// etat bouton
bool state_btUp   = false; 
bool state_btDown = false;  
bool btpause      = false;  // pour limiter delais appui
bool blocage      = false;  // empeche appui 2 bt en meme temps

// -------- MAIN ----------------------



void setup()
{
    
    //pinMode(BUTON_UP_PIN, INPUT_PULLUP);    // appuyé = LOW, déja declare dans simpleBouton.h 
    //pinMode(BUTON_DOWN_PIN, INPUT_PULLUP);  // appuyé = LOW, déja declare dans simpleBouton.h 
    boutonD.attacher(simpleClicD, doubleClicD);
    
    
    // set all the motor control pins to outputs
    pinMode(ENA_PIN, OUTPUT);
    pinMode(IN1_PIN, OUTPUT);
    pinMode(IN2_PIN, OUTPUT);
    analogWrite(ENA_PIN, 0);  // PWM off
    
    // Led builtin Uno/Nano
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW); 
    
    // stop moteur a l'initialisation
    stop_motor();   
    
    // Serial console
    Serial.begin(115200);
    
    Serial.println(__FILE__);
    Serial.print("INA219_LIB_VERSION: ");
    Serial.println(INA219_LIB_VERSION);
    
    Wire.begin();          // sda, scl
    Wire.setClock(400000); // fast I2C speed, a désactiver si pose problème
    if (!INA.begin() )
    {
        Serial.println("Could not connect INA219. Fix and Reboot");
    }
    
    // config shunt INA : "A max, R shunt"
    // INA.setMaxCurrentShunt(1, RSHUNT_INA);
    INA.setMaxCurrentShunt(2.5, RSHUNT_INA);     
    
    delay(100);
    
    // postionner moteur en vitesse 1, voir debut du Loop !
    
    
} // endsetup


void loop()
{
    v_motor  = INA.getBusVoltage();
    mA_motor = INA.getCurrent_mA();
    mW_motor = INA.getPower_mW();
    shuntMv  = INA.getShuntVoltage_mV();
    
    //Lecture de la durée d'appui boutons AVANT d'actualiser
    //uint32_t dureeUp   = boutonUp.dureeEnfonce();  
    //uint32_t dureeDown = boutonDown.dureeEnfonce(); 
    boutonUp.actualiser();
    boutonDown.actualiser();
    boutonD.actualiser();     // pour double clic bt.down ! (cf function)
    
    unsigned int mstartmotor; // pour temps a deduire au start motor (cont. intensite)
    if(state_shifter==0) mstartmotor = (TIMEOUT_1to3+MARGE_HOME)-SUP_PIC;
    if(state_shifter>0) mstartmotor = TIMEOUT_STEP-SUP_PIC;
    
    //Gestion du petit malin qui appuie sur les deux boutons en même temps
    if (boutonUp.estEnfonce() && boutonDown.estEnfonce()) blocage = true;
    if (boutonUp.estRelache() && boutonDown.estRelache()) blocage = false;
    if (blocage) return;
    
    
    // Au boot, si aucune position definie (0), on descend a la butee basse sur v1 
    if(state_shifter < 1 && !motor_turn) {
        
        Serial << "to Home v1" << endl;
        timer_motor.start(TIMEOUT_1to3 + MARGE_HOME );  // trajet au plus long + petite marge
        turn_motor_down();   // move motor DOWN
        state_btDown = true; // simuler etat bouton 
    }
    else {
        
        // -- Bouton UP, MONTER vers v3 --
        if (boutonUp.vientDEtreEnfonce() && !motor_turn && state_shifter < 3) {
            Serial << "appui bt UP" << endl;
            state_btUp = true;
            timer_motor.start(TIMEOUT_STEP);
            turn_motor_up(); // move motor UP
            
            state_shifter+=1;
            
            // pas d'instruction delay() ici !!
        }
        
        // -- Bouton DOWN, DESCENDRE vers v1 --
        if (boutonDown.vientDEtreEnfonce() && !motor_turn && state_shifter > 1) {
            Serial << "appui bt DOWN" << endl;
            state_btDown = true;
            timer_motor.start(TIMEOUT_STEP);
            turn_motor_down(); // move motor DOWN
            
            state_shifter-=1;
            
            // pas d'instruction delay() ici !!
        }
    } // endelse
    
    
    if(!timer_motor.time_over() && timer_motor.time_left_ms() % 100 == 0) {
        Serial << timer_motor.time_left_ms() << endl; // debug, decompte timer tous les 100
    }
    
    // controle depassement intensite (en deduisant demarage moteur)
    if (timer_motor.time_left_ms() < mstartmotor && abs(mA_motor) >= MA_MAX) {
        stop_motor();
        timer_motor.expire();  // timer off
        if(state_shifter<1) state_shifter=1; // init sur v1 apres le home
        
        Serial << "STOP mA! mA=" << mA_motor << " state v=" << state_shifter << endl;
        if(state_btUp || state_btDown) {
            btpause      = true;
            state_btUp   = false;
            state_btDown = false;
        }
        // pas d'instruction delay() ici !!
    } // endif timer
    
    // timer termine
    if(timer_motor.time_over()) {
        stop_motor(); 
        if(state_shifter<1) state_shifter=1; // init sur v1 apres le home
        
        //Serial << "STOP timer, state v=" << state_shifter << endl; // debug
        if(state_btUp || state_btDown) {
            btpause      = true;
            state_btUp   = false;
            state_btDown = false;
        }
        // pas d'instruction delay() ici !!
    } // endif timer
    
    // -- pause reprise boutons
    if(btpause) {
        Serial << "pause reprise (" << DELAY_PUSH_BTN << " ms) state_shifter=v" << state_shifter << endl; 
        delay(DELAY_PUSH_BTN); 
        btpause = false; 
    }
    
} // endloop



// ------------- FUNCTIONS ---------------------------


// double-clic bouton down : reinit position home (down to v1) 
void doubleClicD() {
    if(DOUBLE_CLIC_REINIT) {
        // RAZ
        state_btUp    = false; 
        state_btDown  = false;  
        state_shifter = 0;  
        Serial.println("double clic bt down : REINIT HOME v1!"); 
    }
} // endfunc

void simpleClicD() { 
    //Serial.println("simple clic bt down"); // pas utilise
    //
} // endfunc


// stop motor
void stop_motor() {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
    analogWrite(ENA_PIN, 0);  // PWM off 
    motor_turn = false;
    digitalWrite(LED_PIN, LOW); // led nano
} //endfunc

// marche avant (PLUS, UP, MONTER)
bool turn_motor_up() {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
    analogWrite(ENA_PIN, SPEED_MOTOR);  // choix PWM
    motor_turn = true;
    digitalWrite(LED_PIN, HIGH); // led nano
} //endfunc


// marche arriere (MOINS, DOWN, DESCENDRE)
bool turn_motor_down() {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
    analogWrite(ENA_PIN, SPEED_MOTOR);  // choix PWM
    motor_turn = true;
    digitalWrite(LED_PIN, HIGH); // led nano
} //endfunc


