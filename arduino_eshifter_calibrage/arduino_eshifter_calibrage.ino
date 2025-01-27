/******************************************************************
Created with PROGRAMINO IDE for Arduino - 18.01.2025 | 13:38:17
Update      :
Author      : Chris74 pour Kiki85 (Cyclurba)
Description : e-shifter ebike Vanmoof s5, Arduino tests

forum Cyclurba : https://cyclurba.fr/forum/757849/programme-arduino-schifter.html?discussionID=31918

INA219 :
https://how2electronics.com/how-to-use-ina219-dc-current-sensor-module-with-arduino/
https://wiki.dfrobot.com/Gravity:%20I2C%20Digital%20Wattmeter%20SKU:%20SEN0291

L298N:
https://www.robotique.tech/tutoriel/commander-un-moteur-par-arduino-et-le-module-l298n/

conso montage a vide (Arduino nano + l298n + ina219) sous 8V = ~29mA / 230mW

******************************************************************/


// -------- INCLUDE -------------------------------


#include <Wire.h>          // I2C
#include "INA219.h"        // from Rob.Tillaart, voir https://github.com/RobTillaart/INA219


// simplification envois Serial et screen serial, expl: Serial << "A:" << var << endl;
#include <Streaming.h>     // Peter Polidoro, https://github.com/janelia-arduino/Streaming    

#include <RunningMedian.h> // statistiques, https://github.com/RobTillaart/RunningMedian

// duree d'appui bouton et gestion bouton
// librairie du membre bricoleau, voir https://forum.arduino.cc/t/partage-boutons-poussoirs/361656
#include "simpleBouton.h"  


// -------- ARDUINO PIN, selon schema joint  --------------

// butons
const int BUTON_P_PIN = 3;  // bouton poussoir Plus (UP, Monter), interupt 3 possible  
const int BUTON_M_PIN = 2;  // bouton poussoir Moins (DOWN, Descendre), interupt 2 possible

// L298N, motor driver
const int ENA_PIN = 9;   // enable A, PWM
const int IN1_PIN = 8;   // in1
const int IN2_PIN = 7;   // in2 

// LED interne Uno/Nano
const int LED_PIN = 13;  // LED_BUILTIN


// INA219 curent sensor  
// I2C PIN SDA A4, SCL A5


// --------  CONSTANTES SYSTEME, MODIFIABLES ------------------------

// 0 = mode mimimum, juste pour vérifier les valeurs du wattmètre INA
// 1 = mode test boutons P et M avec mouvement moteur et infos ms
#define CHOIX_PROG 1 

#define ADRESSE_INA 0x40     // adresse I2C INA219, generalement 0x40 (DFRobot=0x45) 
#define RSHUNT_INA 0.100     // valeur shunt, generalement R100 = 0.100 Ohm  (DFRobot=0.010) 

const int SPEED_MOTOR = 255; // speed moteur par defaut, 0 to 255 max (PWM 8bit)

const int MA_RUN = 360;      // mA, conso en utilisation (sous ~6.5V au moteur)
const int MA_MARGE = 200;    // mA, + marge de butee a pas depasser pour securite (stale = ? mA sous 8V)
const int MA_MAX = MA_RUN + MA_MARGE; 

// -- ces parametres influencent sur le controle intensite max moteur
const int SUP_PIC = 30;   // filtrer les premieres ms du pic demarage moteur, env. 30-50=Ok ou 0 pour desactive
const int MIN_STAT = 20;  // 20mA minimum pour etre affiche en direct et ajoute aux stats (laisser comme ca !)


const int TIMEOUT_1to3 = 1500;             // ms, temps pour passer de la vitesse 1 à la vitesse 3 (butee basse a haute)
const int TIMEOUT_1to2 = TIMEOUT_1to3/2;   // ms, en principe TIMEOUT_1to3 / 2
const int TIMEOUT_2to3 = TIMEOUT_1to3/2;   // ms, en principe TIMEOUT_1to3 / 2
// a voir si 3 vers 2 et 2 vers 1 dans l'autre sens sont semblables ...

const int DELAY_REPRISE_BTN = 1200; // ms, lorsqu'appui sur bouton et position atteinte (stop) = le delais d'attente avant reprise
// empeche reprise indesirable




// --------  VARIABLES GLOBALES (ne pas modifier) ----------------------

// INA
float v_motor  = 0.0;
float mA_motor = 0.0;
float mW_motor = 0.0;
float shuntMv  = 0.0;
float pic_mA   = 0.0;  // stock val max


// etat position shifter
byte state_shifter = 0;  // 0=inconnu, 1=V1, 2=V2, 3=V3

// etat bouton
bool state_btP; 
bool state_btM;  

// -------- INIT OBJETS ----------------------------

INA219 INA(ADRESSE_INA);   

RunningMedian samples = RunningMedian(100);

simpleBouton boutonP(BUTON_P_PIN);  // objet bouton Plus  (monter)
simpleBouton boutonM(BUTON_M_PIN);  // objet bouton Moins (descendre)


// -------- MAIN ----------------------

void setup()
{
    // boutons montes sans resistances = input_pullup  
    //pinMode(BUTON_P_PIN, INPUT_PULLUP);  // appuyé = LOW ! (HIGH relaché)
    pinMode(BUTON_M_PIN, INPUT_PULLUP);  // appuyé = LOW !
    
    // set all the motor control pins to outputs
    pinMode(ENA_PIN, OUTPUT);
    pinMode(IN1_PIN, OUTPUT);
    pinMode(IN2_PIN, OUTPUT);
    analogWrite(ENA_PIN, 255);  // PWM max
    
    // Led builtin Uno/Nano
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW); 
    
    // stopper moteur a l'initialisation
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
    
    delay(1000);
    
    // a faire, postionner moteur en vitesse 1
    
} // endsetup


void loop()
{
    v_motor  = INA.getBusVoltage();
    mA_motor = INA.getCurrent_mA();
    mW_motor = INA.getPower_mW();
    shuntMv  = INA.getShuntVoltage_mV();
    
    //Lecture de la durée d'appui boutons AVANT d'actualiser
    uint32_t dureeP = boutonP.dureeEnfonce();  
    uint32_t dureeM = boutonM.dureeEnfonce(); 
    boutonP.actualiser();
    boutonM.actualiser();
    static long dureeP_temp;
    static long dureeM_temp;
    
    
    #if CHOIX_PROG == 0 
        static uint32_t oldtime = millis(); // static ! timer sortie serial prog 0
        
        uint32_t check_t = millis() - oldtime;
        if (check_t >= 1000) {
            oldtime = millis(); // update timer loop
            Serial << "V=" << v_motor << "  mA=" << mA_motor << "  mW=" << mW_motor << endl;
        }
        
        // par defaut le moteur est stoppe
        stop_motor();   
        
    #endif  // endif choix prog 0
    
    
    
    #if CHOIX_PROG == 1  
        
        // -- Bouton Plus, MONTER vers v3 --
        // -----------------
        if (boutonP.vientDEtreEnfonce())
        {
            Serial << "appui bt P" << endl;
            state_btP = true;
            digitalWrite(LED_PIN, HIGH); 
            pic_mA = 0.0;     // raz var pic max
            samples.clear();  // raz sanples stats
        }
        if (boutonP.vientDEtreRelache())
        {
            if(dureeP_temp > 0) {
                Serial << "*** relache P, duree totale dir v3 " << dureeP_temp << " ms"<< endl;
                dureeP_temp = 0;
            }
            else Serial << "*** relache P, duree totale dir v3 " << dureeP << " ms"<< endl;
            
            Serial << "*** moyenne mA=" << samples.getAverage() << " median=" << samples.getMedian() << endl;
            Serial << "*** max mA=" << samples.getHighest() << " pic mA=" << pic_mA << " nb.samples=" << samples.getCount() << endl;
            
            stop_motor();
            state_btP = false;
            digitalWrite(LED_PIN, LOW); 
        }
        
        // tant que le bt P est maintenu enfonce
        if(state_btP) {
            // -- moteur monte direction vitesse 3
            turn_motor_forward(SPEED_MOTOR); // speed moteur a tester ...  
            
            
            // on filtre les xx premieres ms (start moteur = pic)
            if(dureeP > SUP_PIC) {
                
                // compare (en val absolu) et stock val max
                if(abs(mA_motor) > pic_mA) pic_mA = abs(mA_motor); 
                
                // on ajoute aux stat seulemnt si > min_stat
                if(abs(mA_motor) > MIN_STAT) {
                    samples.add(abs(mA_motor));  // ajouter echantillon aux stat
                    Serial << "dir v3, mA=" << mA_motor << "  V=" << v_motor << " pic mA=" << pic_mA <<  "  ms=" << dureeP  << endl;
                }
                
                
                // STOP si > CONSTANTES (abs, =valeur en abolu, negatif=positif)
                if(abs(mA_motor) >= MA_MAX || dureeP >= TIMEOUT_1to3) {
                    
                    stop_motor();  // si butée reculer un peu, a voir 
                    dureeP_temp   = dureeP;
                    state_btP     = false;
                    state_shifter = 3; // butee vitesse 3
                    digitalWrite(LED_PIN, LOW); 
                    
                    if(dureeP >= TIMEOUT_1to3) {
                        Serial << "*** STOP timing MAX ! ms=" << dureeP << endl;
                    }
                    if(abs(mA_motor) >= MA_MAX) {
                        Serial << "*** STOP  mA MAX ! mA=" << mA_motor << " state_shifter = " << state_shifter << endl;
                    }
                    
                    delay(DELAY_REPRISE_BTN);  // pause bouton, empeche reprise trop rapide
                    
                }
                
            } // endif filtre les start moteur
        } // endif state_btp
        
        
        // -- Bouton Moins, DESCENDRE vers v1 --
        // ------------------
        if (boutonM.vientDEtreEnfonce())
        {
            Serial << "appui bt M" << endl;
            state_btM = true;
            digitalWrite(LED_PIN, HIGH); 
            pic_mA = 0.0;     // raz var pic max
            samples.clear();  // raz sanple stat
        }
        if (boutonM.vientDEtreRelache())
        {
            if(dureeM_temp > 0) {
                Serial << "*** relache M, duree totale dir v1 " << dureeM_temp << " ms"<< endl;
                dureeM_temp = 0;
            }
            else Serial << "*** relache M, duree totale dir v1 " << dureeM << " ms"<< endl;
            
            Serial << "*** moyenne mA=" << samples.getAverage() << " median=" << samples.getMedian() << endl;
            Serial << "*** max mA=" << samples.getHighest() << " pic mA=" << pic_mA << " nb.samples=" << samples.getCount() << endl;
            
            stop_motor();
            state_btM = false;
            digitalWrite(LED_PIN, LOW); 
        }
        // tant que le bt M est maintenu enfonce
        if(state_btM) {
            // -- moteur descend direction vitesse 1
            turn_motor_backward(SPEED_MOTOR); // speed moteur a tester ... 
            
            // on filtre les xx premieres ms (start moteur = pic)
            if(dureeM > SUP_PIC) {
                
                // compare (en val absolu) et stock val max 
                if(abs(mA_motor) > pic_mA) pic_mA = abs(mA_motor); 
                
                // on ajoute aux stat seulemnt si > min_stat
                if(abs(mA_motor) > MIN_STAT) {
                    samples.add(abs(mA_motor));  // ajouter echantillon aux stat
                    Serial << "dir v1, mA=" << mA_motor << "  V=" << v_motor << " pic mA=" << pic_mA <<  "  ms=" << dureeM  << endl;
                }
                
                
                // STOP si > CONSTANTES (abs, =valeur en abolu, negatif=positif)
                if(abs(mA_motor) >= MA_MAX || dureeM >= TIMEOUT_1to3) {
                    
                    stop_motor();  // si butée avancer un peu, a voir 
                    dureeM_temp   = dureeM;
                    state_btM     = false;
                    state_shifter = 1; // butee vitesse 1
                    digitalWrite(LED_PIN, LOW); 
                    
                    if(dureeM >= TIMEOUT_1to3) {
                        Serial << "*** STOP timing MAX ! ms=" << dureeM << endl;
                    }
                    if(abs(mA_motor) >= MA_MAX) {
                        Serial << "*** STOP  mA MAX ! mA=" << mA_motor << " state_shifter = " << state_shifter << endl;
                    }
                    
                    delay(DELAY_REPRISE_BTN);  // pause bouton, empeche reprise trop rapide
                }
                
            } // endif filtre les start moteur
        } // endif state_btM
        
        
    #endif  // endif choix prog 1   
    
    
    //Serial.print(INA.getMathOverflowFlag());
    //Serial.print(INA.getConversionFlag());
    
    
} // endloop



// ------------- FUNCTIONS ---------------------------


// stop motor
void stop_motor() {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
    analogWrite(ENA_PIN, 0);  // PWM off 
} //endfunc


// marche avant
void turn_motor_forward(int pwm) {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
    analogWrite(ENA_PIN, pwm);  // choix PWM
} //endfunc


// marche arriere
void turn_motor_backward(int pwm) {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
    analogWrite(ENA_PIN, pwm);  // choix PWM
} //endfunc

