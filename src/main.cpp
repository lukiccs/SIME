#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_AS5600.h>

static const float PRECNIK_TOCKA =              72.0; // u mm
static const float RASTOJANJE_IZMEDJU_TOCKOVA = 107.2; // u mm
static uint32_t prethodnoVreme = 0;
uint32_t trenutnoVreme = 0;
const float pi = 3.14159;
const float T = 0.01; // 10ms, frekvencija azuriranja od 100Hz

//obavezno promeniti za svaku plocu
#define WIRE1_SDA               2
#define WIRE1_SCL               3
#define WIRE0_SDA               4
#define WIRE0_SCL               5
#define I2C_ADRESS_AS5600       0x36
#define I2C_MASTER_FREQ_HZ      100000
#define LEVI_MOTOR_PIN1         20
#define LEVI_MOTOR_PIN2         21
#define DESNI_MOTOR_PIN1        18
#define DESNI_MOTOR_PIN2        19

float prethodniLeviUgao = 0.0;
float prethodniDesniUgao = 0.0;

//pocetna citanja
uint16_t leviRaw0;
uint16_t desniRaw0;
int32_t dLeviRaw0 = 0;

// LPF
const float alfa = 0.2;
float brzinaLevogLPF = 0.0;
float brzinaDesnogLPF = 0.0;

//ACC limiter
const float maxACC = 200; //rad/s^2
float brzinaLevog0 = 0;
float brzinaDesnog0 = 0;

//definisanje polazaja SIME
float leviPut;
float desniPut;
float X = 0.0;
float Y = 0.0;
float theta = 0.0;

//PI regulacija
float integralLevi = 0;
float integralDesni = 0;
float KpL = 7.0;
float KiL = 1;
float KpD = 7.0;
float KiD = 1;
float pwmLimit = 200;
float Kvl = 20;
float Kvd = 21;

//trapez brzine
float brzina = 0;


Adafruit_AS5600 leviEnkoder;
Adafruit_AS5600 desniEnkoder;


TwoWire Wire1(WIRE1_SDA, WIRE1_SCL);


void resetovanjeEnkodera(){
    leviRaw0 = leviEnkoder.getRawAngle();
    desniRaw0 = desniEnkoder.getRawAngle();
}

void debug(){
    // provera povezanosti levog enkodera
    if(!leviEnkoder.begin(I2C_ADRESS_AS5600, &Wire1)){
        Serial.println("Nije pronadjen levi enkoder");
        while(1){
            delay(100);
        };
    }
  // provera povezanosti desnog enkodera
    if(!desniEnkoder.begin(I2C_ADRESS_AS5600, &Wire)){
        Serial.println("Nije pronadjen desni enkoder");
        while(1){
            delay(100);
        };
    }
  //provera magneta levog enkodera
    if(leviEnkoder.isMagnetDetected()){
        Serial.println("Magnet levog enkodera je detektovan");
    }else {
        Serial.println("Magnet levog enkodera nije detektovan");
    }
    if(leviEnkoder.isAGCminGainOverflow()){
        Serial.println("Magnet levog enkodera je previse jak");
    }
    if(leviEnkoder.isAGCmaxGainOverflow()){
        Serial.println("Magnet levog enkodera je previse slab");
    }
    //provera magneta desnog enkodera
    if(desniEnkoder.isMagnetDetected()){
        Serial.println("Magnet desnog enkodera je detektovan");
    }else {
        Serial.println("Magnet desnog enkodera nije detektovan");
    }
    if(desniEnkoder.isAGCminGainOverflow()){
        Serial.println("Magnet desnog enkodera je previse jak");
    }
    if(desniEnkoder.isAGCmaxGainOverflow()){
        Serial.println("Magnet desnog enkodera je previse slab");
    }
}

float levaStrana(){
    int32_t leviUgaoRaw = (int32_t)leviEnkoder.getRawAngle();
    int32_t dLeviRaw = (leviUgaoRaw - (int32_t)leviRaw0);
    if(dLeviRaw > 2048){dLeviRaw -= 4096;}
    if(dLeviRaw < -2048){dLeviRaw += 4096;}
    leviRaw0 = leviUgaoRaw;

    float dLeviRad = (dLeviRaw * 2*pi) / 4096.0;
    leviPut = dLeviRad * (0.5 * PRECNIK_TOCKA);
    float brzinaLevog = dLeviRad / T;
    //acc max
    if((brzinaLevog - brzinaLevog0) > (maxACC * T)){
        brzinaLevog = brzinaLevog0 + maxACC * T;
    } else if((brzinaLevog - brzinaLevog0) < -(maxACC * T)){
        brzinaLevog = brzinaLevog0 - maxACC * T;
    }
    brzinaLevog0 = brzinaLevog;
    // LPF
    brzinaLevogLPF = brzinaLevogLPF + alfa*(brzinaLevog - brzinaLevogLPF);

    return -brzinaLevogLPF;
}

float desnaStrana(){
    int32_t desniUgaoRaw = (int32_t)desniEnkoder.getRawAngle();
    int32_t dDesniRaw = (desniUgaoRaw - (int32_t)desniRaw0);
    if(dDesniRaw > 2048){dDesniRaw -= 4096;}
    if(dDesniRaw < -2048){dDesniRaw += 4096;}
    desniRaw0 = desniUgaoRaw;

    float dDesniRad = (dDesniRaw * 2*pi) / 4096.0;
    desniPut = dDesniRad * (0.5 * PRECNIK_TOCKA);
    float brzinaDesnog = dDesniRad / T;
    //acc max
    if((brzinaDesnog - brzinaDesnog0) > (maxACC * T)){
        brzinaDesnog = brzinaDesnog0 + maxACC * T;
    } else if((brzinaDesnog - brzinaDesnog0) < -(maxACC * T)){
        brzinaDesnog = brzinaDesnog0 - maxACC * T;
    }
    brzinaDesnog0 = brzinaDesnog;
    //LPF
    brzinaDesnogLPF = brzinaDesnogLPF + alfa*(brzinaDesnog - brzinaDesnogLPF);
    return brzinaDesnogLPF;
}

//promena orijentacije sime
float promenaTheta(float leviPut, float desniPut){
    float deltaTheta = (desniPut - leviPut) / RASTOJANJE_IZMEDJU_TOCKOVA;
    return deltaTheta;
}
//promena polozaja sime
float promenaX(float leviPut, float desniPut, float theta){
    float deltaX = ((leviPut + desniPut) / 2.0) * cos(theta);
    return deltaX;
}

float promenaY(float leviPut, float desniPut, float theta){
    float deltaY = ((leviPut + desniPut) / 2.0) * sin(theta);
    return deltaY;
}

float trapezBrzine(){
    if(brzina <= 7){
        brzina += maxACC * T;
    }
    else{
        brzina -= maxACC * T;
    }
    return brzina;
}

//PI regulacija leve strane
float leviPI(float brzinaLevogZeljena, float brzinaLevogStvarna){
    //greska
    float E = brzinaLevogZeljena - brzinaLevogStvarna;
    //proporcionalni deo
    float P = KpL * E;
    //novi integralni
    integralLevi += KiL * E * T;
    //feedforward deo
    float FF = brzinaLevogZeljena * 25.5;
    //upravljacka velicina
    float U = FF + (P + integralLevi) * 21.0;
    float Usat = constrain(U, -255.0, 255.0);
    
    integralLevi += 0.1 * (Usat - U) * T;//Antiwindup
    
    return constrain(U, -255.0, 255.0);
}

//PI regulacija desni strane
float desniPI(float brzinaDesnogZeljena, float brzinaDesnogStvarna){
    //greska
    float E = brzinaDesnogZeljena - brzinaDesnogStvarna;
    //proporcionalni deo
    float P = KpD * E;
    //novi integralni
    integralDesni += KiD * E * T;
    //feedforward deo
    float FF = brzinaDesnogZeljena * 25.5; //Kv za FF
    //upravljacka velicina
    // float U = FF + P * 21.0;
    float U = FF + (P + integralDesni) * 21.0;
    float Usat = constrain(U, -255.0, 255.0);
    
    integralDesni += 0.1 * (Usat - U) * T;//Antiwindup

    return Usat;
}

//setovanje PWM na motore
void setLevi(float PWMLevi){
    if(PWMLevi > 255.0){
        PWMLevi = 255.0;
    }
    else if(PWMLevi < -255.0){
        PWMLevi = -255.0;
    }
    if(PWMLevi >= 0){
        analogWrite(LEVI_MOTOR_PIN2, PWMLevi);
        digitalWrite(LEVI_MOTOR_PIN1, LOW);
    }
    else{
        analogWrite(LEVI_MOTOR_PIN1, -PWMLevi);
        digitalWrite(LEVI_MOTOR_PIN2, LOW);
    }
}

//setovanje PWM na motore
void setDesni(float PWMDesni){
    if(PWMDesni > 255.0){
        PWMDesni = 255.0;
    }
    else if(PWMDesni < -255.0){
        PWMDesni = -255.0;
    }
    if(PWMDesni >= 0){
        analogWrite(DESNI_MOTOR_PIN2, PWMDesni);
        digitalWrite(DESNI_MOTOR_PIN1, LOW);
    }
    else{
        analogWrite(DESNI_MOTOR_PIN1, -PWMDesni);
        digitalWrite(DESNI_MOTOR_PIN2, LOW);
    }
}


void setup() {
    Serial.begin(115200);
    delay(10);
    Serial.print("Radi serial");

    // I2C0
    Wire.begin();
    // I2C1
    Wire1.begin();

    Serial.println("Pokretanje sistema...");
    Serial.println("Debug...");
    debug();
    resetovanjeEnkodera();

    pinMode(LEVI_MOTOR_PIN1, OUTPUT);
    pinMode(LEVI_MOTOR_PIN2, OUTPUT);
    pinMode(DESNI_MOTOR_PIN1, OUTPUT);
    pinMode(DESNI_MOTOR_PIN2, OUTPUT);
    delay(1000);
    Serial.println("Gotov Setup");
}

void loop() {
    //100Hz azuriranje podataka, tj. svake 10ms(zbog PI regulacije, onaj ko bude radio neka izmenu frekvenciju uzoraka)
    trenutnoVreme = millis();
    float dt = trenutnoVreme - prethodnoVreme;
    if(trenutnoVreme - prethodnoVreme < (T*1000)) return;
    
    float wL = levaStrana();
    float wD = desnaStrana();
    float wZeljeno = 8;
    
    float PWMLevi = leviPI(wZeljeno, wL);
    float PWMDesni = desniPI(wZeljeno, wD);
    // float PWMLevi = leviPI(wZeljeno, levaStrana());
    // float PWMDesni = desniPI(wZeljeno, desnaStrana());
    
    setLevi(PWMLevi);
    setDesni(PWMDesni);
    
    // leviPut = (wL* T)*(0.5*PRECNIK_TOCKA);//rad/sonda mnozim sa T da dobijem rad onda da dobijem luk
    // desniPut = (wD* T)*(0.5*PRECNIK_TOCKA);
    
    theta += promenaTheta(leviPut, desniPut);
    X += promenaX(leviPut, desniPut, theta);
    Y += promenaY(leviPut, desniPut, theta);
    
    //da theta ne bi raslo ili smanjivalo unedogled
    if(theta > pi) theta -= 2*pi;
    else if(theta < -pi) theta += 2*pi;
    
    // ispis podataka
    // Serial.print("X: ");
    // Serial.print(X);
    // Serial.print(" Y: "); Serial.print(Y);
    // Serial.print(" theta: "); Serial.println(theta * 180.0 / pi);
    Serial.print(dt);
    Serial.print(" ");
    
    // Serial.print("Desni ugao raw: ");
    // Serial.print(desniEnkoder.getRawAngle());
    // Serial.print("T(ms): ");Serial.println(trenutnoVreme - prethodnoVreme);
    Serial.print(" ");
    Serial.print(PWMLevi);
    Serial.print(" ");
    Serial.print(wL);
    Serial.print(" ");
    Serial.print(PWMDesni);
    Serial.print(" ");
    Serial.println(wD);

    prethodnoVreme = trenutnoVreme;
    //DODATI float leviPut = (dLeviRaw / 4096.0) * 2*pi * (0.5*PRECNIK_TOCKA); da bih izbegao leviPut = (wL * T) * (0.5 * PRECNIK_TOCKA); da bi odometrija bila stabilnija
    //<333
}