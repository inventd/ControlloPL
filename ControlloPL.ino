#include <Servo.h>

#define NUMERO_DI_74HC165 2
#define DATA_WIDTH NUMERO_DI_74HC165 * 8

#undef INPUT_SERIAL

/*
 * Defiinizione pin_out
 */
#ifdef INPUT_SERIAL
// gestione shift register
#define LOAD_PIN 8
#define ENABLE_PIN 9
#define DATA_PIN 11
#define CLOCK_PIN 12
#else
#define SENSOR_SX1 4
#define SENSOR_SX2 5
#define SENSOR_DX1 6
#define SENSOR_DX2 7
#endif

/*
 * Macro
 */
#define SENS_PL(N, I) byte(I >> (4 * (N - 1)) & 0x0F)
#define GATE_A(I) byte(I & 0x03)
#define GATE_B(I) byte((I >> 2) & 0x03)
#define STATO_A(I) byte(I & 0x0F)
#define STATO_B(I) byte((I >> 4) & 0x0F)

/*
 * Definizione stati
 */
#define GATE_ERRORE 0x0F   // 00001111
#define GATE_LIBERO 0x00   // 00000000
#define GATE_INGRESSO 0x01 // 00000001
#define GATE_OCCUPATO 0x02 // 00000011
#define GATE_USCITA 0x03   // 00000010

// Definizione ingressi
#define SENS_NULL B0000
#define SENS_INGRESSO B0001
#define SENS_USCITA B0010
#define SENS_FULL B0011

unsigned long letturaSensori;
unsigned long oldLetturaSensori;
byte statoPL1;
int contaAssiPL1 = 0;
byte statoPL2;
int contaAssiPL2 = 0;
byte statoPL3;
int contaAssiPL3 = 0;

Servo servoPL1;

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);

  servoPL1.attach(3);

  // Configurazione linee per gestione shift register 74HC165
#ifdef INPUT_SERIAL
  pinMode(LOAD_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN, INPUT);

  digitalWrite(CLOCK_PIN, LOW);
  digitalWrite(LOAD_PIN, HIGH);
#else
  pinMode(SENSOR_SX1, INPUT);
  pinMode(SENSOR_SX2, INPUT);
  pinMode(SENSOR_DX1, INPUT);
  pinMode(SENSOR_DX2, INPUT);
#endif

  letturaSensori = leggiSensoriPL();
  oldLetturaSensori = letturaSensori;
}

void loop()
{
  // letturaSensori = leggiSensoriPL();

  letturaSensori = simulaIngressi(oldLetturaSensori);

  if (letturaSensori != oldLetturaSensori)
  {
    oldLetturaSensori = letturaSensori;
    cicloMacchinaStati(letturaSensori);
    visualizzaStato();
    servoPL1.write((contaAssiPL1 > 0) ? 0 : 180);
  }
}

void cicloMacchinaStati(unsigned long statoIngressi)
{
  statoPL1 = cicloMacchinaPL(statoPL1, SENS_PL(1, statoIngressi), &contaAssiPL1);
  statoPL2 = cicloMacchinaPL(statoPL2, SENS_PL(2, statoIngressi), &contaAssiPL2);
  statoPL3 = cicloMacchinaPL(statoPL3, SENS_PL(3, statoIngressi), &contaAssiPL3);
}

byte cicloMacchinaPL(byte statoAttuale, byte ingressi, int *contaAssi)
{
  // Controllo Gate A
  byte statoGateA = controlloGate(STATO_A(statoAttuale), GATE_A(ingressi), contaAssi);
  // Controllo Gate B
  byte statoGateB = controlloGate(STATO_B(statoAttuale), GATE_B(ingressi), contaAssi);

  return (statoGateA | (statoGateB << 4));
}

byte controlloGate(byte statoGate, byte ingressi, int *contaAssi)
{
  byte statoProssimo = statoGate;

  switch (statoGate)
  {
  case GATE_LIBERO:
    if (ingressi & SENS_FULL)
    {
      statoProssimo = GATE_ERRORE;
      if (ingressi == SENS_INGRESSO)
        statoProssimo = GATE_INGRESSO;
      else if (ingressi == SENS_USCITA)
        statoProssimo = GATE_USCITA;
    }
    break;
  case GATE_INGRESSO:
    if (ingressi != SENS_INGRESSO)
    {
      statoProssimo = GATE_ERRORE;
      if (ingressi == SENS_FULL)
      {
        (*contaAssi)++;
        statoProssimo = GATE_OCCUPATO;
      }
      else if (ingressi == SENS_NULL)
      {
        (*contaAssi)--;
        //((*contaAssi) > 0) ? *contaAssi : 0;
        statoProssimo = GATE_LIBERO;
      }
    }
    break;
  case GATE_OCCUPATO:
    if (ingressi != SENS_FULL)
    {
      statoProssimo = GATE_ERRORE;
      if (ingressi == SENS_INGRESSO)
        statoProssimo = GATE_INGRESSO;
      else if (ingressi == SENS_USCITA)
        statoProssimo = GATE_USCITA;
    }
    break;
  case GATE_USCITA:
    if (ingressi != SENS_USCITA)
    {
      statoProssimo = GATE_ERRORE;
      if (ingressi == SENS_NULL)
        statoProssimo = GATE_LIBERO;
      else if (ingressi == SENS_FULL)
        statoProssimo = GATE_OCCUPATO;
    }
    break;
  }
  return (statoProssimo);
}

unsigned long leggiSensoriPL()
{
  unsigned long bytesVal = 0;
#ifdef INPUT_SERIAL
  long bitVal;

  digitalWrite(ENABLE_PIN, HIGH);
  digitalWrite(LOAD_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(LOAD_PIN, HIGH);
  digitalWrite(ENABLE_PIN, LOW);

  for (int i = 0; i < DATA_WIDTH; i++)
  {
    bitVal = digitalRead(DATA_PIN);
    bytesVal |= (bitVal << ((DATA_WIDTH - 1) - i));

    digitalWrite(CLOCK_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(CLOCK_PIN, LOW);
  }

#else
  long bitValSX1, bitValSX2, bitValDX1, bitValDX2;

  bitValSX1 = digitalRead(SENSOR_SX1);
  bitValSX2 = digitalRead(SENSOR_SX2);
  bitValDX1 = digitalRead(SENSOR_DX1);
  bitValDX2 = digitalRead(SENSOR_DX2);
  bytesVal = bitValSX1 | (bitValSX2 >> 1) | (bitValDX1 >> 2) | (bitValDX2 >> 3);
  delayMicroseconds(85);
#endif

  return (bytesVal);
}

unsigned long simulaIngressi(unsigned long ingressiAttuali)
{
  if (Serial.available() > 0)
  {
    // read the incoming byte:
    int incomingPLnumber = Serial.parseInt();
    if (incomingPLnumber > 0 && incomingPLnumber < 4)
    {
      Serial.print("PL selezionato: ");
      Serial.println(incomingPLnumber, DEC);
      Serial.println("Sequenza di input? (es.: [1000])");
      String statoBit = Serial.readString();
      Serial.print("implementa: ");
      Serial.println(statoBit);
      int provv = 0;
      for (size_t i = 1; i < statoBit.length() - 1; i++)
      {
        provv = (provv << 1) | ((statoBit[i] == '1') ? 0x01 : 0);
      }
      provv <<= 4 * (incomingPLnumber - 1);
      Serial.print("dato elaborato: ");
      Serial.println(provv, BIN);
      switch (incomingPLnumber)
      {
      case 1:
        ingressiAttuali = ingressiAttuali & 0xFF0 | provv;
        break;
      case 2:
        ingressiAttuali = ingressiAttuali & 0xF0F | provv;
        break;
      case 3:
        ingressiAttuali = ingressiAttuali & 0x0FF | provv;
        break;
      }
    }
  }
  return ingressiAttuali;
}

#define FRMT_BIN_8(I) ("000000000000" + String(I, BIN)).substring(String(I, BIN).length())

void visualizzaStato()
{
  Serial.print("Stato ingressi ultima lettura: ");
  Serial.println(FRMT_BIN_8(letturaSensori));
  Serial.print("Conta assi PL [1] [2] [3] ->  [");
  Serial.print(contaAssiPL1);
  Serial.print("]  [");
  Serial.print(contaAssiPL2);
  Serial.print("]  [");
  Serial.print(contaAssiPL3);
  Serial.println("]");
}
