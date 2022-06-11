#include <Servo.h>

#define NUMERO_DI_74HC165 2
#define DATA_WIDTH NUMERO_DI_74HC165 * 8

#undef INPUT_SERIAL
#undef SIMULATORE
#undef SERVO360

/*
 * Defiinizione pin_out
 */
#ifdef INPUT_SERIAL
// gestione shift register
#define LOAD_PIN (uint8_t)8U
#define ENABLE_PIN (uint8_t)9U
#define DATA_PIN (uint8_t)11U
#define CLOCK_PIN (uint8_t)12U
#else
#define SENSOR_SX1 (uint8_t)14U
#define SENSOR_SX2 (uint8_t)15U
#define SENSOR_DX1 (uint8_t)16U
#define SENSOR_DX2 (uint8_t)17U
#endif

#define SERVO_PL1 3
#define SEMAFORO_PL1 LED_BUILTIN
#define SERVO_PL2 5
#define SEMAFORO_PL2 7
#define SERVO_PL3 6
#define SEMAFORO_PL3 10

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

// stati sbarra
#define SBARRA_ABBASSATA 0
#define SBARRA_ALZATA 1

unsigned long letturaSensori;
unsigned long oldLetturaSensori;
byte statoPL1;
byte statoSbarraPL1 = SBARRA_ALZATA;
int contaAssiPL1 = 0;
byte statoPL2;
byte statoSbarraPL2 = SBARRA_ALZATA;
int contaAssiPL2 = 0;
byte statoPL3;
byte statoSbarraPL3 = SBARRA_ALZATA;
int contaAssiPL3 = 0;

Servo servoPL1;
Servo servoPL2;
Servo servoPL3;

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);

  servoPL1.attach(SERVO_PL1);
  pinMode(SEMAFORO_PL1, OUTPUT);
  servoPL2.attach(SERVO_PL2);
  pinMode(SEMAFORO_PL2, OUTPUT);
  servoPL3.attach(SERVO_PL3);
  pinMode(SEMAFORO_PL3, OUTPUT);

  // Configurazione linee per gestione shift register 74HC165
#ifdef INPUT_SERIAL
  pinMode(LOAD_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN, INPUT);

  digitalWrite(CLOCK_PIN, LOW);
  digitalWrite(LOAD_PIN, HIGH);
#else
  pinMode(SENSOR_SX1, INPUT_PULLUP);
  pinMode(SENSOR_SX2, INPUT_PULLUP);
  pinMode(SENSOR_DX1, INPUT_PULLUP);
  pinMode(SENSOR_DX2, INPUT_PULLUP);
#endif

  letturaSensori = leggiSensoriPL();
  oldLetturaSensori = letturaSensori;
}

void loop()
{
#ifdef SIMULATORE
  letturaSensori = simulaIngressi(oldLetturaSensori);
#else
  letturaSensori = leggiSensoriPL();
  // Serial.print("x");
#endif

  if (letturaSensori != oldLetturaSensori)
  {
    oldLetturaSensori = letturaSensori;
    cicloMacchinaStati(letturaSensori);
    movimentaPL();
    visualizzaStato();
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
        statoProssimo = GATE_LIBERO;
    }
    break;
  case GATE_OCCUPATO:
    if (ingressi != SENS_FULL)
    {
      statoProssimo = GATE_ERRORE;
      if (ingressi == SENS_INGRESSO)
      {
        (*contaAssi)--;
        (*contaAssi) = ((*contaAssi) > 0) ? *contaAssi : 0;
        statoProssimo = GATE_INGRESSO;
      }
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

  bitValSX1 = !digitalRead(SENSOR_SX1);
  bitValSX2 = !digitalRead(SENSOR_SX2);
  bitValDX1 = !digitalRead(SENSOR_DX1);
  bitValDX2 = !digitalRead(SENSOR_DX2);
  bytesVal = bitValSX1 | (bitValSX2 << 1) | (bitValDX1 << 2) | (bitValDX2 << 3);
  delayMicroseconds(85);
#endif

  return (bytesVal);
}

void movimentaPL()
{
  /* Gestione sbarre */
  if (contaAssiPL1 > 0)
  {
    // Abbassa sbarra
#ifdef SERVO360
    if (statoSbarraPL1 == SBARRA_ALZATA)
    {
      servoPL1.write(70);
      delay(300);
      servoPL1.write(90);
      statoSbarraPL1 = SBARRA_ABBASSATA;
    }
#else
    servoPL1.write(180);
#endif
  }
  else
  {
    /* Alza la sbarra */
#ifdef SERVO360
    if (statoSbarraPL1 == SBARRA_ABBASSATA)
    {
      servoPL1.write(110);
      delay(290);
      servoPL1.write(90);
      statoSbarraPL1 = SBARRA_ALZATA;
    }
#else
    servoPL1.write(90);
#endif
  }
  /* Gestione semafori */
  if ((SENS_PL(1, letturaSensori)) || (contaAssiPL1 > 0))
    digitalWrite(SEMAFORO_PL1, HIGH);
  else
    digitalWrite(SEMAFORO_PL1, LOW);
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
