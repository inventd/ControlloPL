#include <Arduino.h>
#include <Servo.h>

#define NUMERO_DI_74HC165 2
#define DATA_WIDTH NUMERO_DI_74HC165 * 8

#define NUMERO_MAX_PL 3 // Numero max di passaggi a livello gestibili

#undef INPUT_SERIAL

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
#define SENS_PL(N, I) byte(I >> (4 * N) & 0x0F)
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
#define SENS_NULL 0x00     // 0000
#define SENS_INGRESSO 0x01 // 0001
#define SENS_USCITA 0x02   // 0010
#define SENS_FULL 0x03     // 0011

typedef struct {
  byte statoPL;
  int contaAssi;
  Servo servoPL;
  uint8_t semaforo;
} PassaggioLivello;

unsigned long letturaSensori;
unsigned long oldLetturaSensori;

PassaggioLivello passaggioLivello[NUMERO_MAX_PL];

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);

  passaggioLivello[0].contaAssi = 0;
  passaggioLivello[0].statoPL = 0;
  passaggioLivello[0].servoPL.attach(SERVO_PL1);
  passaggioLivello[0].semaforo = SEMAFORO_PL1;

  passaggioLivello[1].contaAssi = 0;
  passaggioLivello[1].statoPL = 0;
  passaggioLivello[1].servoPL.attach(SERVO_PL2);
  passaggioLivello[1].semaforo = SEMAFORO_PL2;

  passaggioLivello[2].contaAssi = 0;
  passaggioLivello[2].statoPL = 0;
  passaggioLivello[2].servoPL.attach(SERVO_PL3);
  passaggioLivello[2].semaforo = SEMAFORO_PL3;

  for (size_t i = 0; i < NUMERO_MAX_PL; i++)
  {
    pinMode(passaggioLivello[i].semaforo, OUTPUT);
  }
  
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
  letturaSensori = leggiSensoriPL();

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
  for (size_t i = 0; i < NUMERO_MAX_PL; i++)
  {
    passaggioLivello[i].statoPL =
      cicloMacchinaPL(
        passaggioLivello[i].statoPL,
        SENS_PL(i, statoIngressi),
        &(passaggioLivello[i].contaAssi)
      );
  }
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
  for (size_t i = 0; i < NUMERO_MAX_PL; i++)
  {
    if (passaggioLivello[i].contaAssi > 0)
      passaggioLivello[i].servoPL.write(180);
    else
      passaggioLivello[i].servoPL.write(90);

    if ((SENS_PL(i, letturaSensori)) || (passaggioLivello[i].contaAssi > 0))
      digitalWrite(passaggioLivello[i].semaforo, HIGH);
    else
      digitalWrite(passaggioLivello[i].semaforo, LOW);
  }
}

#define FRMT_BIN_8(I) ("000000000000" + String(I, BIN)).substring(String(I, BIN).length())

void visualizzaStato()
{
  Serial.print("Stato ingressi ultima lettura: ");
  Serial.println(FRMT_BIN_8(letturaSensori));
  Serial.print("Conta assi PL [1] [2] [3] -> ");
  for (size_t i = 0; i < NUMERO_MAX_PL; i++)
  {
    Serial.print(" [");
    Serial.print(passaggioLivello[i].contaAssi);
    Serial.print("]");
  }
  Serial.println();
}
