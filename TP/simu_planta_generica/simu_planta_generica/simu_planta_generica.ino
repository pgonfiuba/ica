
#include <Arduino.h>
#include <math.h>
#include <stdlib.h>

// ====================================================================
// Simulador de un sistema dinámico basado en - Wemos D1 mini (ESP8266)
// Integracion: Euler forward, Ts configurable
// Telemetria: configurable, compatible con Serial Plotter
// ====================================================================

// Tiempos
const uint32_t TS_US = 50000;    // Paso de integracion [us]
const uint32_t TOUT_US = 50000;  // Período de publicación de datos [us]

const float U_MIN = -10;
const float U_MAX = 10;
const float Y_MIN = -1000;
const float Y_MAX = 1000;

// ==================================================
// SELECCION DE PLANTA
// Configurar el tipo de sistema descomentando una
// línea a continuación
//
// Se podría sumar respuesta inversa (cero no minimo y polo), e integrador + polo
// ==================================================

#define PLANTA_FOPDT
//#define PLANTA_SOPDT
//#define PLANTA_INTEGRADORA


// ==================================================
// PARAMETROS DEL MODELO SELECCIONADO
// Convencion: A(z) y = B(z) u
// A(z) = 1 + a1*z^-1 + a2*z^-2 ...
// B(z) = b0 + b1*z^-1 + b2*z^-2 ...
// ==================================================

#if defined(PLANTA_FOPDT)

// FOPDT: K=0.8, tau=3s, L=0.4s, Ts=0.05s
// a = exp(-Ts/tau) = 0.983471
// b = K*(1-a) = 0.013223
// Retardo: 8 muestras

const int NA = 1;
const int NB = 1;
const int D = 8;

const float A[NA] = { -0.983471 };
const float B[NB] = { 0.013223 };

#elif defined(PLANTA_SOPDT)

// Segundo orden: reemplazar coeficientes
const int NA = 2;
const int NB = 1;
const int D = 0;

const float A[NA] = { -1.5, 0.56 };
const float B[NB] = { 0.06 };

#elif defined(PLANTA_INTEGRADORA)

const int NA = 1;
const int NB = 1;
const int D = 0;

const float A[NA] = { -1.0 };
const float B[NB] = { 0.01 };

#endif

// ==================================================
// Estados del simulador
// ==================================================
#define MAX_HIST 20
float uHist[MAX_HIST] = { 0 };
float yHist[MAX_HIST] = { 0 };

// ==================================================
// Validaciones en tiempo de compilación
// ==================================================
static_assert(NA <= MAX_HIST, "Orden A excedido");
static_assert(D + NB <= MAX_HIST, "Retardo + B excedido");

// ==================================================
// Inicialización
// ==================================================

void iniciarPlanta() {

  for (int i = 0; i < MAX_HIST; i++) {
    uHist[i] = 0.0;
    yHist[i] = 0.0;
  }
}

// ==================================================
// Ruido de medición
// ==================================================
const float sigma_q = 0.1f;  // Desvio estandar

float ruidoMedicion() {
  // Dos uniformes independientes en (0,1)
  float r1 = random(1, 1000000) / 1000000.0f;
  float r2 = random(1, 1000000) / 1000000.0f;

  // Ruido aproximadamente gaussiano: N(0, sigma_q^2)
  return sigma_q * sqrtf(-2.0f * logf(r1))
         * cosf(2.0f * PI * r2);
}

// ==================================================
// Simulador
// ==================================================

float integrar(float u) {
  float y = 0;

  // Saturacion de entrada
  u = constrain(u, U_MIN, U_MAX);

  // Numerador: entradas actuales y anteriores
  for (int j = 0; j < NB; j++) {
    int idx = D + j;

    if (j == 0 && D == 0)
      y += B[j] * u;
    else
      y += B[j] * uHist[idx - 1];
  }

  // Denominador: salidas anteriores
  for (int i = 0; i < NA; i++) {
    y -= A[i] * yHist[i];
  }

  // Saturacion de salida
  y = constrain(u, Y_MIN, Y_MAX);

  // Actualizar historiales
  for (int i = MAX_HIST - 1; i > 0; i--) {
    uHist[i] = uHist[i - 1];
    yHist[i] = yHist[i - 1];
  }

  uHist[0] = u;
  yHist[0] = y;

  float y_r = y + ruidoMedicion();
  return y_r;
}

// ==================================================
// Recepcion de la acción de control por USB/serie
// Acepta, por ejemplo: 2.5, -3, 0 o u=2.5
// El cambio de mando se aplica en el siguiente paso.
// ==================================================
float u_latched = 0.0f;
char rxBuffer[32];
uint8_t rxIndex = 0;

void leerSerie() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\r' || c == '\n') {
      if (rxIndex > 0) {
        rxBuffer[rxIndex] = '\0';

        char *p = rxBuffer;
        if (p[0] == 'u' || p[0] == 'U') {
          p++;
          if (*p == '=') p++;
        }

        char *endptr;
        float valor = strtof(p, &endptr);

        if (endptr != p && *endptr == '\0') {
          u_latched = constrain(valor, U_MIN, U_MAX);
        }

        rxIndex = 0;
      }
    } else if (c >= 32 && c <= 126) {
      if (rxIndex < sizeof(rxBuffer) - 1) {
        rxBuffer[rxIndex++] = c;
      } else {
        // Descarta una entrada demasiado larga
        rxIndex = 0;
      }
    }
  }
}

// ==================================================
// Telemetria para Arduino Serial Plotter
// Columnas: t,y_con_ruido(k), u(k)
// Separador: tabulador
// ==================================================
void transmitir(float u, float y) {
  Serial.print(micros());
  Serial.print('\t');
  Serial.print(y);
  Serial.print('\t');
  Serial.println(u);
}

// ==================================================
// Setup
// ==================================================

// Variables auxiliares para temporizar
uint32_t nextStep;
uint32_t nextOutput;


void setup() {
  Serial.begin(115200);

  randomSeed(micros());

  iniciarPlanta();
  const uint32_t ahora = micros();
  nextStep = ahora + TS_US;
  nextOutput = ahora + TOUT_US;
}

// ==================================================
// Loop
// ==================================================
float y = 0.0f;  // Salida actual
float u = 0.0f;  // Acción de mando [0-100]

void loop() {
  leerSerie();

  uint32_t ahora = micros();

  // Un paso de integracion por cada periodo programado
  if ((int32_t)(ahora - nextStep) >= 0) {
    nextStep += TS_US;
    y = integrar(u);
  }

  // Telemetria independiente a frecuencia propia
  if ((int32_t)(ahora - nextOutput) >= 0) {
    u = u_latched;
    nextOutput += TOUT_US;
    transmitir(u, y);
  }

  yield();  // Atiende las tareas del ESP8266
}