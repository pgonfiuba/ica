# Simulador de plantas para prácticas de control (HIL)

## 1. Descripción

Este proyecto implementa en un Wemos/ESP8266 un simulador de plantas
dinámicas para realizar prácticas de control en configuración
**Hardware-in-the-Loop (HIL)**.

El microcontrolador ejecuta un modelo dinámico discreto. Desde una
computadora se envía la señal de control (u) y se recibe la salida
simulada (y), junto con una marca de tiempo generada por el reloj del
Wemos.

La interfaz de uso es común a todos los modelos:

``` cpp
float y = integrar(float u);
```

La función recibe la entrada actual y devuelve la salida actual. Los
valores anteriores de entrada y salida se conservan internamente en los
historiales del simulador.

Cada Wemos puede compilarse con un modelo diferente, seleccionado
mediante directivas `#define`. De esta manera, el alumno puede trabajar
con una planta sin conocer necesariamente su modelo ni sus parámetros.

## 2. Arquitectura general

El funcionamiento se puede resumir en el siguiente ciclo:

1.  La computadora envía un valor de entrada (u) por el puerto serie.
2.  El Wemos recibe el valor y lo conserva en `u_latched`.
3.  Cada `TS_US` microsegundos se ejecuta el modelo discreto mediante
    `integrar(u)`.
4.  Cada `TOUT_US` microsegundos se transmite una trama de telemetría.
5.  La computadora registra las muestras y reconstruye el tiempo usando
    las marcas de tiempo del Wemos.

La entrada y la salida se limitan mediante saturaciones configurables.

## 3. Configuración temporal y rangos

En el firmware se definen, entre otros, los siguientes parámetros:

``` cpp
// Tiempos
const uint32_t TS_US   = 50000;  // Paso de integración [us]
const uint32_t TOUT_US = 50000;  // Período de publicación [us]

const float U_MIN = -10;
const float U_MAX =  10;
const float Y_MIN = -1000;
const float Y_MAX =  1000;
```

### `TS_US`: período de integración

Es el período con el que se actualiza el estado de la planta simulada:

T_s = TS_US * 1E6 [s]

**Los coeficientes del modelo deben corresponder exactamente a este
período de muestreo.** La planta se implementa directamente en tiempo
discreto: no se discretiza automáticamente dentro del Wemos.

Por ejemplo, si `TS_US = 50000`, el modelo debe haberse discretizado con
(T_s=0.05 s). Si se modifica `TS_US`, en general hay que volver a
discretizar la planta y actualizar sus coeficientes.

### `TOUT_US`: período de publicación

Determina cada cuánto se envía una trama de telemetría por el puerto
serie. No define el paso de integración del modelo.

Puede coincidir con `TS_US` o ser un múltiplo de este. Si es mayor, se
publican menos muestras que las que calcula internamente el simulador.

### Límites de entrada y salida

-   `U_MIN` y `U_MAX`: saturación de la entrada recibida desde la
    computadora.
-   `Y_MIN` y `Y_MAX`: saturación de la salida simulada.

Los límites deben ser coherentes con las unidades elegidas para cada
planta. Por ejemplo, pueden representar voltios, porcentaje de apertura,
temperatura, nivel u otra magnitud.

## 4. Modelo dinámico discreto

El simulador utiliza una ecuación en diferencias general:

y[k] = -Sum_{i=1}^{n_a} a_i y[k-i]
+ Sum_{j=0}^{n_b} b_j u[k-d-j] 

En esta convención:

-   El coeficiente principal del denominador está normalizado a uno.
-   `A[]` contiene los coeficientes (a_1,...,a_{n_a}),
    **sin** el término inicial 1.
-   `B[]` contiene (b_0,...,b_{n_b}).
-   `D` es el retardo puro en muestras.
-   `NA` es la cantidad de coeficientes almacenados en `A[]`.
-   `NB` es la cantidad de coeficientes almacenados en `B[]`.

El código implementa la ecuación en diferencias usando los valores
actuales y los historiales de entrada y salida.

### Historiales

Se reservan dos vectores:

``` cpp
#define MAX_HIST 20

float uHist[MAX_HIST];
float yHist[MAX_HIST];
```

-   `uHist` almacena las entradas anteriores.
-   `yHist` almacena las salidas anteriores.

El índice cero representa la muestra anterior más reciente. Los índices
mayores representan muestras progresivamente más antiguas.

La memoria es fija y no se reserva dinámicamente. Por eso, **todos los
órdenes y retardos deben respetar `MAX_HIST`**. Para la estructura de
coeficientes indicada, deben cumplirse:

``` cpp
static_assert(NA <= MAX_HIST, "Orden A excedido");
static_assert(D + NB <= MAX_HIST, "Retardo + B excedido");
```

Estas verificaciones evitan que el cálculo intente leer fuera de los
vectores.

## 5. Selección de una planta

El modelo que se compila se selecciona en el bloque de definiciones:

``` cpp
//#define PLANTA_FOPDT
//#define PLANTA_SOPDT
//#define PLANTA_INTEGRADORA

#define PLANTA_FOPDT
```

Para cambiar de planta, se comenta la definición activa y se descomenta
la deseada. Debe quedar seleccionada una sola alternativa.

Cada alternativa define sus propios `NA`, `NB`, `D`, `A[]` y `B[]`. La
función `integrar()` no necesita modificarse.

## 6. Cómo agregar un modelo nuevo

Para incorporar una planta:

1.  Obtener o identificar su función de transferencia continua.
2.  Discretizarla para el período (T_s) definido por `TS_US`.
3.  Expresar la transferencia discreta en la convención
    (G(z)=B(z)z\^{-d}/A(z)) utilizada por el código.
4.  Normalizar el denominador para que el coeficiente principal sea uno.
5.  Separar el retardo entero `D` de los coeficientes del numerador.
6.  Verificar que `NA <= MAX_HIST` y `D + NB <= MAX_HIST`.
7.  Agregar una nueva alternativa `#elif defined(PLANTA_NOMBRE)` en el
    bloque de modelos.
8.  Seleccionar esa alternativa mediante `#define PLANTA_NOMBRE` y
    compilar.

Ejemplo de estructura:

``` cpp
#elif defined(PLANTA_NUEVA)

const int NA = ...;
const int NB = ...;
const int D  = ...;

const float A[NA] = { ... };
const float B[NB] = { ... };
```

Los puntos suspensivos deben reemplazarse por los valores calculados
para el modelo elegido.

### Importante: la planta debe estar discretizada

El Wemos **no recibe parámetros continuos** como (K), (`tau`) o
(L). Recibe directamente los coeficientes de la ecuación en diferencias.

El retardo se representa en muestras. Si (L/T_s) no es entero, debe
elegirse una aproximación o implementar una representación de retardo
fraccionario; el código básico contempla retardo entero.

Si se cambia `TS_US`, hay que recalcular `A[]`, `B[]` y `D`. No alcanza
con cambiar solamente el temporizador.

## 7. Comunicación serie

### Configuración

La comunicación utiliza el puerto serie USB del Wemos:

-   Baud rate: `115200`.
-   Terminador de comandos: salto de línea (`LF`, `\n`).
-   Formato de telemetría: tres columnas separadas por tabuladores.

### Envío de la entrada

La computadora puede enviar un número seguido de salto de línea:

``` text
2.5
```

También se acepta el formato:

``` text
u=2.5
```

La rutina `leerSerie()` interpreta el valor y lo limita al intervalo
`[U_MIN, U_MAX]`. El último valor válido recibido queda almacenado en
`u_latched` y se mantiene hasta recibir otro comando.

Por lo tanto, no es necesario enviar la entrada en cada paso de
integración: basta con transmitir un nuevo valor cuando el controlador o
el experimento lo requiera.

### Recepción de telemetría

La rutina `transmitir(u,y)` publica una línea con tres columnas:

``` text
timestamp_us    y    u
```

Los campos son:

1.  `timestamp_us`: valor de `micros()` en microsegundos.
2.  `y`: salida de la planta.
3.  `u`: entrada aplicada por el simulador.

Las columnas están separadas por tabuladores y cada trama termina con
salto de línea. La entrada transmitida permite registrar la señal
efectivamente aplicada, no solamente la que la computadora intentó
enviar.

El contador `micros()` se desborda aproximadamente cada 71,6 minutos. Al
reconstruir el tiempo, debe contemplarse ese desbordamiento.

**No conectar simultáneamente el Arduino Serial Plotter y MATLAB/Python
al mismo puerto:** el puerto serie normalmente puede ser abierto por un
solo programa a la vez.

## 8. Adquisición desde MATLAB

El proyecto incluye el script `adquirir_datos.m`, que ejemplifica una
adquisición con excitación PRBS.

El script:

-   Configura la duración del ensayo y los períodos de adquisición y
    excitación.
-   Genera una PRBS mediante `idinput` (requiere System Identification
    Toolbox).
-   Abre el puerto serie a 115200 baudios.
-   Envía los valores de entrada al Wemos.
-   Lee las tramas de telemetría.
-   Guarda timestamp, salida y entrada.
-   Reconstruye el tiempo a partir del reloj del Wemos.
-   Grafica las señales y guarda los datos en un archivo MAT.

### Parámetros principales del script

``` matlab
Texp = 360;       % Duración [s]
Ts_adq = 0.05;    % Período de telemetría esperado [s]
Ts_prbs = 0.05;   % Actualización de la PRBS [s]
A = 1;            % Amplitud de la PRBS
```

Hay que ajustar el puerto serie al equipo utilizado. En Linux, por
ejemplo:

``` matlab
'/dev/ttyUSB0'
```

En Windows puede ser, por ejemplo, `COM5`.

`Ts_adq` documenta el período esperado de telemetría; el tiempo efectivo
de cada muestra se obtiene de `timestamp_us`. `Ts_prbs` define cada
cuánto MATLAB actualiza la entrada de excitación. No tiene por qué
coincidir con el período de integración del modelo, aunque para ensayos
sencillos suele elegirse de manera compatible.

### Lectura básica desde MATLAB

Con la interfaz moderna `serialport`:

``` matlab
puerto = "/dev/ttyUSB0";
s = serialport(puerto, 115200);
configureTerminator(s, "LF");
s.Timeout = 2;

% Enviar una entrada
writeline(s, "2.5");

% Leer una trama
linea = readline(s);
v = sscanf(linea, '%f');

if numel(v) == 3
    timestamp_us = v(1);
    y = v(2);
    u = v(3);
end

clear s
```

La lectura debe repetirse para adquirir una secuencia de muestras.
Conviene descartar líneas incompletas o que no contengan exactamente
tres valores numéricos.

## 9. Adquisición desde Python

Se puede utilizar `pyserial` para enviar comandos y leer la telemetría.

Instalación:

``` bash
python -m pip install pyserial numpy matplotlib
```

Ejemplo mínimo de adquisición:

``` python
import serial
import time
import numpy as np
import matplotlib.pyplot as plt

PORT = "/dev/ttyUSB0"  # En Windows: "COM5"
BAUD = 115200
TEXP = 30.0
U = 2.5

ser = serial.Serial(PORT, BAUD, timeout=2)

# Algunos Wemos se reinician al abrir el puerto.
time.sleep(2)
ser.reset_input_buffer()

# Enviar entrada
ser.write(f"{U}\n".encode())

datos = []
t0 = time.monotonic()

try:
    while time.monotonic() - t0 < TEXP:
        linea = ser.readline().decode(
            "ascii", errors="ignore"
        ).strip()

        try:
            valores = [float(x) for x in linea.split()]
        except ValueError:
            continue

        if len(valores) != 3:
            continue

        timestamp_us, y, u = valores
        datos.append((timestamp_us, y, u))

finally:
    ser.write(b"0\n")
    ser.close()

datos = np.asarray(datos)

if len(datos) > 0:
    timestamp_us = datos[:, 0]
    y = datos[:, 1]
    u = datos[:, 2]

    # Reconstrucción del tiempo contemplando el rollover
    dt_us = np.mod(np.diff(timestamp_us), 2**32)
    t = np.concatenate(([0.0], np.cumsum(dt_us))) * 1e-6

    plt.figure()
    plt.subplot(2, 1, 1)
    plt.plot(t, y)
    plt.ylabel("y")
    plt.grid()

    plt.subplot(2, 1, 2)
    plt.step(t, u, where="post")
    plt.ylabel("u")
    plt.xlabel("Tiempo [s]")
    plt.grid()

    plt.tight_layout()
    plt.show()

    np.savetxt(
        "datos_planta.csv",
        np.column_stack((t, timestamp_us, y, u)),
        delimiter=",",
        header="t_s,timestamp_us,y,u",
        comments=""
    )
```

Este ejemplo mantiene una entrada constante. Para implementar un
controlador o una excitación variable, se puede actualizar `U` y
transmitir el nuevo valor mediante `ser.write()`.

## 10. Consideraciones para los ensayos

-   El período de integración `TS_US` determina la dinámica discreta
    simulada. Debe coincidir con el período usado al calcular los
    coeficientes.
-   El período de telemetría `TOUT_US` determina la frecuencia de
    publicación, no la discretización de la planta.
-   La entrada se mantiene constante entre comandos serie.
-   La saturación de entrada se aplica en el Wemos, por lo que la
    columna `u` de telemetría permite verificar el valor aplicado.
-   La adquisición desde una PC no es estrictamente determinista: el
    envío de comandos puede presentar latencias y jitter. Para
    identificar la planta, registrar siempre la entrada y la salida
    efectivamente recibidas.
-   La marca de tiempo se genera en el Wemos. Es preferible usarla para
    reconstruir el eje temporal en lugar de suponer que todas las
    muestras llegaron a intervalos perfectamente uniformes.
-   Los modelos deben ser numéricamente razonables y estables para la
    aritmética de punto flotante del microcontrolador.
-   Los límites de saturación deben ser compatibles con las unidades y
    escalas de cada ensayo.

## 11. Resumen

La arquitectura separa tres cuestiones:

-   **Modelo:** coeficientes discretos `A[]`, `B[]` y retardo `D`.
-   **Ejecución:** período de integración `TS_US` y función común
    `integrar(u)`.
-   **Interfaz experimental:** comunicación serie, saturaciones y
    período de telemetría `TOUT_US`.

Para sumar una planta no hay que modificar la adquisición ni la interfaz
de control: alcanza con discretizar el modelo para el `TS_US` elegido,
agregar sus coeficientes y seleccionar el `#define` correspondiente.
