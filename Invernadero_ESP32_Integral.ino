/*
  INVERNADERO 5 x 5 x 3 m - ESP32 WROOM32
  Integracion de sensores, LCD, extractor, nebulizacion, calefaccion y cortinas.
  Version de entrega: 2026-11-19

  REQUIERE: DHT sensor library (Adafruit), Adafruit Unified Sensor,
            LiquidCrystal I2C (API LiquidCrystal_I2C(addr,cols,rows)).

  SEGURIDAD: Por defecto TODAS las salidas de potencia y cortinas estan
  BLOQUEADAS. NO conectar equipos de 127 V ni motores hasta verificar
  contactos, aislamiento, protecciones, polaridades y pruebas de arranque.
  Los relés del WJZH simulan botones; no suministran potencia al motor.
  Los BTS7960 NO se manejan: no forman parte del mando WJZH aqui definido.
*/
#include <Arduino.h>
#include <Wire.h>
#include <DHT.h>
#include <LiquidCrystal_I2C.h>

// ------------------------- PINES -------------------------
constexpr uint8_t PIN_DHT = 4;
constexpr uint8_t PIN_SUELO[4] = {32, 33, 34, 35};
constexpr uint8_t PIN_EXTRACTOR = 13;
constexpr uint8_t PIN_BOMBA = 16;
constexpr uint8_t PIN_CAL1 = 17;
constexpr uint8_t PIN_CAL2 = 23;
constexpr uint8_t PIN_CORTINAS[4] = {18, 19, 25, 26};
constexpr uint8_t PIN_SDA = 21;
constexpr uint8_t PIN_SCL = 22;

// ------------------ CONFIGURACION SEGURA ------------------
// CAMBIAR A true SOLO despues de verificar hardware con cargas aisladas.
constexpr bool HABILITAR_POTENCIA = false;
constexpr bool HABILITAR_CORTINAS = false;
constexpr bool HABILITAR_CORTINAS_AUTO = false;

// Polaridad de cada modulo: comprobada LOW-activo solo en 4 reles de cortinas.
// CONFIRMAR las otras cuatro salidas antes de habilitar potencia.
constexpr bool ACTIVO_BAJO_EXTRACTOR = true;
constexpr bool ACTIVO_BAJO_BOMBA = true;
constexpr bool ACTIVO_BAJO_CAL1 = true;
constexpr bool ACTIVO_BAJO_CAL2 = true;
constexpr bool ACTIVO_BAJO_CORTINAS = true;

// Si la LCD esta en blanco, verificar direccion I2C (0x27 o 0x3F).
constexpr uint8_t LCD_ADDR = 0x27;

// Sensores suelo: CALIBRAR individualmente. 0..4095 son valores ADC brutos.
// Estos valores ilustrativos NO son una calibracion real.
constexpr int ADC_SECO[4] = {3000, 3000, 3000, 3000};
constexpr int ADC_HUMEDO[4] = {1300, 1300, 1300, 1300};

// ----------------- PARAMETROS CLIMATICOS ------------------
constexpr float T_OBJ = 26.0f;
constexpr float T_VENT_ON = 27.0f;
constexpr float T_VENT_OFF = 26.0f;
constexpr float T_CAL_ON = 25.0f;
constexpr float T_CAL_OFF = 26.0f;
constexpr float T_CAL2_ON = 23.5f;
constexpr float T_CAL2_OFF = 25.0f;
constexpr float HR_BOMBA_ON = 55.0f;
constexpr float HR_BOMBA_OFF = 62.0f;
constexpr float T_SEGURIDAD_ALTA = 40.0f;
constexpr float T_SEGURIDAD_BAJA = 5.0f;
constexpr uint32_t DHT_INTERVALO_MS = 2500UL;
constexpr uint32_t DHT_TIMEOUT_MS = 12000UL;
constexpr uint32_t LCD_INTERVALO_MS = 1000UL;
constexpr uint32_t REPORTE_MS = 5000UL;
constexpr uint32_t RETARDO_CAL2_MS = 60000UL;
constexpr uint32_t RETARDO_CORTINAS_MS = 180000UL;
constexpr uint32_t PULSO_BOTON_MS = 300UL;

// Tiempo de recorrido DEBE MEDIRSE. 0 bloquea secuenciacion automatica.
constexpr uint32_t RECORRIDO_CORTINA_MS = 0UL;

DHT dht(PIN_DHT, DHT22);
LiquidCrystal_I2C lcd(LCD_ADDR, 20, 4);

float temperatura = NAN, humedad = NAN;
int sueloADC[4] = {0, 0, 0, 0};
int sueloPct[4] = {0, 0, 0, 0};
uint32_t ultimoDHT = 0, ultimaLecturaValida = 0;
uint32_t ultimoLCD = 0, ultimoReporte = 0;
uint32_t inicioDemandaCalor = 0, inicioDemandaVent = 0;
bool dhtValido = false;
bool extractor = false, bomba = false, cal1 = false, cal2 = false;
bool demandaCalorAnterior = false, demandaVentAnterior = false;

// Un solo pulso de cortina a la vez. Sin autohoming ni movimientos al arrancar.
int8_t canalPulso = -1;
uint32_t inicioPulso = 0;
uint32_t ultimaOrdenCortina = 0;
bool cortinasSolicitadasAbiertas = false;
bool cortinasEstadoConocido = false;

void escribirSalida(uint8_t pin, bool encender, bool activoBajo) {
  digitalWrite(pin, encender ? (activoBajo ? LOW : HIGH)
                              : (activoBajo ? HIGH : LOW));
}

void apagarPotencia() {
  extractor = bomba = cal1 = cal2 = false;
  escribirSalida(PIN_EXTRACTOR, false, ACTIVO_BAJO_EXTRACTOR);
  escribirSalida(PIN_BOMBA, false, ACTIVO_BAJO_BOMBA);
  escribirSalida(PIN_CAL1, false, ACTIVO_BAJO_CAL1);
  escribirSalida(PIN_CAL2, false, ACTIVO_BAJO_CAL2);
}

void apagarCortinas() {
  for (uint8_t i = 0; i < 4; ++i)
    escribirSalida(PIN_CORTINAS[i], false, ACTIVO_BAJO_CORTINAS);
  canalPulso = -1;
}

void configurarSalidaSegura(uint8_t pin, bool activoBajo) {
  // Precarga el latch antes de poner OUTPUT para minimizar transitorios.
  escribirSalida(pin, false, activoBajo);
  pinMode(pin, OUTPUT);
  escribirSalida(pin, false, activoBajo);
}

bool lecturaDHTDisponible(uint32_t ahora) {
  return dhtValido && (ahora - ultimaLecturaValida <= DHT_TIMEOUT_MS);
}

void leerSensores(uint32_t ahora) {
  if (ahora - ultimoDHT < DHT_INTERVALO_MS) return;
  ultimoDHT = ahora;
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (!isnan(t) && !isnan(h) && t >= -10 && t <= 60 && h >= 0 && h <= 100) {
    temperatura = t;
    humedad = h;
    ultimaLecturaValida = ahora;
    dhtValido = true;
  } else {
    Serial.println(F("AVISO: lectura DHT22 invalida"));
  }
  for (uint8_t i = 0; i < 4; ++i) {
    sueloADC[i] = analogRead(PIN_SUELO[i]);
    if (ADC_SECO[i] != ADC_HUMEDO[i]) {
      long pct = map(sueloADC[i], ADC_SECO[i], ADC_HUMEDO[i], 0, 100);
      sueloPct[i] = constrain((int)pct, 0, 100);
    }
  }
}

void aplicarPotencia() {
  const bool habilitado = HABILITAR_POTENCIA;
  escribirSalida(PIN_EXTRACTOR, habilitado && extractor, ACTIVO_BAJO_EXTRACTOR);
  escribirSalida(PIN_BOMBA, habilitado && bomba, ACTIVO_BAJO_BOMBA);
  escribirSalida(PIN_CAL1, habilitado && cal1, ACTIVO_BAJO_CAL1);
  escribirSalida(PIN_CAL2, habilitado && cal2, ACTIVO_BAJO_CAL2);
}

void controlarClima(uint32_t ahora) {
  if (!lecturaDHTDisponible(ahora) || temperatura < T_SEGURIDAD_BAJA ||
      temperatura > T_SEGURIDAD_ALTA) {
    apagarPotencia();
    demandaCalorAnterior = demandaVentAnterior = false;
    return;
  }

  // Histeresis: las demandas de frio y calor no pueden coexistir.
  if (temperatura >= T_VENT_ON) extractor = true;
  else if (temperatura <= T_VENT_OFF) extractor = false;

  if (temperatura <= T_CAL_ON) cal1 = true;
  else if (temperatura >= T_CAL_OFF) cal1 = false;

  if (extractor) { cal1 = false; cal2 = false; }
  if (cal1) extractor = false;

  if (cal1 && !demandaCalorAnterior) inicioDemandaCalor = ahora;
  demandaCalorAnterior = cal1;

  // Segundo calefactor solo tras demora y frio persistente.
  if (!cal1 || temperatura >= T_CAL2_OFF) cal2 = false;
  else if (temperatura <= T_CAL2_ON &&
           ahora - inicioDemandaCalor >= RETARDO_CAL2_MS) cal2 = true;

  // Humidificacion solo si no hay calefaccion y no hay sobretemperatura.
  if (cal1 || cal2 || temperatura >= T_VENT_ON) bomba = false;
  else if (humedad <= HR_BOMBA_ON) bomba = true;
  else if (humedad >= HR_BOMBA_OFF) bomba = false;

  aplicarPotencia();
}

bool iniciarPulsoCortina(uint8_t canal, uint32_t ahora) {
  if (!HABILITAR_CORTINAS || canal > 3 || canalPulso >= 0) return false;
  apagarCortinas();
  escribirSalida(PIN_CORTINAS[canal], true, ACTIVO_BAJO_CORTINAS);
  canalPulso = (int8_t)canal;
  inicioPulso = ahora;
  ultimaOrdenCortina = ahora;
  cortinasEstadoConocido = false; // No hay sensor de posicion.
  return true;
}

void actualizarPulso(uint32_t ahora) {
  if (canalPulso >= 0 && ahora - inicioPulso >= PULSO_BOTON_MS) {
    apagarCortinas();
    Serial.println(F("Pulso de boton terminado"));
  }
}

void controlCortinasAutomatico(uint32_t ahora) {
  // Deshabilitado hasta medir recorridos, verificar WJZH y disponer de
  // proteccion adecuada ante ordenes de movimiento y posiciones desconocidas.
  if (!HABILITAR_CORTINAS || !HABILITAR_CORTINAS_AUTO ||
      RECORRIDO_CORTINA_MS == 0 || !lecturaDHTDisponible(ahora)) return;
  // Esta version NO emite ordenes automaticas de movimiento: falta una
  // confirmacion segura de posicion/fin de movimiento de cada cortina.
  // Las condiciones de temperatura se muestran solo como recomendacion.
}

void lineaLCD(uint8_t fila, const char *texto) {
  lcd.setCursor(0, fila);
  char buffer[21];
  snprintf(buffer, sizeof(buffer), "%-20.20s", texto);
  lcd.print(buffer);
}

void actualizarLCD(uint32_t ahora) {
  if (ahora - ultimoLCD < LCD_INTERVALO_MS) return;
  ultimoLCD = ahora;
  char b[32];
  if (lecturaDHTDisponible(ahora)) {
    snprintf(b, sizeof(b), "T:%4.1fC HR:%4.1f%%", temperatura, humedad);
    lineaLCD(0, b);
  } else lineaLCD(0, "DHT22 SIN LECTURA");
  snprintf(b, sizeof(b), "S1:%3d%% S2:%3d%%", sueloPct[0], sueloPct[1]);
  lineaLCD(1, b);
  snprintf(b, sizeof(b), "S3:%3d%% S4:%3d%%", sueloPct[2], sueloPct[3]);
  lineaLCD(2, b);
  if (!HABILITAR_POTENCIA && !HABILITAR_CORTINAS)
    lineaLCD(3, "MODO SEGURO: BLOQUEO");
  else {
    snprintf(b, sizeof(b), "E:%d B:%d C1:%d C2:%d", extractor, bomba, cal1, cal2);
    lineaLCD(3, b);
  }
}

void reportar(uint32_t ahora) {
  if (ahora - ultimoReporte < REPORTE_MS) return;
  ultimoReporte = ahora;
  Serial.print(F("DHT_OK=")); Serial.print(lecturaDHTDisponible(ahora));
  Serial.print(F(" T=")); Serial.print(temperatura);
  Serial.print(F(" HR=")); Serial.print(humedad);
  Serial.print(F(" ADC="));
  for (uint8_t i = 0; i < 4; ++i) {
    Serial.print(sueloADC[i]);
    if (i < 3) Serial.print(',');
  }
  Serial.print(F(" DEMANDAS E/B/C1/C2="));
  Serial.print(extractor); Serial.print('/');
  Serial.print(bomba); Serial.print('/');
  Serial.print(cal1); Serial.print('/');
  Serial.println(cal2);
}

void comandosSerial(uint32_t ahora) {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') continue;
    Serial.print(F("RECIBIDO: ")); Serial.println(c);
    if (c == '0') {
      apagarCortinas();
      Serial.println(F("Rele de mando de cortinas: apagados (NO es STOP de motor WJZH)"));
    } else if (c >= '1' && c <= '4') {
      if (!HABILITAR_CORTINAS) {
        Serial.println(F("BLOQUEADO: HABILITAR_CORTINAS=false"));
      } else if (iniciarPulsoCortina((uint8_t)(c - '1'), ahora)) {
        Serial.println(F("Pulso enviado. NO confirma movimiento ni posicion"));
      } else Serial.println(F("Canal ocupado"));
    } else if (c == 'S' || c == 's') {
      Serial.println(F("ESTADO: monitor activo"));
      reportar(ahora + REPORTE_MS);
    } else {
      Serial.println(F("Comandos: S=estado, 0=desactivar reles, 1..4=mandos bloqueados"));
    }
  }
}

void setup() {
  // Precarga y configura salidas ANTES de inicializar perifericos.
  configurarSalidaSegura(PIN_EXTRACTOR, ACTIVO_BAJO_EXTRACTOR);
  configurarSalidaSegura(PIN_BOMBA, ACTIVO_BAJO_BOMBA);
  configurarSalidaSegura(PIN_CAL1, ACTIVO_BAJO_CAL1);
  configurarSalidaSegura(PIN_CAL2, ACTIVO_BAJO_CAL2);
  for (uint8_t i = 0; i < 4; ++i)
    configurarSalidaSegura(PIN_CORTINAS[i], ACTIVO_BAJO_CORTINAS);
  apagarPotencia();
  apagarCortinas();

  Serial.begin(115200);
  delay(300);
  Serial.println(F("\nINVERNADERO ESP32 - INICIO SEGURO"));
  Serial.println(F("Serial 115200. Enviar S para estado."));
  Serial.println(F("ATENCION: salidas bloqueadas por configuracion."));

  Wire.begin(PIN_SDA, PIN_SCL);
  lcd.init();
  lcd.backlight();
  lineaLCD(0, "INVERNADERO ESP32");
  lineaLCD(1, "INICIANDO SISTEMA");
  lineaLCD(2, "SALIDAS BLOQUEADAS");
  lineaLCD(3, "SERIAL 115200");
  dht.begin();
  analogReadResolution(12);
  for (uint8_t i = 0; i < 4; ++i) pinMode(PIN_SUELO[i], INPUT);
}

void loop() {
  uint32_t ahora = millis();
  comandosSerial(ahora);
  actualizarPulso(ahora);
  leerSensores(ahora);
  controlarClima(ahora);
  controlCortinasAutomatico(ahora);
  actualizarLCD(ahora);
  reportar(ahora);
}
