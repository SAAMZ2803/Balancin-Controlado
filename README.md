# 🏗️ Balancín Controlado - ESP32

Sistema de control de un balancín motorizado basado en **ESP32 TTGO T-Display** con **sensado de ángulo en tiempo real**, **control PWM** y **regulador PI** para mantener el equilibrio automático.

## 📋 Descripción del Proyecto

Este proyecto implementa un sistema de **control en lazo cerrado** para un balancín motorizado. El sistema lee continuamente el ángulo de inclinación mediante un sensor analógico, procesa la medida y ajusta la potencia del motor mediante PWM para mantener un ángulo objetivo.

**Características principales:**
- ✅ Control PID con **anti-windup** para evitar saturación del integrador
- ✅ 4 modos de operación: rampa, 100% fijo, on/off, y control PID
- ✅ Interface serial interactivo (1200 baud)
- ✅ Pantalla TFT para visualización del menú
- ✅ Telemetría en tiempo real (ángulo, error, PWM)
- ✅ Medición de tiempo de respuesta (0° → max)
- ✅ Deadband configurable para compensar zona muerta del motor

## 🔧 Hardware

### Placa Principal
- **Microcontrolador:** ESP32 TTGO T-Display (ttgo-t1)
- **Pantalla:** TFT 135×240 px integrada
- **Conexión USB:** Para programación y monitoreo

### Periféricos
| Componente | GPIO | Función |
|-----------|------|---------|
| Sensor de ángulo (ADC) | GPIO 36 | Lectura analógica del ángulo |
| Motor PWM | GPIO 25 | Control del motor DC |
| LED auxiliar | GPIO 4 | Indicador de estado |

### Especificaciones Técnicas
- **Rango de ángulo:** 0° a 64°
- **Resolución ADC:** 12 bits (0-4095)
- **Frecuencia PWM:** 200 kHz
- **Bits PWM:** 8 bits (resolución 0-255)
- **Baudrate Serial:** 115200 bps
- **Tasa de muestreo:** 50 Hz (20 ms) con promedio de 10 muestras

## 📦 Requisitos

### Software
- **PlatformIO IDE** (VS Code + extensión)
- **Framework:** Arduino
- **Dependencias:**
  - `bodmer/TFT_eSPI@^2.5.43` (para la pantalla TFT)

### Instalación Rápida

```bash
# Clonar el repositorio
git clone https://github.com/SAAMZ2803/Balancin-Controlado.git
cd Balancin-Controlado

# Compilar
pio run

# Cargar en el ESP32
pio run -t upload

# Monitor serial
pio device monitor -b 115200
```

## 🎮 Modos de Operación

### Modo 0: Apagar Motor
```
Entrada: 0
Efecto: Motor en 0% PWM, integrador resetea
```

### Modo 1: Rampa 0→100% en 5 segundos
```
Entrada: 1
Efecto: Rampa lineal de potencia (útil para pruebas de respuesta dinámica)
```

### Modo 2: 100% Inmediato
```
Entrada: 2
Efecto: Motor a máxima potencia al instante
```

### Modo 3: On/Off Toggle
```
Entrada: 3
Efecto: Invierte estado del motor (encendido/apagado a 100%)
```

### Modo 4: Control PID (Lazo Cerrado)
```
Entrada: 4
Efecto: Activa control automático con setpoint por serial

Usar:
  4              → Activa modo PID
  30.0           → Nuevo setpoint de 30°
  <Enter>        → Confirma entrada
```

## 📊 Telemetría en Tiempo Real

Cada 20 ms se imprime una línea con el estado completo:
```
Angulo:32.45 | Setpoint:30.00 | PWM:82.35% | Modo:3 | Error:2.45 | Isum:15.32 | Eprev:2.10
```

**Campos:**
- `Angulo`: Ángulo medido (0-64°)
- `Setpoint`: Objetivo PID
- `PWM`: Duty cycle del motor (0-100%)
- `Modo`: 0=Idle, 1=Rampa, 2=FullOn, 3=PID
- `Error`: Diferencia setpoint - ángulo
- `Isum`: Suma del error (integrador)
- `Eprev`: Error anterior (para derivada)

## ⚙️ Configuración y Calibración

### Calibración del Sensor ADC
En `src/main.cpp`, líneas 10-11:
```cpp
constexpr int    kAdcMin      = 3791;    // ADC en posición 0°
constexpr int    kAdcMaxCal   = 2520;    // ADC en posición máx (64°)
```

**Procedimiento:**
1. Posicionar el balancín a 0°
2. Leer valor ADC en serial
3. Actualizar `kAdcMin`
4. Posicionar a 64°
5. Leer valor ADC
6. Actualizar `kAdcMaxCal`

### Ajuste de Ganancias PI
```cpp
constexpr float kPidKp = 0.0163f;  // Proporcional (respuesta rápida)
constexpr float kPidKi = 0.0698f;  // Integral (elimina error sostenido)
constexpr float kPidKd = 0.0f;     // Derivativa (generalmente 0 para esta planta)
```

**Tuning:**
- ↑ `Kp` → Respuesta más agresiva (riesgo de oscillación)
- ↑ `Ki` → Mejor rechazo de perturbaciones (riesgo de overshoot)
- Deadband (`kPwmDeadbandMin`) → Compensa zona muerta del motor

### Zona Muerta del Motor (Deadband)
```cpp
constexpr float kPwmDeadbandMin = 120.0f;  // PWM mínimo para que el motor gire
```

Mapeo interno:
- PWM [0-120] → Salida 0 (motor quieto)
- PWM [120-255] → Salida [120-255] (motor activo)

## 🚀 Uso Avanzado

### Monitoreo desde Python (opcional)
```python
import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 115200)
while True:
    line = ser.readline().decode('utf-8').strip()
    print(line)
    time.sleep(0.1)
```

### Exportar datos de telemetría
```bash
pio device monitor -b 115200 > datos.log
```

## ⚠️ Notas de Seguridad

1. **Verificar alimentación:** Asegurar que el motor tenga su propia fuente con masa común al ESP32
2. **MOSFET/Relay:** Usar driver apropiado (ej: IRF540N o relé) entre GPIO25 y motor
3. **Protección EMI:** Agregar diodo de clamp y capacitor cerca del motor si hay ruido
4. **Limite mecánico:** Asegurar que el balancín no supere ±64° para evitar daños
5. **Emergencia:** Desenchufar poder o enviaar `0` para apagar motor inmediatamente

## 📐 Esquema Simplificado

```
     (ADC)
        ↓
    [Potenciómetro/Sensor]
        ↓
    [ADC 12-bit → Ángulo]
        ↓
    [Controlador PI]
        ↓
    GPIO25 (PWM)
        ↓
    [MOSFET Driver]
        ↓
    [Motor DC]
        ↓
    [Balancín]
```

## 🐛 Troubleshooting

| Problema | Solución |
|----------|----------|
| Motor no responde | Verificar voltaje en GPIO25, revisar driver MOSFET |
| Ángulo no cambia | Revisar sensor, medir ADC raw en Serial |
| PID oscila | Reducir `Kp`, aumentar deadband |
| PID muy lento | Aumentar `Kp`, reducir deadband |
| Serial muestra basura | Cambiar baudrate a 115200 |

## 📝 Archivo de Referencias

- `src/main.cpp` - Código principal del controlador
- `graficar_angulo.m` - Script MATLAB para análisis de datos
- `platformio.ini` - Configuración del proyecto

## 📜 Licencia

Proyecto educativo - Libre para uso académico

---

**Autor:** Samuel Zea  
**Plataforma:** ESP32 TTGO T-Display  
**Última actualización:** 2026-05-12
