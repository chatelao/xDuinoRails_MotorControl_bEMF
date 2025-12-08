# Entwicklerreferenz

Dieses Dokument bietet eine detaillierte Referenz für die API der `xDuinoRails_MotorControl_bEMF`-Bibliothek.

## Typedefs

### `hal_bemf_update_callback_t`

```cpp
typedef void (*hal_bemf_update_callback_t)(int raw_bemf_value);
```

Ein Funktionszeigertyp für den BEMF-Update-Callback.

- **Parameter:**
  - `raw_bemf_value`: Der rohe, ungefilterte differenzielle BEMF-Wert.
- **Kontext:** Diese Funktion wird aus einer Interrupt-Service-Routine (ISR) aufgerufen. Sie sollte so kurz und effizient wie möglich gehalten werden.

---

## Funktionen

### `hal_motor_init`

```cpp
void hal_motor_init(
  uint8_t pwm_a_pin,
  uint8_t pwm_b_pin,
  uint8_t bemf_a_pin,
  uint8_t bemf_b_pin,
  hal_bemf_update_callback_t callback
);
```

Initialisiert die Low-Level-Hardware für die Motorsteuerung. Diese Funktion muss einmal vor jeder anderen Funktion in dieser Bibliothek aufgerufen werden.

- **Parameter:**
  - `pwm_a_pin`: Der GPIO-Pin für den PWM-Kanal A.
  - `pwm_b_pin`: Der GPIO-Pin für den PWM-Kanal B.
  - `bemf_a_pin`: Der ADC-Pin für die BEMF-Messung A.
  - `bemf_b_pin`: Der ADC-Pin für die BEMF-Messung B.
  - `callback`: Ein Zeiger auf die Funktion, die aufgerufen werden soll, wenn neue BEMF-Daten verfügbar sind.

---

### `hal_motor_set_pwm`

```cpp
void hal_motor_set_pwm(int duty_cycle, bool forward);
```

Stellt das PWM-Tastverhältnis und die Richtung des Motors ein.

- **Parameter:**
  - `duty_cycle`: Das PWM-Tastverhältnis, typischerweise von 0 bis 255. Der genaue Bereich kann je nach PWM-Auflösung der Hardware variieren.
  - `forward`: Die Motorrichtung. `true` für vorwärts, `false` für rückwärts.

---

### `hal_motor_get_bemf_buffer`

```cpp
int hal_motor_get_bemf_buffer(
  volatile uint16_t** buffer,
  int* last_write_pos
);
```

Ruft den internen BEMF-Ringpuffer zur Diagnose und zum Debugging ab.

- **Parameter:**
  - `buffer` (out): Ein Zeiger auf einen `uint16_t`-Zeiger. Dieser wird aktualisiert, um auf den internen Ringpuffer zu zeigen.
  - `last_write_pos` (out): Ein Zeiger auf eine Ganzzahl. Dieser wird mit dem Index des zuletzt geschriebenen Samples im Puffer aktualisiert.
- **Rückgabe:**
  - Die Gesamtgröße des Ringpuffers.

---

### `hal_motor_get_current_buffer`

```cpp
int hal_motor_get_current_buffer(
  volatile uint16_t** buffer,
  int* last_write_pos
);
```

Ruft den Ringpuffer für die Strommessung ab. Optional: Nur verfügbar, wenn von der Plattform und Konfiguration unterstützt/aktiviert.

- **Parameter:**
  - `buffer` (out): Zeiger auf den `uint16_t`-Zeiger, der auf die Adresse des Puffers gesetzt wird.
  - `last_write_pos` (out): Zeiger auf eine Ganzzahl, die auf die zuletzt geschriebene Position gesetzt wird.
- **Rückgabe:**
  - Die Größe des Ringpuffers oder 0, falls nicht unterstützt.

---

### `hal_motor_check_solenoid_position`

```cpp
void hal_motor_check_solenoid_position(
  int ping_pwm_value,
  int ping_duration_ms,
  int measurement_delay_us,
  int* response_a,
  int* response_b
);
```

Überprüft die Solenoid-/Motorposition durch "Anpingen" beider Richtungen und Messen der Reaktion. Diese Funktion führt einen Diagnosetest durch, indem sie die Richtung "vorwärts" anpingt, die BEMF-Reaktion misst und dies dann für die Richtung "rückwärts" wiederholt.

- **Parameter:**
  - `ping_pwm_value`: Das PWM-Tastverhältnis (0-255) für den Ping-Impuls.
  - `ping_duration_ms`: Die Dauer des Ping-Impulses in Millisekunden.
  - `measurement_delay_us`: Die Verzögerung nach dem Ausschalten vor der Messung der Reaktion.
  - `response_a` (out): Die gemessene Reaktion für die Vorwärts-/A-Richtung.
  - `response_b` (out): Die gemessene Reaktion für die Rückwärts-/B-Richtung.
