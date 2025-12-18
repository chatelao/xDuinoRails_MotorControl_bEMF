# Plan: Unterstützung für diskrete H-Brücke (4-Pin)

## Ziel
Erweiterung der Motorsteuerungs-Bibliothek um die Unterstützung von diskreten H-Brücken, bei denen 4 separate GPIO-Pins für die High-Side (HS) und Low-Side (LS) MOSFETs benötigt werden.

## Technische Anforderungen (RP2040)
*   Nutzung von 2 PWM-Slices pro Motor (eine Slice pro Halbbrücke) zur Nutzung der Hardware-Features.
*   Verwendung der Hardware-Dead-Time (Totzeit) des RP2040 zur Vermeidung von Shoot-Through (Kurzschluss durch gleichzeitiges Schalten).
*   Synchrone Ansteuerung für BEMF-Messung.

## Schritte

### 1. API-Erweiterung (`motor_control_hal.h`)
- [x] Neue Initialisierungsfunktion definieren:
  ```cpp
  void hal_motor_init_discrete(
      uint8_t hs_a_pin, uint8_t ls_a_pin, // Linke Halbbrücke (Seite A)
      uint8_t hs_b_pin, uint8_t ls_b_pin, // Rechte Halbbrücke (Seite B)
      uint8_t bemf_a_pin, uint8_t bemf_b_pin,
      hal_bemf_update_callback_t callback,
      uint8_t motor_id
  );
  ```

### 2. Datenstrukturen (`motor_control_hal_rp2040.cpp`)
- [x] `MotorContext` erweitern:
  - Speicherung von 4 Pins statt 2.
  - Flag für `is_discrete_mode`.
  - Speicherung von 2 Slice-IDs (Slice A für Linke HB, Slice B für Rechte HB).

### 3. Implementierung Init-Logik
- [x] Pin-Validierung:
  - Prüfen, ob HS/LS Paare jeder Seite auf demselben PWM-Slice liegen (Erforderlich für Hardware-Dead-Time).
    - Beispiel: `hs_a_pin` and `ls_a_pin` müssen auf derselben Slice-ID liegen.
- [x] PWM-Konfiguration:
  - Setzen der Dead-Time via SDK (`pwm_set_dead_time`).
  - Konfiguration der Polarität für komplementäre Ausgänge.

### 4. Implementierung Steuer-Logik (`hal_motor_set_pwm`)
- [x] Fallunterscheidung in `hal_motor_set_pwm`:
  - Wenn `is_discrete_mode`:
    - Berechnung der Duty Cycles für beide Halbbrücken.
    - **Vorwärts**: Linke HB = High (HS=An, LS=Aus), Rechte HB = PWM.
    - **Rückwärts**: Linke HB = PWM, Rechte HB = High.
    - *Hinweis:* Die genaue Kommutierungsstrategie (Slow Decay vs. Fast Decay) bestimmt das Schaltmuster.

### 5. Verifikation
- [x] Kompilierbarkeit prüfen.
- [ ] Logik-Verifikation (z.B. mit Logic Analyzer).
