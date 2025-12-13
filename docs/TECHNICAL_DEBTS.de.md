# Technische Schulden

Dieses Dokument listet die bekannten technischen Schulden im `xDuinoRails_MotorControl_bEMF`-Projekt auf.

## Kritische Fehler
- **BEMF auf RP2040 deaktiviert**: Das Makro `USE_IRQ_TRIGGER` ist in der Build-Konfiguration nicht definiert, was dazu führt, dass der HAL die Interrupt-Registrierung überspringt und die BEMF-Messung effektiv deaktiviert.

## Testen & Verifizierung
- **Fehlende Testinfrastruktur**: Das Verzeichnis `test/` fehlt. Unit-Tests müssen implementiert und in die CI-Pipeline integriert werden.
- **Begrenzte Simulationsabdeckung**: Die CI-Simulation führt nur `SineWaveSpeed` auf dem RP2040 aus. Sie sollte erweitert werden, um andere Beispiele und Szenarien abzudecken.

## Dokumentation
- **Fehlende Prozessdokumentation**: `TESTING_IMPROVEMENTS.md` wird in `AGENTS.md` referenziert, existiert aber nicht.
- **Veraltete Roadmap**: Die Roadmap führte zuvor gelöschte Plattformen (ESP32, nRF52, AVR) als implementiert auf.

## Code-Qualität
- **Magische Zahlen**: Ersetzen Sie fest codierte Werte in HAL-Implementierungen durch benannte Konstanten oder konfigurierbare Parameter.
