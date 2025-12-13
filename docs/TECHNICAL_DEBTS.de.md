# Technische Schulden

Dieses Dokument listet die bekannten technischen Schulden im `xDuinoRails_MotorControl_bEMF`-Projekt auf.

## Testen & Verifizierung
- **Fehlende Testinfrastruktur**: Unit-Tests müssen implementiert und in die CI-Pipeline integriert werden.
- **Begrenzte Simulationsabdeckung**: Die CI-Simulation führt nur `SineWaveSpeed` auf dem RP2040 aus. Sie sollte erweitert werden, um andere Beispiele und Szenarien abzudecken.

## Dokumentation
- **Veraltete Roadmap**: Die Roadmap führte zuvor gelöschte Plattformen (ESP32, nRF52, AVR) als implementiert auf.

## Code-Qualität
- **Magische Zahlen**: Ersetzen Sie fest codierte Werte in HAL-Implementierungen durch benannte Konstanten oder konfigurierbare Parameter.
