# PWM-Steuerung und Frequenzwahl

Pulsweitenmodulation (PWM) ist die Kerntechnologie zur Regelung der Motorleistung. Dieses Dokument beschreibt die aktuelle Implementierung und die Auswirkungen der gewählten PWM-Frequenz.

## Aktuelle Implementierung: Hardware-beschleunigtes PWM

Das Projekt nutzt die Hardware-PWM-Slices des RP2040, um eine hochfrequente, nicht-blockierende Motorsteuerung zu realisieren.

- **PWM-Frequenz:** Die Standardfrequenz beträgt **20 kHz**.
- **Spezial-Edition:** Für die "LED Edition" ist die Frequenz auf **10 Hz** eingestellt, um die PWM-Impulse auf LEDs sichtbar zu machen.

## 20 kHz Frequenz

Die Wahl von 20 kHz legt die Schaltfrequenz oberhalb des hörbaren Bereichs für die meisten Menschen, was zu einem geräuschlosen Betrieb führt.

### Vorteile

- **Geräuschloser Betrieb:** Kein hörbares Pfeifen oder Brummen vom Motor.
- **Glatteres Drehmoment:** Die hohe Frequenz führt zu einem gleichmäßigeren Stromfluss durch die Motorwicklungen.

### BEMF-Messung

Bei 20 kHz beträgt die gesamte Zykluszeit 50 µs. Die BEMF-Messung muss während der OFF-Phase des PWM-Zyklus erfolgen.

- **Messfenster:** Die Messung erfolgt unmittelbar nach dem Ende des PWM-Impulses.
- **Hardware-Trigger:** Der PWM-Wrap-Interrupt löst eine Hardware-Timer-Verzögerung (typischerweise 10 µs) aus, um das Einschwingen zu ermöglichen, gefolgt von einem ADC-Start-Trigger.
- **DMA-Übertragung:** ADC-Ergebnisse werden automatisch per DMA in den Speicher übertragen.

Dieses enge Timing wird vollständig durch Hardware-Trigger und DMA gehandhabt und erfordert nur minimale CPU-Intervention.
