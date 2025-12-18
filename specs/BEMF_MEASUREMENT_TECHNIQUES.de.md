# BEMF-Messtechnik

Die genaue Messung der Back-EMF (BEMF) ist entscheidend für die präzise, sensorlose Regelung des Motors. Dieses Dokument beschreibt die im Projekt implementierte Technik, um aus dem verrauschten Rohsignal eine stabile Geschwindigkeitsinformation zu gewinnen.

Der Prozess basiert auf einer differenziellen Messung.

## 1. Grundlage: Differenzielle Messung

Die BEMF wird als Differenzspannung zwischen den beiden Motoranschlüssen (`bemfAPin` und `bemfBPin`) gemessen.

```cpp
// Aus src/main.cpp
int bemfA = analogRead(bemfAPin);
int bemfB = analogRead(bemfBPin);
int measured_bemf = abs(bemfA - bemfB);
```

**Vorteile:**
- **Unterdrückung von Gleichtaktstörungen:** Rauschen, das auf beiden Leitungen gleichzeitig auftritt (z.B. durch die PWM-Ansteuerung induziert), wird durch die Differenzbildung effektiv eliminiert.
- **Robuste Grundlage:** Liefert ein besseres Rohsignal als eine Messung gegen Masse.

## Zusammenfassung

Die aktuelle Lösung nutzt eine bewährte Technik zur robusten Signalerfassung:

| Stufe | Technik | Zweck |
| :--- | :--- | :--- |
| **1** | **Differenzielle Messung** | Grundlage: Erfassung des Rohsignals mit Unterdrückung von Gleichtaktstörungen. |

Diese Methode stellt sicher, dass ein sauberes Rohsignal für die weitere Verarbeitung, beispielsweise für einen PI-Regler, zur Verfügung steht.
