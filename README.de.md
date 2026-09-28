# esphome-vpd-regler

[English](README.md) | **Deutsch**

> [!NOTE]
> Dies ist die deutsche Übersetzung. Maßgeblich ist die englische
> [README.md](README.md). Weichen beide voneinander ab, gilt die englische.

> [!WARNING]
> **Dieses Repository wurde vollständig mit KI erstellt.** Code,
> Dokumentation und Tests hat ein KI-Assistent geschrieben.
>
> Ich habe es selbst in meinem Growzelt getestet, aber meine Tests können
> nicht jeden Aufbau, jeden Lüfter, jeden Sensor und jeden Fehlerfall abdecken.
> Setze es mit Bedacht ein: Beobachte den Regler am Anfang genau, halte die
> Sicherheitsgrenzen sinnvoll eingestellt und lass ihn nicht unbeaufsichtigt
> laufen, wo eine falsche Lüftereinstellung deinen Pflanzen oder deiner
> Technik schaden könnte. Es gibt keinerlei Gewährleistung (siehe
> [LICENSE](LICENSE)).

Eine externe Komponente für [ESPHome](https://esphome.io), die das
**VPD (Dampfdruckdefizit)** eines Growzelts mit einem einzelnen Abluftlüfter
regelt.

- Modellbasierter Regler mit **Kalman-Filter**, der schätzt, wie viel
  Feuchtigkeit die Pflanzen abgeben, statt eines PID
- Findet das **sinnvolle Maximum**: Er dreht den Lüfter nicht weiter hoch,
  sobald mehr Luft weniger VPD bringen würde, als du aufzugeben bereit bist
  (spart Energie und Lärm, besonders im Winter)
- Getrennte **Tag- und Nachtziele**, weicher **Sollwert-Übergang** nach
  Lichtwechseln, gemerkte Lüfterdrehzahl je Lichtphase
- **Temperatur- und Feuchteschutz**, der den Regler übersteuert, mit
  einer Benachrichtigung pro Ereignis
- Schalter **Tent open** (Zelt offen), der den Regler pausiert, während du im
  Zelt arbeitest
- Blatttemperatur von einem IR-Sensor (z. B. MLX90614) oder über einen festen
  Offset für Tag und Nacht
- Optionaler zweiter Zeltsensor (z. B. über ESP-NOW) per Lambda
- Jede Einstellung ist eine Home-Assistant-Entität, jeder interne Wert kann
  als Diagnosesensor ausgegeben werden
- Zustandstexte und Benachrichtigungen auf Englisch oder Deutsch

Der Lüfter ist das Einzige, was er ansteuert. Befeuchter, Heizungen oder das
Dimmen der Lampe gehören in eigene Komponenten und können die Entitäten
dieser Komponente lesen.

---

## Wie es funktioniert

Das Zelt folgt der Raumluft. Der Regler arbeitet mit dem Dampfdruck (kPa),
der gleich bleibt, wenn sich die angesaugte Luft erwärmt:

```
e_zelt = e_raum + E           E  = Dampfdrucküberschuss des Zelts (kPa)
E      = L / q(u)             L  = Feuchtelast (Transpiration)
q(u)   = q0 + (1 - q0) * u    q  = relativer Luftstrom, q0 = Luftstrom bei 0 %
```

Änderungen im Raum gehen direkt ins Zelt über, der Lüfter teilt den
Überschuss `E` durch den Luftstrom. Das Zelt folgt mit der Zeitkonstante
`tau / q`.

Alle 10 s

1. misst der Regler Zelt-, Raum- und Blatttemperatur,
2. schätzt ein Kalman-Filter auf `E` die Last `L`,
3. leitet er aus `L` das **sinnvolle Maximum** ab: die Lüftereinstellung, bei
   der voller Lüfter nur noch das *erlaubte VPD-Opfer* mehr bringen würde,
4. bewegt er den Lüfter auf einer Log-Skala zum Bandrand
   (`ln q += k * dt * ln(E / E_rand)`, mit `k = speed * q / tau`) und macht
   innerhalb des Totbands nichts,
5. wendet er die Sicherheit an: `Ausgang = max(Regler, Temperatur-P, Feuchte-P)`.

Nach einem Lichtwechsel und nach dem Schließen des Zelts beginnt das wirksame
Ziel beim gemessenen VPD und läuft innerhalb der Zeit für den
*Sollwert-Übergang* zum Ziel. So kämpft der Lüfter nicht gegen eine Lücke an,
die Aufheizen oder Abkühlen von selbst schließt.

Die Richtung steht fest: Mehr Abluft bedeutet trockenere Luft und ein höheres
VPD. Ist deine Raumluft feuchter als die Zeltluft, ist dieser Regler nichts
für dich.

---

## Installation

```yaml
external_components:
  - source: github://Malvus524/esphome-vpd-regler@v1.0.0
    components: [vpd_kalman]
```

Lege einen Tag fest (`@v1.0.0`), damit Updates nur dann kommen, wenn du es
willst.

## Minimale Konfiguration

```yaml
vpd_kalman:
  output: fan_pwm              # Float-Ausgang, der den Lüfter ansteuert
  night: lights_off            # Binärsensor, ON = Nacht
  temperature: tent_temperature
  humidity: tent_humidity
  room_temperature: room_temperature
  room_humidity: room_humidity
  language: de                 # deutsche Zustandstexte und Meldungen
```

Das legt alle Einstellungen und Schalter an, mit englischen Namen (eigene
Namen gibst du über `name`, siehe [Einstellungen](#einstellungen-number-entitäten)).
Der Regler startet im **Handbetrieb**: Schalte den Schalter *VPD control* ein,
damit er den Lüfter übernimmt. Ein vollständiges Beispiel mit Sensoren,
Diagnose und Benachrichtigungen steht in [example.yaml](example.yaml).

---

## Konfigurationsreferenz

Die Schlüssel sind englisch und bleiben so. In den Tabellen steht unter
*Standardname* der Name, den die Entität ohne eigenes `name` bekommt.

### Eingänge

| Schlüssel | Pflicht | Beschreibung |
|---|---|---|
| `output` | ja | Float-Ausgang des Lüfters (z. B. `ledc`). Der Regler schreibt 1-100 %, er schaltet den Lüfter nie aus. Mit `min_power`/`max_power` des Ausgangs passt du das an deinen Lüfter an. |
| `night` | ja | Binärsensor, ON = Nacht (Licht aus). Tag/Nacht wählt Ziel, Blatt-Offset und gemerkte Lüfterdrehzahl. Bis er nach dem Start einen Zustand hat, hält der Regler den Lüfter (max. 3 min). |
| `temperature`, `humidity` | ja | Luftsensor im Zelt. |
| `leaf_temperature` | nein | Blatttemperatursensor (IR). Wird genutzt, solange der Schalter *Leaf temperature from sensor* an ist und der Wert höchstens `leaf_max_deviation` von der Lufttemperatur abweicht. Sonst Lufttemperatur + Blatt-Offset Tag/Nacht. |
| `room_temperature`, `room_humidity` | ja | Luft, die der Lüfter ansaugt. Fehlt sie, hält der Regler den Lüfter. |
| `external_climate` | nein | Lambda, das `vpd_kalman::ExternalClimate` mit `temperature`, `humidity`, `leaf_temperature` (und `selected` fürs Log) zurückgibt. Sind alle drei gültig, werden sie statt der eigenen Sensoren genutzt, z. B. für einen zweiten Sensor auf Höhe des Blätterdachs. Siehe unten. |

### Optionen

| Schlüssel | Standard | Beschreibung |
|---|---|---|
| `airflow_at_zero` | `0.2` | Luftstrom des Lüfters bei 0 % im Verhältnis zu 100 % (`q0`). Geht in die Feuchtelast und das sinnvolle Maximum ein. |
| `time_constant` | `21s` | Zeit, die das Zelt bei 100 % Lüfter braucht, bis 63 % einer Feuchteänderung abgeführt sind (`tau`). Legt die Filterdynamik fest und, zusammen mit *Controller speed*, wie schnell der Regler eingreift. |
| `leaf_max_deviation` | `6.0` | Blattsensorwerte, die weiter als dieser Wert (°C) von der Lufttemperatur entfernt sind, gelten als Messfehler. |
| `language` | `en` | `en` oder `de`: Zustandstexte, Benachrichtigungen und Logzeilen. |
| `storage_keys` | - | Nur für den Umstieg von einer `globals:`-Lösung, siehe unten. |
| `on_message` | - | Automation mit `title` und `message` (`std::string`) für Benachrichtigungen. |

Der Regler läuft alle 10 s. Das ist fest, weil Filter und Regelgesetz darauf
abgestimmt sind.

### Einstellungen (Number-Entitäten)

Alle werden automatisch angelegt, im Flash gespeichert und als Eingabefeld
angezeigt. Jede akzeptiert die üblichen Number-Optionen (`name`, `id`, `icon`,
`entity_category`, ...) sowie `min_value`, `max_value`, `step` und
`initial_value`.

| Schlüssel | Standardname | Standard | Bereich | Bedeutung |
|---|---|---|---|---|
| `manual_speed` | Fan manual speed | 50 % | 1-100 | Lüfterdrehzahl, solange *VPD control* aus ist. |
| `target_day` | VPD target day | 1.1 kPa | 0.3-2.5 | |
| `target_night` | VPD target night | 0.9 kPa | 0.3-2.5 | |
| `deadband` | VPD deadband | 0.05 kPa | 0.01-0.5 | Ziel +/- dieser Wert gilt als erreicht, der Lüfter bleibt, wo er ist. |
| `vpd_sacrifice` | Allowed VPD sacrifice | 0.03 kPa | 0-0.3 | Wie viel VPD der Regler aufgeben darf, um Lüfter zu sparen. 0 = hochfahren bis *Fan maximum automatic*. |
| `speed` | Controller speed | 0.25 | 0.05-1 | 0.25 ist laut Modell aperiodisch gedämpft. Höher = schneller, kann aber schwingen, wenn das Zelt langsamer ist als `time_constant`. |
| `transition` | Setpoint transition | 45 min | 0-180 | Dauer des Sollwert-Übergangs. 0 = aus. |
| `tent_open_max` | Tent open max. duration | 30 min | 5-240 | *Tent open* schaltet sich nach dieser Zeit selbst aus. |
| `fan_min` | Fan minimum automatic | 5 % | 1-100 | |
| `fan_max` | Fan maximum automatic | 100 % | 1-100 | |
| `fan_emergency` | Fan emergency speed | 40 % | 1-100 | Wird genutzt, wenn der Zeltsensor ausfällt, und ist das Minimum, ab dem die Sicherheit einsetzt. |
| `temperature_max` | Safety temperature max | 28 °C | 20-40 | Darüber mindestens *Fan emergency speed*, linear steigend bis 100 % bei Max + P-Band. |
| `temperature_band` | Safety temperature P-band | 3 K | 0.5-10 | |
| `humidity_max` | Safety humidity max | 75 % | 40-95 | Muss über der Feuchte liegen, die sich aus deinem VPD-Ziel ergibt. |
| `humidity_band` | Safety humidity P-band | 10 % | 1-30 | |
| `leaf_offset_day` | Leaf offset day | -2.0 °C | -10-5 | Blatttemperatur = Luft + Offset, wenn kein gültiger Blattsensorwert genutzt wird. |
| `leaf_offset_night` | Leaf offset night | -1.0 °C | -10-5 | |

### Schalter

| Schlüssel | Standardname | Wiederherstellung | Bedeutung |
|---|---|---|---|
| `control` | VPD control | Standard aus | ON = der Regler steuert den Lüfter (übernimmt die aktuelle Drehzahl ohne Sprung). OFF = Handdrehzahl. |
| `tent_open` | Tent open | immer aus | Hält den Lüfter, pausiert Filter und Merken, die Sicherheit bleibt aktiv. |
| `leaf_sensor` | Leaf temperature from sensor | Standard an | `leaf_temperature` statt der Offsets nutzen. |

### Diagnose (optional)

Wird nur angelegt, wenn du den Schlüssel (mit mindestens einem `name`)
hinzufügst. Alle akzeptieren die üblichen Sensor-Optionen.

| Schlüssel | Einheit | Bedeutung |
|---|---|---|
| `control_vpd` | kPa | VPD, mit dem der Regler arbeitet (ungefiltert). |
| `target_active` | kPa | Wirksames Ziel, einschließlich Sollwert-Übergang. |
| `controller_output` | % | Was der Regler will (vor der Sicherheit). |
| `sensible_max` | % | Höchste sinnvolle Lüftereinstellung, siehe *Allowed VPD sacrifice*. |
| `fan_output` | % | Was am Ausgang anliegt. |
| `excess` | kPa | Dampfdrucküberschuss Zelt - Raum. |
| `excess_target` | kPa | Überschuss, den das Ziel verlangt. 0 oder negativ = mit dieser Raumluft unerreichbar. |
| `moisture_load` | kPa | Kalman-Schätzung der Last, proportional zur Transpiration. |
| `next_step_benefit` | kPa | VPD-Gewinn der nächsten Lüfterstufe (+15 % Luftstrom). |
| `vpd_at_max` | kPa | VPD, das *Fan maximum automatic* bei gleicher Temperatur erreichen würde. |
| `state` | Text | Mit `language: de` z. B. *Im Band*, *Regeln*, *An sinnvollem Maximum*, *Ziel unerreichbar*, *Sicherheit (Temperatur)*, *Zelt offen - pausiert (5 min)*. |
| `temperature_protection` | binär | ON, solange der Temperaturschutz den Lüfter anhebt. |
| `humidity_protection` | binär | ON, solange der Feuchteschutz den Lüfter anhebt. |

---

## Benachrichtigungen

```yaml
vpd_kalman:
  # ...
  on_message:
    - homeassistant.action:
        action: notify.notify
        data:
          title: !lambda 'return title;'
          message: !lambda 'return message;'
```

Wird gesendet, wenn der Temperatur- oder Feuchteschutz auslöst (einmal pro
Ereignis, Entwarnung nach 60 min ohne Auslösung) und wenn *Tent open*
automatisch endet. Für `homeassistant.action` musst du dem Gerät in den
Optionen der ESPHome-Integration erlauben, Home-Assistant-Aktionen
auszuführen.

## Zweiter Zeltsensor (`external_climate`)

Zum Beispiel ein Sensor auf Höhe des Blätterdachs an einem anderen ESP,
empfangen über ESP-NOW oder `packet_transport`. Gib für fehlende Werte NAN
zurück, der Regler nutzt dann für diesen Durchlauf seine eigenen Sensoren.
Wechselt die Quelle, wird der Filter neu abgeglichen und ein
Sollwert-Übergang startet, weil zwei Sensoren an verschiedenen Stellen
verschiedene VPDs messen.

```yaml
switch:
  - platform: template
    id: use_canopy_sensor
    name: "Regeln mit Sensor am Blätterdach"
    optimistic: true
    restore_mode: RESTORE_DEFAULT_ON

vpd_kalman:
  # ...
  external_climate: !lambda |-
    vpd_kalman::ExternalClimate c;
    c.selected = id(use_canopy_sensor).state;
    if (c.selected) {
      c.temperature = id(canopy_temperature).state;
      c.humidity = id(canopy_humidity).state;
      c.leaf_temperature = id(canopy_temperature).state - 2.0f;
    }
    return c;
```

## Umstieg von einem YAML-Lambda

Entitäten behalten ihren Home-Assistant-Verlauf und ihre gespeicherten Werte,
wenn du ihnen denselben `name` (daraus wird der Flash-Schlüssel abgeleitet)
und dieselbe `id` (für deine anderen Lambdas) gibst. Hat deine alte Lösung
die Reglerwerte in wiederherstellenden `globals:` gespeichert, verweise mit
`storage_keys` auf deren IDs:

```yaml
vpd_kalman:
  storage_keys:
    controller_output: u_vpd_gespeichert
    day: u_tag_gespeichert
    night: u_nacht_gespeichert
```

## Lizenz

[MIT](LICENSE)
