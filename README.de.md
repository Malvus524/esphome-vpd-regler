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

## Warum ein eigener VPD-Regler?

Das VPD entscheidet, wie viel Wasser die Pflanzen verdunsten: Ist es zu
niedrig, transpirieren sie kaum, nehmen wenig Nährstoffe auf und laden
Schimmel ein. Ist es zu hoch, schließen sie ihre Spaltöffnungen. Im Zelt ist
der Abluftlüfter meist das Einzige, was es bewegt. Der Lüfterregler
entscheidet also, wie nah du ans Ziel kommst und wie viel Strom und Lärm das
kostet.

Ein normaler PID-Regler auf "VPD -> Lüfter" funktioniert auf dem Papier, aber
ein Growzelt verletzt mehrere seiner Annahmen:

| Problem im Zelt | Was ein PID macht | Was dieser Regler macht |
|---|---|---|
| **Der Lüfter wirkt sehr ungleichmäßig.** Der Feuchteüberschuss fällt mit `1 / Luftstrom`: 10 -> 20 % Lüfter ändert viel, 80 -> 90 % kaum etwas. Außerdem reagiert das Zelt bei wenig Lüfter viel langsamer (`tau / Luftstrom`). | Feste Verstärkungen passen nur zu einem Arbeitspunkt: bei niedriger Drehzahl zu aggressiv (schwingt), bei hoher zu träge. | Arbeitet auf einer logarithmischen Luftstrom-Skala mit einer Verstärkung, die dem Luftstrom folgt (`k = speed * q / tau`), und verhält sich dadurch bei jeder Drehzahl gleich. |
| **Das Ziel ist oft unerreichbar**, z. B. wenn die Raumluft feucht ist oder die Pflanzen viel transpirieren. | Der I-Anteil läuft hoch und der Lüfter läuft stundenlang auf 100 %, für ein paar Hundertstel kPa. | Schätzt die Feuchtelast und bleibt beim **sinnvollen Maximum** stehen, ab dem mehr Lüfter weniger VPD bringt, als du aufzugeben bereit bist. Spart viel Energie, besonders im Winter. |
| **Die Raumluft kommt 1:1 herein.** Jede Änderung im Raum zeigt sich im Zelt. | Merkt es erst, wenn sich das VPD schon bewegt hat, und schießt dann über oder unter. | Misst die Raumluft und arbeitet mit dem Dampfdrucküberschuss `Zelt - Raum`, sodass Änderungen im Raum nicht wie Regelfehler aussehen. |
| **Lichtwechsel sind große, vorhersehbare Sprünge** bei Temperatur und Transpiration. | Jagt dem Sprung von der alten Drehzahl aus hinterher, schießt über und braucht lange zum Einschwingen. | Springt direkt auf die Drehzahl, die er sich für diese Lichtphase gemerkt hat, und führt das Ziel weich nach (*Sollwert-Übergang*), statt gegen eine Lücke anzukämpfen, die sich von selbst schließt. |
| **Verrauschte Sensoren.** | Der D-Anteil verstärkt das Rauschen, deshalb wird er oft abgeschaltet. | Ein Kalman-Filter trennt Sensorrauschen von echten Laständerungen und begrenzt Ausreißer. |
| **Zelt offen, Sensorausfälle.** | Läuft auf sinnlosen Messwerten hoch. | Hält den Lüfter, solange das Zelt offen ist, wechselt auf einen Regler ohne Raumsensor, solange der Raumsensor ausfällt, und fährt eine Notdrehzahl, wenn der Zeltsensor ausfällt. |
| **Zu heiß oder zu feucht.** | Hat nur ein Ziel. | Eine eigene Sicherheitsebene übersteuert den Regler oberhalb von Temperatur- und Feuchtegrenzen und meldet das. |

Kurz gesagt: Ein PID reagiert nur auf die Abweichung. Dieser Regler weiß
außerdem, *warum* das VPD abweicht (Raumluft, Pflanzenlast, Lichtphase) und
ob mehr Lüfter überhaupt helfen würde. Dadurch kommt das VPD nah ans Ziel,
ohne dass der Lüfter umsonst auf Vollgas läuft.

## Was es ist

Eine externe Komponente für [ESPHome](https://esphome.io), die das
**VPD (Dampfdruckdefizit)** eines Growzelts mit einem einzelnen Abluftlüfter
regelt.

- Modellbasierter Regler mit **Kalman-Filter**, der schätzt, wie viel
  Feuchtigkeit die Pflanzen abgeben, statt eines PID
- Findet das **sinnvolle Maximum**: Er dreht den Lüfter nicht weiter hoch,
  sobald mehr Luft weniger VPD bringen würde, als du aufzugeben bereit bist
  (spart Energie und Lärm, besonders im Winter)
- Ein **VPD-Ziel**, oder mit Tag/Nacht-Sensor getrennte **Tag- und
  Nachtziele**, weicher **Sollwert-Übergang** nach Lichtwechseln und
  gemerkte Lüfterdrehzahl je Lichtphase, siehe [Tag/Nacht-Sensor](#tagnacht-sensor-optional)
- **Temperatur- und Feuchteschutz**, der den Regler übersteuert, mit
  einer Benachrichtigung pro Ereignis
- Schalter **Tent open** (Zelt offen), der den Regler pausiert, während du im Zelt
  arbeitest
- Blatttemperatur von einem IR-Sensor (z. B. MLX90614) oder über einen festen
  Offset (Tag/Nacht bei Tag/Nacht-Sensor)
- Optionaler zweiter Zeltsensor (z. B. über ESP-NOW) per Lambda
- **Ersatzregler**, der nur den Zeltsensor braucht: Er übernimmt
  automatisch, solange der Raumsensor ausfällt, oder läuft allein, wenn du
  keinen Raumsensor hast, siehe [Ersatzregler](#ersatzregler)
- Jede Einstellung ist eine Home-Assistant-Entität, jeder interne Wert kann
  als Diagnosesensor ausgegeben werden
- Optionale [Tuning-Parameter](#tuning-parameter-optional) für andere Aufbauten,
  als feste Werte oder als Entitäten, nur wenn du sie hinzufügst
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

### Die Entwicklungsversion testen

Neue Funktionen werden auf dem Branch `dev` getestet, bevor sie als Tag
erscheinen. Um sie auszuprobieren, nimm statt des Tags den Branch-Namen:

```yaml
external_components:
  - source: github://Malvus524/esphome-vpd-regler@dev
    components: [vpd_kalman]
    refresh: 1h                # wie oft ESPHome nach neuen Commits schaut
```

`dev` kann unfertig sein und sich jederzeit ändern oder kaputtgehen. Für ein
Zelt, das unbeaufsichtigt laufen muss, bleib bei einem Tag. Um zurückzugehen,
trag wieder den Tag ein.

## Minimale Konfiguration

```yaml
vpd_kalman:
  output: fan_pwm              # Float-Ausgang, der den Lüfter ansteuert
  temperature: tent_temperature
  humidity: tent_humidity
  room_temperature: room_temperature
  room_humidity: room_humidity
  night: lights_off            # optional: Binärsensor, ON = Nacht
  language: de                 # deutsche Zustandstexte und Meldungen
```

Das legt alle Einstellungen und Schalter an, mit englischen Namen (eigene
Namen: siehe [Namen der Entitäten](#namen-der-entitäten)).
Ohne `night` gibt es ein *VPD target*, mit `night` ein Tag- und ein Nachtziel,
siehe [unten](#tagnacht-sensor-optional). Der Regler startet im
**Handbetrieb**: Schalte den Schalter *VPD control* ein, damit er den Lüfter
übernimmt. Ein vollständiges Beispiel mit Sensoren, Diagnose und
Benachrichtigungen steht in [example.yaml](example.yaml).

### Tag/Nacht-Sensor (optional)

Ohne `night` läuft der Regler immer in einer Phase: ein *VPD target*, ein
*Leaf offset*, kein Warten auf Tag/Nacht nach dem Start. Der
Sollwert-Übergang läuft trotzdem nach dem Schließen des Zelts und nach einer
Zieländerung.

```yaml
vpd_kalman:
  output: fan_pwm
  temperature: tent_temperature
  humidity: tent_humidity
  room_temperature: room_temperature
  room_humidity: room_humidity
  target:
    name: "VPD Ziel"
    initial_value: 1.1
```

Mit `night` bekommst du getrennte Ziele für Tag und Nacht. Lichtwechsel sind
große, vorhersehbare Sprünge, deshalb springt der Regler dann außerdem auf die
Drehzahl, die er sich für die neue Lichtphase gemerkt hat, startet einen
Sollwert-Übergang und nutzt den Blatt-Offset der Phase. Willst du ein Ziel,
aber diese Vorteile, füge zusätzlich `target` hinzu: Es ersetzt *VPD target
day/night*, die Blatt-Offsets bleiben getrennt.

```yaml
vpd_kalman:
  # ...
  night: lights_off
  target:
    name: "VPD Ziel"
```

| | ohne `night` | mit `night` | mit `night` + `target` |
|---|---|---|---|
| Ziele | `target` | `target_day`, `target_night` | `target` |
| Blatt-Offsets | `leaf_offset` | `leaf_offset_day`, `leaf_offset_night` | `leaf_offset_day`, `leaf_offset_night` |
| Lüfterdrehzahl je Lichtphase, Übergang bei Lichtwechsel | - | ja | ja |

Schlüssel der jeweils anderen Variante werden abgelehnt, ebenso ohne `night`
die Tuning-Parameter `light_memory_after`, `light_memory_delay`,
`boot_wait_night` und die `storage_keys` `day`/`night`.

## Namen der Entitäten

Jede Entität hat im YAML einen Schlüssel (erste Spalte der Tabellen unten).
Die Schlüssel bleiben englisch, die Namen legst du selbst fest. Es gibt zwei
Arten:

- **Einstellungen und Schalter** werden immer angelegt, mit dem englischen
  Standardnamen aus den Tabellen. Um einen umzubenennen, schreib seinen
  Schlüssel mit `name` dazu.
- **Diagnosen, der Schalter `force_fallback` und Tuning-Entitäten** sind
  optional. Es gibt sie nur, wenn du ihren Schlüssel mit `name` hinzufügst.

```yaml
vpd_kalman:
  # ...
  language: de

  # Immer angelegt, nur der Name ändert sich
  control:
    name: "VPD Regelung"
  tent_open:
    name: "Zelt offen"
  fan_max:
    name: "Lüfter Maximum Automatik"
  manual_speed:
    name: "Lüfter Handdrehzahl"
    id: luefter_hand             # optional, für eigene Lambdas

  # Optional, nur angelegt, weil sie hier stehen
  fan_output:
    name: "Lüfter Ausgang"
  state:
    name: "VPD Regler Zustand"
  force_fallback:                # nur mit Raumsensor
    name: "Ersatzregler nutzen"
  tuning:
    fallback_rate:
      name: "Ersatzregler Rate"
```

Gut zu wissen:

- Schlüssel, die du weglässt, behalten ihren Standardnamen (Einstellungen,
  Schalter) oder werden nicht angelegt (Diagnosen).
- Neben `name` akzeptiert jede Entität die üblichen ESPHome-Optionen wie
  `id`, `icon`, `entity_category` oder `disabled_by_default`, Einstellungen
  außerdem `min_value`, `max_value`, `step` und `initial_value`. `id`
  brauchst du nur für eigene Lambdas (`id(luefter_hand).state`), Home
  Assistant sieht sie nicht.
- Leg die Namen fest, bevor du dich auf die Entitäten verlässt. Home
  Assistant bildet die Entitäts-ID aus Gerätename und Name, und auch der im
  Flash gespeicherte Wert hängt am Namen. Änderst du einen `name` später,
  legt Home Assistant eine neue Entität an (der Verlauf bleibt bei der
  alten), Einstellungen beginnen wieder bei `initial_value` und Schalter bei
  ihrem Standard aus der Spalte *Wiederherstellung*.
- Ziele und Blatt-Offsets: Benenne die Schlüssel deiner Variante um, siehe
  [Tag/Nacht-Sensor](#tagnacht-sensor-optional). `target` zusätzlich zu
  `night` benennt nicht nur um, sondern schaltet auf ein Ziel für Tag und
  Nacht.
- Tuning-Parameter gehen ohne Namen auch als fester Wert, siehe
  [Tuning-Parameter](#tuning-parameter-optional). Erst der Block mit `name`
  legt eine Entität an.

---

## Konfigurationsreferenz

Die Schlüssel sind englisch und bleiben so. In den Tabellen steht unter
*Standardname* der Name, den die Entität ohne eigenes `name` bekommt.

### Eingänge

| Schlüssel | Pflicht | Beschreibung |
|---|---|---|
| `output` | ja | Float-Ausgang des Lüfters (z. B. `ledc`). Der Regler schreibt 1-100 %, er schaltet den Lüfter nie aus. Mit `min_power`/`max_power` des Ausgangs passt du das an deinen Lüfter an. |
| `night` | nein | Binärsensor, ON = Nacht (Licht aus). Tag/Nacht wählt Ziel, Blatt-Offset und gemerkte Lüfterdrehzahl. Bis er nach dem Start einen Zustand hat, hält der Regler den Lüfter (max. 3 min). Ohne ihn gibt es nur ein Ziel und einen Blatt-Offset, siehe [Tag/Nacht-Sensor](#tagnacht-sensor-optional). |
| `temperature`, `humidity` | ja | Luftsensor im Zelt. |
| `leaf_temperature` | nein | Blatttemperatursensor (IR). Wird genutzt, solange der Schalter *Leaf temperature from sensor* an ist und der Wert höchstens `leaf_max_deviation` von der Lufttemperatur abweicht. Sonst Lufttemperatur + Blatt-Offset. |
| `room_temperature`, `room_humidity` | nein | Luft, die der Lüfter ansaugt. Beide oder keiner. Mit ihnen läuft der Kalman-Regler, und der [Ersatzregler](#ersatzregler) übernimmt, solange sie ausfallen. Ohne sie läuft nur der Ersatzregler. |
| `external_climate` | nein | Lambda, das `vpd_kalman::ExternalClimate` mit `temperature`, `humidity`, `leaf_temperature` (und `selected` fürs Log) zurückgibt. Sind alle drei gültig, werden sie statt der eigenen Sensoren genutzt, z. B. für einen zweiten Sensor auf Höhe des Blätterdachs. Siehe unten. |

### Optionen

| Schlüssel | Standard | Beschreibung |
|---|---|---|
| `airflow_at_zero` | `0.2` | Luftstrom des Lüfters bei 0 % im Verhältnis zu 100 % (`q0`). Geht in die Feuchtelast und das sinnvolle Maximum ein. Der Ersatzregler nutzt ihn für seine logarithmische Lüfterskala. |
| `time_constant` | `21s` | Zeit, die das Zelt bei 100 % Lüfter braucht, bis 63 % einer Feuchteänderung abgeführt sind (`tau`). Legt die Filterdynamik fest und, zusammen mit *Controller speed*, wie schnell der Regler eingreift. Nur Kalman-Regler. |
| `leaf_max_deviation` | `6.0` | Blattsensorwerte, die weiter als dieser Wert (°C) von der Lufttemperatur entfernt sind, gelten als Messfehler. |
| `language` | `en` | `en` oder `de`: Zustandstexte, Benachrichtigungen und Logzeilen. |
| `storage_keys` | - | Nur für den Umstieg von einer älteren YAML-Lösung: Gib `controller_output`, `day` und `night` die IDs deiner wiederherstellenden `globals:`, damit deren gespeicherte Werte erhalten bleiben. |
| `tuning` | - | Optionale Tuning-Parameter, siehe [unten](#tuning-parameter-optional). |
| `on_message` | - | Automation mit `title` und `message` (`std::string`) für Benachrichtigungen. |

Der Regler läuft alle 10 s. Das ist fest, weil Filter und Regelgesetz darauf
abgestimmt sind.

### Einstellungen (Number-Entitäten)

Alle werden automatisch angelegt, im Flash gespeichert und als Eingabefeld
angezeigt. Welche Ziele und Blatt-Offsets es gibt, hängt von `night` ab, siehe
[Tag/Nacht-Sensor](#tagnacht-sensor-optional). Jede akzeptiert die üblichen
Number-Optionen (`name`, `id`, `icon`, `entity_category`, ...) sowie
`min_value`, `max_value`, `step` und `initial_value`.

| Schlüssel | Standardname | Standard | Bereich | Bedeutung |
|---|---|---|---|---|
| `manual_speed` | Fan manual speed | 50 % | 1-100 | Lüfterdrehzahl, solange *VPD control* aus ist. |
| `target` | VPD target | 1.2 kPa | 0.3-2.5 | Ohne `night`, oder mit `night`, wenn du diesen Schlüssel hinzufügst: ein Ziel für Tag und Nacht. |
| `target_day` | VPD target day | 1.2 kPa | 0.3-2.5 | Nur mit `night`. |
| `target_night` | VPD target night | 1.0 kPa | 0.3-2.5 | Nur mit `night`. |
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
| `leaf_offset` | Leaf offset | -2.0 °C | -10-5 | Blatttemperatur = Luft + Offset, wenn kein gültiger Blattsensorwert genutzt wird. Nur ohne `night`. |
| `leaf_offset_day` | Leaf offset day | -2.0 °C | -10-5 | Offset in der Tagphase. Nur mit `night`. |
| `leaf_offset_night` | Leaf offset night | -1.0 °C | -10-5 | Nur mit `night`. |

### Schalter

Die Spalte *Wiederherstellung* sagt, wie ein Schalter nach einem Neustart
(Stromausfall, OTA-Update, Reset) steht. *Standard aus* und *Standard an*
heißen: Der Schalter steht wieder so wie vor dem Neustart. Aus bzw. an gilt
nur, wenn noch nichts gespeichert ist, also beim allerersten Start oder
nachdem du seinen `name` geändert hast. *immer aus* heißt: nach jedem
Neustart aus. Ändern kannst du das mit der üblichen Schalter-Option
`restore_mode`.

| Schlüssel | Standardname | Wiederherstellung | Bedeutung |
|---|---|---|---|
| `control` | VPD control | Standard aus | ON = der Regler steuert den Lüfter (übernimmt die aktuelle Drehzahl ohne Sprung). OFF = Handdrehzahl. |
| `tent_open` | Tent open | immer aus | Hält den Lüfter, pausiert Filter, Merken und Grenzfinder, die Sicherheit bleibt aktiv. |
| `leaf_sensor` | Leaf temperature from sensor | Standard an | `leaf_temperature` statt der Offsets nutzen. |
| `force_fallback` | - (optional, nur mit `name`) | Standard aus | ON = der [Ersatzregler](#ersatzregler) läuft, obwohl der Raumsensor funktioniert, z. B. um beide Regler zu vergleichen. Nur mit Raumsensor. |

### Diagnose (optional)

Wird nur angelegt, wenn du den Schlüssel (mit mindestens einem `name`)
hinzufügst, siehe [Namen der Entitäten](#namen-der-entitäten). Alle
akzeptieren die üblichen Sensor-Optionen.

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
| `learned_airflow_50` | % | [Lüfterkennlinie lernen](#lüfterkennlinie-lernen): Luftstrom bei 50 % Lüfter in % des vollen. |
| `learned_sensor_offset` | % | Gelernter Versatz des Zeltsensors gegenüber dem Raumsensor (%rF). |
| `learned_sensor_lag` | s | Gelernte Trägheit des Zeltfühlers. |
| `learned_sensible_max` | % | Sinnvolles Maximum mit der gelernten Kennlinie, auch solange sie noch nicht genutzt wird. |
| `state` | Text | Mit `language: de` z. B. *Im Band*, *Regeln*, *An sinnvollem Maximum*, *Ziel unerreichbar*, *Sicherheit (Temperatur)*, *Zelt offen - pausiert (5 min)*. |
| `temperature_protection` | binär | ON, solange der Temperaturschutz den Lüfter anhebt. |
| `humidity_protection` | binär | ON, solange der Feuchteschutz den Lüfter anhebt. |
| `fallback_active` | binär | ON, solange der Ersatzregler den Lüfter steuert. |
| `fan_curve_learned` | binär | ON, solange das sinnvolle Maximum die gelernte Kennlinie nutzt. |
| `limit_finder_drift` | kPa/min | Ersatzregler: VPD-Drift, gemessen vor dem letzten Testschritt. |
| `limit_finder_vpd_change` | kPa | Ersatzregler: Wirkung des letzten Testschritts, ohne die Drift. |
| `limit_finder_cost_before` | - | Ersatzregler: `J` vor dem letzten Testschritt. |
| `limit_finder_cost_after` | - | Ersatzregler: `J` nach dem letzten Testschritt (mit Marge). |

---

## Lüfterkennlinie lernen

Der Kalman-Regler nimmt eine gerade Lüfterkennlinie an (`airflow_at_zero`)
und dass Zelt- und Raumsensor gleich messen. Viele Lüfter fördern schon weit
unter 100 % fast ihre volle Luftmenge, und zwei Feuchtesensoren weichen oft
um einige %rF voneinander ab. Beides verschiebt das *sinnvolle Maximum*. Die
Komponente lernt beides selbst, ohne Testläufe und ohne Einstellung:

- Eine Bank aus 720 kleinen Filtern läuft mit, jeder eine Annahme über
  Kennlinie, Versatz des Zeltsensors (-4 ... +4 %rF) und Trägheit des
  Zeltfühlers (0-60 s), immer mit dem eigenen Zeltsensor. Jeder sammelt
  Punkte dafür, wie gut er den nächsten Messwert vorhersagt, aber nur in den
  10 min nach einer Lüfteränderung und nur, wenn nichts Unerwartetes passiert
  ist (Zelt ohne Schalter geöffnet, Gießen, Lichtwechsel). Die Punkte
  verblassen mit einer Halbwertszeit von 48 h und überstehen einen Neustart.
- Die gelernte Kennlinie wird erst genutzt, wenn sie klar besser vorhersagt
  als die eingestellte und 48 h stabil war. Dann nutzt sie nur das sinnvolle
  Maximum, das Regeltempo bleibt. Bis dahin arbeitet der Regler genau wie
  ohne Lernen.
- Das dauert einige Tage. Ein Lüfter, der zu `airflow_at_zero` passt, wird
  nie übernommen.

Optionale Diagnosen zeigen, was sie gelernt hat: `learned_airflow_50`,
`learned_sensor_offset`, `learned_sensor_lag`, `learned_sensible_max` (auch
solange sie nicht genutzt wird, zum Vergleichen) und `fan_curve_learned`.

## Ersatzregler

Der Kalman-Regler braucht die Raumluft. Für den Fall, dass sie fehlt, enthält
die Komponente einen zweiten, einfacheren Regler, der nur den Zeltsensor
braucht. Er ist immer Teil der Firmware:

- **Raumsensor eingerichtet:** Der Kalman-Regler läuft. Hat der Raumsensor
  2 min lang keinen gültigen Wert, übernimmt der Ersatzregler. Nach 1 min
  mit wieder gültigen Werten übernimmt der Kalman-Regler erneut. Beide
  Wechsel senden eine Benachrichtigung (`on_message`), der Zustandstext
  bekommt *(ohne Raumsensor)* (englisch *(no room sensor)*), solange der Ersatzregler läuft. Kürzere
  Aussetzer halten nur den Lüfter.
- **Schalter `force_fallback` (optional):** ON übergibt sofort an den
  Ersatzregler, der Raumsensor zeichnet weiter auf, sodass sich beide Regler
  auf denselben Daten vergleichen lassen. OFF gibt sofort zurück (nach der
  Rückkehrverzögerung, falls der Raumsensor gerade fehlt). Keine
  Benachrichtigung, der Zustandstext bekommt *(Ersatzregler)* (englisch *(fallback)*).

  ```yaml
  vpd_kalman:
    # ...
    force_fallback:
      name: "Ersatzregler nutzen"
  ```
- **Kein Raumsensor eingerichtet:** Nur der Ersatzregler läuft.

  ```yaml
  vpd_kalman:
    output: fan_pwm
    night: lights_off
    temperature: tent_temperature
    humidity: tent_humidity
  ```

Beim Wechsel wird der Lüfter ohne Sprung übernommen, Sicherheitsereignisse,
*Tent open* und die gemerkten Ereignisse laufen weiter. Der Ersatzregler
beginnt seine obere Grenze beim letzten sinnvollen Maximum des
Kalman-Reglers. Zurück im Kalman-Regler wird der Filter neu auf die Messung
abgeglichen und behält die geschätzte Last.

Ohne die Raumluft kann der Ersatzregler die Feuchtelast nicht schätzen. Er
arbeitet in drei Ebenen, alle 10 s:

1. **Grundregler**: ein PI-Regler auf einer logarithmischen Lüfterskala,
   `x = ln(u + u0)` mit `u0 = 100 * q0 / (1 - q0)` aus `airflow_at_zero`.
   Gleiche Schritte in `x` sind gleiche relative Luftstromänderungen.
   Innerhalb des Totbands passiert nichts.
2. **Grenzfinder**: Bleibt das VPD unter dem Band, während der Lüfter
   3 Zeitkonstanten lang an seiner oberen Grenze steht, probiert er aus, ob
   eine niedrigere (oder höhere) Grenze besser ist. Er misst die VPD-Drift,
   verschiebt die Grenze um einen Testschritt, wartet 3 Zeitkonstanten und
   vergleicht `J = (Abweichung / 0.1 kPa)^2 + Lüfterkosten * (Lüfter / 100 %)^2`
   vorher und nachher, mit herausgerechneter Drift. Der Schritt wird
   behalten, wenn `J` deutlich kleiner wird (Marge 0.01 kPa), sonst geht die
   Grenze zurück und der nächste Test wartet doppelt so lange (max. 60 min).
   Das ist die Variante des sinnvollen Maximums im Ersatzregler. Die Grenze
   wird nach einem Lichtwechsel, einer Zieländerung, einem Wechsel des
   Zeltsensors und beim Einschalten der Regelung auf *Fan maximum automatic*
   zurückgesetzt.
3. **Sicherheit**: dieselbe wie im Kalman-Regler.

Das geregelte VPD ist der Mittelwert der letzten 30 s. *Tent open*, der
*Sollwert-Übergang* (nach Lichtwechseln, dem Schließen des Zelts und einem
Wechsel des Zeltsensors), Handbetrieb, Benachrichtigungen, Blatttemperatur
und `external_climate` funktionieren wie im Kalman-Regler. Seine
Einstellungen (Zeitkonstante, Rate, Lüfterkosten, Testschritt) sind
[Tuning-Parameter](#tuning-parameter-optional).

Ohne Raumsensor werden die Einstellungen *Allowed VPD sacrifice* und
*Controller speed* nicht angelegt und `time_constant` wird ignoriert. Die
Diagnosen, die nur der Kalman-Regler hat (`excess`, `excess_target`,
`moisture_load`, `next_step_benefit`, `vpd_at_max`, die Diagnosen der
Lüfterkennlinie), und seine
Tuning-Parameter werden abgelehnt.

## Tuning-Parameter (optional)

Für Aufbauten, die von den Standardwerten abweichen. Lässt du einen Parameter
weg, gilt der eingebaute Standard und es wird keine Entität angelegt. Ein
einfacher Wert legt ihn fest (bei Zeiten auch `90s`, `2min`), ein Block mit
`name` macht ihn zu einer Number-Entität unter *Konfiguration*, die du live
ändern kannst (er akzeptiert die üblichen Number-Optionen, `initial_value`
ist standardmäßig der Standardwert):

```yaml
vpd_kalman:
  # ...
  tuning:
    room_fallback_delay: 5min          # fest
    sensor_noise: 0.01                 # fest
    fallback_rate:                     # Entität
      name: "Ersatzregler Rate"
```

| Schlüssel | Einheit | Standard | Bereich | Regler | Bedeutung |
|---|---|---|---|---|---|
| `room_fallback_delay` | min | 2 | 0.5-60 | Wechsel | Wie lange der Raumsensor fehlen darf, bevor der Ersatzregler übernimmt. |
| `room_return_delay` | min | 1 | 0.5-60 | Wechsel | Wie lange der Raumsensor wieder liefern muss, bevor der Kalman-Regler zurück ist. |
| `light_memory_after` | min | 20 | 0-240 | Kalman | Zeit nach einem Lichtwechsel, bevor der Lüfter dieser Lichtphase gemerkt wird. Nur mit `night`. |
| `light_memory_delay` | min | 5 | 0.5-60 | Kalman | Gemerkt wird der Wert von vor dieser Zeit, damit das späte Ende einer Lichtphase ihn nicht verdirbt. Nur mit `night`. |
| `boot_wait_tent` | s | 60 | 10-600 | beide | Nach dem Start den Lüfter so lange halten, solange noch kein Zeltwert da ist. |
| `boot_wait_night` | s | 180 | 10-1800 | beide | Nach dem Start den Lüfter so lange halten, solange Tag/Nacht unbekannt ist. Nur mit `night`. |
| `sensible_max_rate` | %/min | 5 | 0.1-100 | Kalman | Wie schnell sich das sinnvolle Maximum ändern darf. |
| `sensor_noise` | kPa | 0.0063 | 0.001-0.1 | Kalman | Messrauschen des Dampfdrucküberschusses. Höher = der Filter vertraut einzelnen Messwerten weniger. |
| `load_change_per_hour` | %/h | 10 | 1-200 | Kalman | Wie schnell sich die Feuchtelast ändern darf. Höher = die Lastschätzung folgt schneller, aber unruhiger. |
| `temperature_hysteresis` | K | 0.5 | 0-5 | beide | Der Temperaturschutz schaltet erst so weit unter seiner Grenze wieder ab. |
| `humidity_hysteresis` | % | 3 | 0-20 | beide | Dasselbe für den Feuchteschutz. |
| `all_clear_after` | min | 60 | 1-1440 | beide | Ein Sicherheitsereignis endet (Entwarnung) nach dieser Zeit ohne Auslösung. |
| `fallback_time_constant` | min | 2 | 0.5-10 | Ersatz | Wie schnell das VPD auf eine Lüfteränderung reagiert. Legt den P-Anteil und alle Zeiten des Grenzfinders fest (jeweils 3 Zeitkonstanten). |
| `fallback_rate` | %/min | 10 | 1-100 | Ersatz | Relative Lüfteränderung pro Minute bei 0.1 kPa außerhalb des Totbands. |
| `limit_finder_cost` | - | 4 | 0-20 | Ersatz | Gewicht des Lüfters in `J`. Höher = die Grenze wird eher gesenkt. |
| `limit_finder_step` | % | 15 | 5-50 | Ersatz | Größe eines Testschritts, relativ zum Luftstrom. |
| `fallback_smoothing` | s | 30 | 10-300 | Ersatz | Mittelungszeit des geregelten VPD. |
| `limit_test_margin` | kPa | 0.01 | 0-0.1 | Ersatz | Wie deutlich ein Testschritt besser sein muss, um behalten zu werden. |
| `limit_test_max_pause` | min | 60 | 5-480 | Ersatz | Längste Wartezeit zwischen zwei Tests nach verworfenen Schritten. |

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

## Lizenz

[MIT](LICENSE)
