## Kurze Erlauterung zur Anwendung und Struktur:

### Struktur:
Wir haben 3 Microcontroller, die alle ihre eigene main haben:

##### Der Mini ESP an der Dolle, der die Daten ausliest und per ESP Now verschickt (EspNow_Sender):
main_espNow_sender.cpp

##### Der ESP Wroom, der die Daten über ESP Now empfängt und per Uart weiterleitet
main_espNow_receiver.cpp

#### Der XIAO, der die Daten Empfängt, GPS Daten dazupackt und per Bluetooth LE verschickt:
main_BLE_sender.cpp

In diesen Main Files sollen nur eine setup und eine loop Methode drin sein. Der Restliche Code ist im **lib-Ordner**:
Im lib-Ordner bekommt jede Datei eine .h und eine .cpp Datei, die in ihren eigenen Ordner kommen.
Die entsprechenden Dateien, werden dann von main oder anderen unterdateien angesprochen.

#### Bitte alle möglichen Konstanten und defines in den .h Dateien machen. (korrigiert das bitte auch, falls ich das noch nicht überall gemacht habe).

Den include Ordner leer lassen.
Den Test Ordner nur mit temporären tests füllen -> nicht auf Main lassen

#### In Der Platformio.ini sind die Randbedingungen (z.B. welches Board) für alle 3 Projekte festgelegt.

### Anwendung
Bei Einbindung der PlatformIO erweiterung für VSCode, kann man einfach:
- Auf den PlatformIO Kopf links gehen
- oben links auf den Mikrocontroller gehen, den man bespielen möchte
- Build und Upload (,etc.) dort für alle 3 Setups schnell ausführen (Build compiled nur und Upload macht compile (soweit nötig) und lädt das auf den Mikrocontroller.

#### Den richtigen Mikrocontroller ansteuern
Falls ihr an einen Rechner mehrere Mikrocontroller angeschlossen habt, müsst ihr folgende Zeilen in die entsprechenden environments einfügen:
upload_port = /dev/ttyUSB0
monitor_port = /dev/ttyUSB0 (<- nur falls ihr was monitoren wollt)
die sind jeweils auskommentiert bereits da.
Um herauszufinden welche Ports ihr da reinschreiben müsst, müsst ihr auf den PlatformIO Kopf links gehen -> Devices -> auf der letzten Seite sind die angesteckten Ports angegeben.

#### Wichtig:
Für die praktische Asicht links, mit den mehreren Projekten, muss man in Vs Code die Option "Auto Preload Env Task2 anmachen ((STRG + ,) und dann danach suchen)


## Offene TODOS:

- AngleReader Code testen (wurde aus matlab code umgeschrieben, aber noch nicht getestet)
- ForceReader Code einfügen und testen
- die Aufrufe von ReadForce und ReadAngle in Sensor.cpp auskommentieren, sobald die laufen und das machen, was sie sollen.
- Die richtigen Ports herausfinden und diese an die richtigen funktionen übergeben (idk welche das sind) (glaub beim Angle Reader wurde das schon gemacht)
- Testen wie lange ForceReader und AngleReader brauchen. Wichtig : Ist die Eventkette von readForce -> readAngle -> sendData() kurz genug, sodass sie alle 5 Millisekunden stattfinden kann? (Bisher wird alle 5 Millisekunden ein Timer ausgelöst, der die 3 Events in eine Eventqueue packt, die dann in der Main abgefrühstückt wird.
- Testen, ob die Kommunikation vom EspNow-Sender bis zum BLE_sender funktioniert (bisher nur einzeln getestet) -> da ist auch noch Code drin, der dummy Daten erzeugt -> den dann rausmachen
- Die Empfangenen Daten nicht nur über Serial ausprinten, sondern auch über BLE versenden und schauen, ob was ankommt :)
- Am besten ein bisschen Spaß dabei haben.


Hoffe ich hab an alles gedacht -> viel Erfolg und bis Montag (da bin ich wieder den Tag über voll dabei <3)
LG Jake


