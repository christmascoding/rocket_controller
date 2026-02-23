Hier ist eine Zusammenfassung der Analyse und der durchgeführten Schritte in technischem Deutsch, basierend auf der Debugging-Session mit dem Copiloten.

---

## Fehleranalyse-Protokoll: Raketen-MPC-Steuerung

### 1. Gewählter Ansatz (The Approach)

Das System nutzt eine **kaskadierte Kontrollstruktur**, um die Rakete entlang einer vordefinierten Flugbahn (Path) zu führen:

* **Outer Loop (MPC):** Ein *Model Predictive Control* Algorithmus berechnet basierend auf der aktuellen Position und dem zukünftigen Pfadverlauf die benötigte Ziel-Beschleunigung. Er fungiert als strategischer Navigator.
* **Inner Loop (Attitude Control):** Ein PD-Regler (Proportional-Derivative) wandelt diese Beschleunigungsvorgaben in spezifische Neigungswinkel (Pitch/Yaw) um.
* **Aktuierung:** Die physische Umsetzung erfolgt über das **Gimbaling** (Schwenken des Triebwerks).

### 2. Warum es nicht funktionierte (Why it failed)

Die Simulation scheiterte an einer **Inkompatibilität zwischen Missionsziel und Physik**, was zu einer sogenannten *Control Divergence* führte:

* **Aktor-Sättigung (Saturation):** Die MPC verlangte Manöver, um die horizontale Flugbahn zu halten, die eine massive Neigungsänderung erforderten. Die Hardware (Gimbal) ist jedoch im Code auf einen kleinen Winkel (z. B. ) limitiert.
* **Verlust der Steuerautorität:** Sobald der Gimbal sein Limit erreichte ("Sättigung"), konnte der Regler keine feinen Korrekturen mehr vornehmen. Die Rakete begann zu oszillieren und spiralförmig abzustürzen.
* **Unrealistische Trajektorie:** Der Pfad zwang die Rakete zu einem Sinkflug/Horizontalflug in einer Phase, in der die aerodynamischen Kräfte und der Schub sie natürlich nach oben drücken wollten. Die MPC versuchte, gegen die Physik zu "kämpfen", anstatt sie zu nutzen.
* **Fehlendes Integral (I-Anteil):** Da nur ein PD-Regler verwendet wurde, konnten konstante Abweichungen nicht eliminiert werden, was die Drift verstärkte.

### 3. Notwendige Maßnahmen zur Behebung (The Fix)

Um ein stabiles System zu erhalten, müssen folgende Punkte angepasst werden:

* **Trajektorien-Design:** Der Pfad muss "erreichbar" (*feasible*) sein. Anstatt die Rakete sofort auf eine horizontale Linie zu zwingen, sollte der Pfad eine sanfte Kurve beschreiben, die den natürlichen Pitch-Winkel der Rakete berücksichtigt.
* **Gimbal-Autorität erhöhen:** Wenn die Rakete agiler sein muss, muss der maximale Schwenkbereich des Triebwerks in der `config.py` vergrößert werden (z. B. von  auf ).
* **MPC-Constraints synchronisieren:** Die MPC muss "wissen", wie schwach der Gimbal ist. Die Constraints innerhalb des Optimierers müssen exakt die physikalischen Limits der Hardware widerspiegeln, damit die MPC keine unmöglichen Befehle sendet.
* **Dämpfung und Integral-Anteil:** Einführung eines kleinen I-Anteils (PI- oder PID-Logik) zur Korrektur von Drift, kombiniert mit einer stärkeren Dämpfung der Roll-Rate, um das "Spiraling" zu verhindern.

---

### Nächster Schritt

Soll ich für dich die **Gimbal-Limits** in der Konfiguration erhöhen oder möchtest du, dass wir gemeinsam eine **sanftere Trajektorie** (einen sogenannten *Gravity Turn*) entwerfen, damit die MPC nicht mehr gegen die Physik kämpfen muss?