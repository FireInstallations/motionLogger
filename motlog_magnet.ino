/* The program switches off the magnet, when an integer >0 and <= 800 is received over serial Monitor or Twedge
    to activate the magnet it is necessary to switch it on with the button switch
    the button switch is able to switch the magnet off and on after two seconds respoectively
    Wiring:
    connect button switch to ground to Pin 2
    connect relais switch input to Pin 13
    connect magnet to the NC contact of the relais
    the magnet is in chain with the relais and the power supply
    December 2020
*/


bool onoff = true, oldonoff = false;
long timer;
void setup() {
  pinMode(2, INPUT_PULLUP);
  pinMode(13, OUTPUT);
  Serial.begin(9600);
  timer = millis();
  digitalWrite(13, onoff);

}

void loop() {
  // read frequency from Serial monitor and switch the magnet off
  if (Serial.available()) {
    int Input = Serial.parseInt();
    if (Input <= 800 && Input > 0 && onoff == false) {
      onoff = true;
      oldonoff = false;
      digitalWrite (13, HIGH);
      Serial.println(onoff);

    }
    Serial.flush();
  }
  // toggle the magnet status with the microswitch
  if (digitalRead(2) == LOW && onoff != oldonoff) {
    onoff = !onoff;
    // secure that toggling happens only once
    oldonoff = onoff;
    timer = millis();
    digitalWrite(13, onoff);
    Serial.println(onoff);
  }
  // dead time is two seconds
  if (millis() - timer > 2000)
    // after two seconds toggling is allowed again
    oldonoff = !oldonoff;

}
