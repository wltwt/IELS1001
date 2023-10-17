/*
  Kontrollerer diverse LED-funksjoner med robust avlesing av knappetrykk. 

  Koden under blinker et gult lys av og på med jevne mellomrom på 850 ms
  og avleser registrering av knappetrykk uten ventetid.

  Kretsen består av tre farge-LED'er og én knapp.
  
  Grønn LED er koblet til digital-pin 9, gul LED er koblet til digital-pin 10
  og rød LED er koblet til digital-pin 11. 

  Knappen er koblet i en pull-up krets hvor arduinoen leser 5V konstant
  bortsett fra når knappen er trykket ned, da registrerer den 0V.

  Hoved-løkken ble muligens i overkant overfylt.

  Skrevet av: Olav Telneset
  GitHub: https://github.com/wltwt
*/

#include <Arduino.h>

// setter opp hvilke ut-pin'er LED'ene er koblet til.
enum LED_PINS {green = 9, yellow, red, size};

// oppretter en mal for ett "Button" objekt
struct Button {
  int pin;
  int lastState;
  unsigned long lastDebounceTime;
  unsigned long debounceDelay;
  int toggle;
  int state;
  int count;
  int holdTime;
};

// designer struct for tidsavhengige variabler ofte brukt i intervaller
struct Usertime {
  unsigned long now;
  unsigned long earlier;
  unsigned long timer;
};

// håndterer status til hver LED
struct State {
  int state;
  int prevState = HIGH;
};

Button btn = {12, HIGH, 0, 25, HIGH}; // lager knappeobjekt
Usertime freq_850 = {0, 100, 850};    // lager tids-objekt
State gLED, yLED, rLED;               // oppretter tilstandobjekt for LED'ene
static bool redOn = false;            // brukes til å toggle av/på modus-2
static unsigned long gLEDtimer = 0;   // timer for hvor lenge grønn LED skal være på
int btnp = 0;                         // lagrer knappetrykk
bool flag = false;                    // logisk flagg for kontrollflyt

/*
  Funksjon som håndterer alt rundt knappen.

  Tar inn referansen til ett knappe-objekt og utfører operasjoner på det.

  Disse operasjonene inkluderer registrere tilstand, antall knappetrykk
  og hvor lenge knappen er holdt inne.
*/
void debounceButton(Button &button) {

  // les verdien den nåværende verdien til knappen
  int reading = digitalRead(button.pin);

  // ser om knappen har endret tilstand
  if (reading != button.lastState) {
    /*
      Kun når knappen går opp igjen vil kondisjonen under gå av.
      Her blir det reading != LOW som signaliserer oppslipp av knappen
      siden vi bruker INPUT_PULLUP for knappen.
    */
    if (reading != LOW && button.lastDebounceTime > button.debounceDelay) {
      
      // registrer lengden knappen har vært holdt inne og trekk fra nåværende tid
      int buttonHold = millis() - button.lastDebounceTime;
      
      // skriv verdien til knapp-objektet
      button.holdTime = buttonHold;
    }
    // oppdaterer hvor lenge knappen er holdt inne
    button.lastDebounceTime = millis();
  }
  
  // om knappen er holdt inne i mer enn intervallet vi har valgt kjører den her
  if ((millis() - button.lastDebounceTime) >= button.debounceDelay) {
    
    // sjekker om den avleste verdiene korresponderer med det som er lagret fra før
    if (reading != button.state) {

      // endre tilstand på knappen til det som er avlest
      button.state = reading;

      // sjekker om knappen har endret seg siden forrige tilstand 
      if(button.state == LOW) {
        
        // toggle knappetilstand
        button.toggle = !button.toggle;
        /* 
          Når koden har kommet hit vil et komplett knappetrykk være registrert.
          Derfor inkrementerer vi knappetrykk-telleren først når vi vet knappen er
          "løftet opp" igjen.
        */ 
        button.count++;
      }
    }
  }

  // sett forrige tilstand til knappen til det som ble avlest
  button.lastState = reading;
}

/*
  Returnerer tilstand med selv-valgt frekvens.
  
  Den tar inn et objekt som holder styr på tid og et LED-status objekt.
  Dette gjør metoden i stand til å gjenvinnes med flere forskjellige
  LED'er og forskjellige tidsintervaller.

  Enkel å benytte som argument til digitalWrite() funksjonen.
*/
int blinkFrequency(Usertime &time, State &led) {
  
  // definerer tiden som er når metoden kalles
  time.now = millis();

  // time.now inkrementerer for hver runde iterasjon av koden
  if (time.now - time.earlier >= time.timer) {
    
    // setter betingelse for når neste tidsinterval skal gå av
    time.earlier = millis();
    
    // toggler led-tilstand
    led.state = !led.state;
  }
  
  // returnerer tilstand til LED'en man sendte inn
  return led.state;
}

// tar inn ett tilstand-objekt for LED'ene og Knappen(e)
void updateLED(State &led, Button &btn) {

    // sett status til LED basert på hva som er registrert i knappe-objektet
    led.state = btn.toggle;

    // toggle tilstand og sist tilstand
    led.prevState = !led.state;
}

void setup() {
  // Initialiserer utgangene til LED-lysene.
  for (int i = green; i != size; i++)
    pinMode(i, OUTPUT); // setter output på LED-pinnene

  // Registrerer hvilken utgang arduinoen skal lytte etter knappetrykk på.
  pinMode(btn.pin, INPUT_PULLUP);
  
  // start seriemonitor for debugging
  Serial.begin(9600);
}

void loop() {
  // lokal variabel slik at logikken i loop er synkronisert
  unsigned long timeNow = millis();

  debounceButton(btn);  // oppdaterer knappen
  updateLED(rLED, btn); // oppdaterer rød LED
  updateLED(gLED, btn); // oppdaterer grønn LED
  
  // Kontroll-logikk for hovedprogrammet.
  if (btn.holdTime > 800) {
    
    // resetter slik at kondisjonen er falsk ved neste iterasjon
    btn.holdTime = 0;

    // toggler tilstand til at rød led skal være på helt til nytt knappetrykk registrert, vi kan kalle det for modus-2
    redOn = !redOn;

    // boolsk verdi som brukes til logiske operasjoner under
    flag = true;
  }

  // av/på bryter for modus-2
  if (redOn) {

    // sjekker om knappen er trykket
    if ( (btn.count == btnp + 1) && flag ) {

      // setter start-tid til hvor lenge grønt lys skal være på
      gLEDtimer = millis();

      // sørger for at kondisjons-sjekken ikke kjører flere ganger
      flag = false;
    }

    // lagre nåværende knappetrykk
    btnp = btn.count;

    // se om det har gått 5 sekunder og se om timeren til grønn LED er over 0 for å unngå at den aktiveres ved oppstart
    if ((timeNow - gLEDtimer < 5000) && gLEDtimer > 0) {
      digitalWrite(green, HIGH);

      // tillater litt slingringsmonn for når vi skal avslutte modus-2 
      if ( (timeNow - gLEDtimer) > 4900 && (timeNow - gLEDtimer) < 5000) {
        
        // skrur av modus-2 selv
        redOn = false;
      }
    }

    // flag holdes høyt (sant) så lenge knappen ikke har registrert flere trykk
    if (flag) {

      // skriv høy verdi
      digitalWrite(red, HIGH);
    } else {
      
      // toggle-rød LED som vanlig
      digitalWrite(red, rLED.state);
    }
  } else {
    digitalWrite(green, gLED.prevState); // skriv tilstand til grønn LED
    digitalWrite(red, rLED.state);       // skriv tilstand til rød LED
  }

  // blinker gul LED med frekvens 850ms
  digitalWrite(yellow, blinkFrequency(freq_850, yLED));
}
