/*
  Kontrollerer diverse LED-funksjoner med robust avlesing av knappetrykk. 

  Koden under blinker et gult lys av og på med jevne mellomrom på 850 ms
  og avleser registrering av knappetrykk uten ventetid.

  Kretsen består av tre farge-LED'er og én knapp.
  
  Grønn LED er koblet til digital-pin 9, gul LED er koblet til digital-pin 10
  og rød LED er koblet til digital-pin 11. 

  Knappen er koblet i en pull-up krets hvor arduinoen leser 5V konstant
  bortsett fra når knappen er trykket ned, da registrerer den 0V.

  Skrevet av: Olav Telneset
  GitHub: https://github.com/wltwt
*/

#include <Arduino.h>

// Setter opp hvilke ut-pin'er LED'ene er koblet til.
enum LED_PINS {green = 9, yellow, red, size};

// Oppretter en mal for ett "Button" objekt.
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

// kanskje unødvendig
struct State {
  int state;
  int prevState = HIGH;
};

// oppretter objekter
Button btn = {12, HIGH, 0, 25, HIGH};
Usertime freq_850 = {0, 100, 850};
State gLED, yLED, rLED;

static bool greenOn = false; 
static unsigned long gLEDtimer = 0;
int btnp = 0;
bool flag = false;


/*
  
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
    // 
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
        // endre tilstand
        button.toggle = !button.toggle;
        
        /* 
          Når koden har kommet hit vil ett komplett knappetrykk være registrert.
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
  
  Den tar inn et tids-objekt og et LED-status objekt.
  Dette gjør metoden i stand til å gjenvinnes med flere forskjellige
  LED'er og forskjellige tidsintervaller.
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

void updateLED(State &led, Button &btn) {
    led.state = btn.toggle;
    led.prevState = !led.state;
}

void setup() {
  // Initialiserer utgangene til LED-lysene.
  for (int i = green; i != size; i++)
    pinMode(i, OUTPUT);

  // Registrerer hvilken utgang arduinoen skal lytte etter knappetrykk på.
  pinMode(btn.pin, INPUT_PULLUP);
  Serial.begin(9600);
}

void loop() {
  unsigned long timeNow = millis();
  debounceButton(btn);
  updateLED(rLED, btn);
  updateLED(gLED, btn);

  if (btn.holdTime > 800) {
    btn.holdTime = 0;
    greenOn = !greenOn;
    flag = true;
  }

  // dummeste løsning men den fungerer
  if (greenOn) {
    if ( (btn.count == btnp + 1) && flag ) {
      gLEDtimer = millis();
      flag = false;
    }
    btnp = btn.count;
    if ( ((timeNow - gLEDtimer < 5000) && gLEDtimer > 0)) {
      digitalWrite(green, HIGH);
      // Serial.print(timeNow);
      // Serial.print(" - gLedTime:");
      // Serial.print(gLEDtimer);
      // Serial.print(" is : ");
      // Serial.println(timeNow - gLEDtimer);
      if ( (timeNow - gLEDtimer) > 4900 && (timeNow - gLEDtimer) < 5000) {
        greenOn = false;
        gLEDtimer = 0;
      }
    }
    digitalWrite(green, HIGH);
  } else {
    digitalWrite(green, gLED.prevState);
  }
  
  // if ( ((timeNow - gLEDtimer < 5000) && gLEDtimer > 0) && !greenOn ) {
  //     digitalWrite(green, HIGH);
  //     Serial.print(timeNow);
  //     Serial.print(" - gLedTime:");
  //     Serial.print(gLEDtimer);
  //     Serial.print(" is : ");
  //     Serial.println(timeNow - gLEDtimer);
  // } else {
  // }

  // hello?
  
  digitalWrite(red, rLED.state);


  // blinker gul LED med frekvens 850ms
  digitalWrite(yellow, blinkFrequency(freq_850, yLED));
}
