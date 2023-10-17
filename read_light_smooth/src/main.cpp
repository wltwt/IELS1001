/*
  Funksjoner.

  Koden består av to hovedfunksjoner som man kan bytte på med seriell kommunikasjon.

  Funksjonene utfører samme oppgave men på forskjellig vis.
  Den ene bruker ett array til å regne ut gjenomsnittet av de fem største
  verdiene som lys-sensoren har lest, den andre bruker kun enkel logikk
  rundt modulo operasjon til å lese n verdier, legge de sammen og dele på antall
  avlest (n). 

  Kretsen består av én lys-sensor koblet til analog inn porten til Arduinoen.
  Det går 5V fra Arduinoen inn til ene benet av lys-sensoren, andre benet er 
  koblet til en 10k ohm motstand som deretter er koblet i jord.

  Skrevet av: Olav Telneset
  GitHub: https://github.com/wltwt
*/

// inkluderer Arduino hovedbiblioteket for å få tilgang til arduino-spesifikke funksjoner
#include <Arduino.h>

// setter opp enkelt objekt for å lagre pin-informasjon
struct Pin {
  const int photocell = A0;
};

const int size = 5;                 // setter størrelse til array
int container[size];                // oppretter tomt array
Pin pin;                            // oppretter Pin objekt
unsigned long readPrev = 0;         // brukes som sjekk i interval
int withArrayValues;                // for lagring av retur-verdien
int withoutArrayValues;             // for lagring av retur-verdien
static bool printWithArray = false; // avgjør hvilken funksjon vi printer fra

/*
  Tar inn en liste og avlesningsverdien til en sensor som argumenter.

  Alt av variabler som brukes er lokale til funksjonen, men de blir ikke
  slettet fra minnet etter funksjonen har kjørt ferdig fordi vi bruker
  static nøkkelordet.

  Først trekker den fra siste elementet som ble lagt til, deretter
  legger den inn en ny verdi i listen som blir lagt til i totalen.
  Deretter sender vi tilbake gjenomsnittet og inkrementerer en
  plass frem i listen.
*/
int read_sensor_with_array(int *container, int readValue) {
  static unsigned int i = 0;  // oppretter index start
  static int total;           // variabel for totalen
  static int average;         // variabel for gjenomsnitt

  // leser siste verdi som ble lagt til og trekker fra totalen
  total -= container[i % size];

  // legger til ny verdi på samme plass
  container[i % size] = readValue;

  // legger til verdien i totalen
  total += container[i % size];

  // regner ut gjenomsnittet
  average = total / size;

  // inkrementerer indexen til listen
  i++;

  // returnerer verdien som ble regnet ut
  return average;
}


//  Tar inn verdien som ble avlest fra lys-sensoren og returnerer gjenomsnittet basert på tidligere avlesinger.
int read_without_array(int readValue) {
  static unsigned int j = 0;                // oppretter index
  static unsigned int totalWithoutArray;    // lokal variabel for total
  static unsigned int averageWithoutArray;  // gjenomsnittsvariabel
  
  // legger til verdier som er avlest for hver runde
  totalWithoutArray += readValue;

  // om det har gått n runder finner vi gjenomsnittet av de tidligere målingene her
  if (j % size == 0) {
    
    // legger sammen totalen og deler på n (5 i dette tilfellet)
    averageWithoutArray = totalWithoutArray / size;
    
    // resetter total-verdien siden vi kun er interessert i de 5 siste avlesingene
    totalWithoutArray = 0;

    // koden når aldri neste inkrementering så indexen må økes her og
    j++;
  
    // returnerer gjenomsnittsverdien
    return averageWithoutArray;  
  }
  
  // inkrementerer rundetall
  j++;

  // returnerer 0 om ikke gjenomsnittsverdien er funnet enda
  return 0;
}

// liten funksjon som tar seg av printing av en beskjed med et gitt interval
void print(unsigned long timer, int message) {
  unsigned long now = millis();
  static unsigned long printerTimer = 0;
  if (now - printerTimer >= timer) {
    printerTimer = millis();
    Serial.println(message);
  }
}

// oppsett
void setup() {
  // må initialisere listen med 0'ere før vi kan bruke den
  for (int i = 0; i < size; i++) {
    container[i] = 0;
  }
  
  // start seriell kommunikasjon
  Serial.begin(9600);
}

void loop() {
  int incomingByte = 0; // avlesingvariabel for seriellmonitor

  // avleser lys-sensoren med 2ms tidsintervall for bedre avlesning
  if (millis() - readPrev >= 2) {

    // setter tiden koden gikk inn sist til en variabel
    readPrev = millis();

    // avleser lys-sensoren og legger den i en variabel som kan brukes av begge funksjonene
    int read = analogRead(pin.photocell);

    // legger retur-verdien fra les med liste funksjone i en variabel
    withArrayValues = read_sensor_with_array(container, read);

    /*
      Siden read_without_array egentlig kun returnerer oppdatert tall
      hver n'te gang gjør vi en liten sjekk slik at den ikke skriver ut
      de gangene den ikke har et nytt oppdatert tall.
    */
    int check = read_without_array(read);
    
    // sjekker om avlest verdi er 0
    if (check != 0) {
    
      // legger oppdatert verdi i en variabel
      withoutArrayValues = check;
    }
  }

  // setter opp hvilken funksjon vi vil skrive ut fra, uten array er standard
  if (printWithArray) {
    print(100,withArrayValues);
  } else {
    print(100,withoutArrayValues);
  }

  // leser om brukeren har sendt ny informasjon
  if (Serial.available() > 0) {
    // legger informasjonen i en variabel
    incomingByte = Serial.read(); 
  }

  // switch operasjon som sjekker om informasjonen som ble sendt er y eller n
  switch (incomingByte) {

  // om y skal outputen i seriemonitoren være fra funksjonen som bruker array om n så motsatt
  case 'y':
    printWithArray = true;
    break;
  case 'n':
    printWithArray = false;
    break;
  }
}