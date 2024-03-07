#define LED_PIN 13
volatile unsigned long interruptCounter = 0;
volatile unsigned long previousMillis = 0;
volatile unsigned long elapsedTime = 0;

// This function initiates timer 1 and its interrupts
inline void init_timer1() {
    TCCR1A = 0; //Init Timer1A
    TCCR1B = 0; //Init TImer1B
    TCCR1B |= B00000001;  // Prescaler = 1
    OCR1A = 64000; //Set output compare value = 16MHz*4ms
    TIMSK1 |= B00000010;  // Enable Timer Compare Interrupt
}

// Timer 1 compare interrupt service routine
ISR(TIMER1_COMPA_vect) {
    
  digitalWrite(LED_PIN, digitalRead(LED_PIN) ^ 1);
  interruptCounter++; // Increment the interrupt counter
  unsigned long currentMillis = millis();
  elapsedTime += currentMillis - previousMillis; // Calculate elapsed time
  previousMillis = currentMillis;
    
}

void setup() {
    pinMode(LED_PIN, OUTPUT);

    // First disable all interrupts
    noInterrupts();

    // Setup timer 1
    init_timer1();

    // Enable all interrupts
    interrupts();
    Serial.begin(9600);
}

void loop() {
    if (millis() - previousMillis >= 1000) { // Print every 1 second
      previousMillis = millis();
      Serial.print("Average time between interrupts: ");
      Serial.print(elapsedTime / interruptCounter);
      Serial.println(" milliseconds");
      interruptCounter = 0;
      elapsedTime = 0;
  }
}