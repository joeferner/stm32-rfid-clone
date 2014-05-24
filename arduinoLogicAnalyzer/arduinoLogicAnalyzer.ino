
#include <stdint.h>
#include <util/delay.h>

#define SAMPLE_RATE       180000

#define DEBUG_PIN         2  /* PD2 */

#define BUFFER_SIZE 1200
volatile char buffer[BUFFER_SIZE];
volatile char* bufferWrite;
volatile char* bufferEnd;

#define COMMAND_BUFFER_SIZE 20
char commandBuffer[COMMAND_BUFFER_SIZE];
int commandBufferOffset;

// Approximate number of CPU clock cycles for the timer interrupt.
//   any sample rate faster than this will not work using interrupts.
#define TICKS_PER_ISR  75

boolean triggered;
boolean useLoop;
uint32_t sampleRate;
uint16_t ticksPerSample;

void clearWorkbench();
void disableTimer1Interrupt();
void enableTimer1Interrupt();
boolean isTimer1Enabled();
boolean trigger();
void writeBufferData();
void sampleUsingLoop();
void sampleUsingInterrupts();

boolean trigger() {
  if(digitalRead(13)) {
    _delay_ms(4);
    return true;
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  commandBufferOffset = 0;
  commandBuffer[0] = '\0';

  pinMode(DEBUG_PIN, OUTPUT);
  pinMode(8, INPUT);  // PB0
  pinMode(9, INPUT);  // PB1
  pinMode(10, INPUT); // PB2
  pinMode(11, INPUT); // PB3
  pinMode(12, INPUT); // PB4
  pinMode(13, INPUT); // PB5

  setupTimer(SAMPLE_RATE);
}

void setupTimer(uint32_t sampleRateArg) {
  sampleRate = sampleRateArg;
  ticksPerSample = (F_CPU / sampleRate) - 1;

  cli();

  TIMSK0 = 0; // disable timer0 interrupt

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  TCCR1B |= (1 << WGM12); // CTC mode

  useLoop = false;
  if(ticksPerSample < TICKS_PER_ISR) {
    useLoop = true;
    
    // setup the timer to time how long it takes to read the samples using a loop
    OCR1A = 0;
    TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10); // 8 prescaler
  } else if(ticksPerSample < 0xffff) {
    OCR1A = ticksPerSample;
    TCCR1B |= (0 << CS12) | (0 << CS11) | (1 << CS10); // 1 prescaler
  } else if((ticksPerSample / 8) < 0xffff) {
    OCR1A = ticksPerSample / 8;
    TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10); // 8 prescaler
  } else if((ticksPerSample / 64) < 0xffff) {
    OCR1A = ticksPerSample / 64;
    TCCR1B |= (0 << CS12) | (1 << CS11) | (1 << CS10); // 64 prescaler
  } else if((ticksPerSample / 256) < 0xffff) {
    OCR1A = ticksPerSample / 256;
    TCCR1B |= (1 << CS12) | (0 << CS11) | (0 << CS10); // 256 prescaler
  } else {
    OCR1A = ticksPerSample / 1024;
    TCCR1B |= (1 << CS12) | (0 << CS11) | (1 << CS10); // 1024 prescaler
  }

  disableTimer1Interrupt();

  bufferWrite = bufferEnd = (buffer + BUFFER_SIZE);
  triggered = false;

  sei();
  
  Serial.print("!main.set sampleRate,");
  Serial.println(sampleRate);
  Serial.print("?ticksPerSample: ");
  Serial.println(ticksPerSample);
  Serial.print("?sampleRate: ");
  Serial.println(sampleRate);
}

void loop() {
  while(Serial.available()) {
    char ch = Serial.read();
    if(ch == '\r') {
    } else if(ch == '\n') {
      commandBuffer[commandBufferOffset] = '\0';
      if(commandBufferOffset == 0) {
        continue;
      } else if(!strcmp(commandBuffer, "!CONNECT")) {
        Serial.println("+OK");
        clearWorkbench();
      } else {
        Serial.println("-Invalid command");
      }
      commandBufferOffset = 0;
      commandBuffer[commandBufferOffset] = '\0';
    } else {
      commandBuffer[commandBufferOffset++] = ch;
      if(commandBufferOffset >= COMMAND_BUFFER_SIZE) {
        commandBufferOffset = 0;
      }
      commandBuffer[commandBufferOffset] = '\0';
    }
  }

  boolean t = trigger();
  if(triggered && !t) {
    triggered = false;
  }

  if(!isTimer1Enabled() && t && !triggered) {
    triggered = true;
    if(useLoop) {
      sampleUsingLoop();
    } else {
      sampleUsingInterrupts();
    }
    digitalWrite(DEBUG_PIN, 0);
    writeBufferData();
  }
}

void sampleUsingLoop() {
  uint16_t loopTime;
  int usDelayPerLoop = ((1.0 / ((float)SAMPLE_RATE)) * 1000000.0) - 2;
  Serial.print("?usDelayPerLoop: ");
  Serial.println(usDelayPerLoop);
  
  bufferWrite = buffer;
  TCNT1 = 0;
  while(1) {
    PORTD |= 0b00000100;
    *bufferWrite++ = PINB << 2;
    if(bufferWrite == bufferEnd) {
      break;
    }
    PORTD &= 0b11111011;
    _delay_us(usDelayPerLoop);
  }
  loopTime = TCNT1;
  Serial.print("?loopTime: ");
  Serial.println(loopTime);

  uint32_t loopTimePrescaler = 8;
  sampleRate = (BUFFER_SIZE / loopTimePrescaler) * F_CPU / loopTime;

  Serial.print("!main.set sampleRate,");
  Serial.println(sampleRate);
  Serial.print("?sampleRate: ");
  Serial.println(sampleRate);
}

void sampleUsingInterrupts() {
  bufferWrite = buffer;
  enableTimer1Interrupt();
  while(isTimer1Enabled()) {
  }
}

ISR(TIMER1_COMPA_vect) {
  PORTD |= 0b00000100;
  *bufferWrite++ = PINB << 2;
  if(bufferWrite == bufferEnd) {
    disableTimer1Interrupt();
  }
  PORTD &= 0b11111011;
}

boolean isTimer1Enabled() {
  return (TIMSK1 & (1 << OCIE1A));
}

void disableTimer1Interrupt() {
  TIMSK1 &= ~(1 << OCIE1A);
}

void enableTimer1Interrupt() {
  TCNT1 = 0;
  TIMSK1 |= (1 << OCIE1A);
}

void writeBufferData() {
  Serial.println("!main.clear");
  Serial.print("!main.beginData ");
  Serial.println(BUFFER_SIZE);
  for(int i = 0; i < BUFFER_SIZE; i++) {
    Serial.print(buffer[i]);
  }
  Serial.println();
}

void clearWorkbench() {
  Serial.println("!clear");
  Serial.println("!set name,'Arduino Logic Analyzer (RFID Clone)'");
  Serial.println("!set description,'6-bit Logic Analyzer'");

  Serial.println("?create plot");
  Serial.println("!add graph,main,0,0,1,1");

  Serial.println("?add signals");
  Serial.println("!main.addSignal writeBtn,1,0,5");
  Serial.println("!main.addSignal TX,1,0,5");
  Serial.println("!main.addSignal RX,1,0,5");
  Serial.println("!main.addSignal d2,1,0,5");
  Serial.println("!main.addSignal d1,1,0,5");
  Serial.println("!main.addSignal d0,1,0,5");
  Serial.println("!main.addAnalyzer EM4305,\"toDevice=1\"");

  setupTimer(SAMPLE_RATE);
}


