/*
    Button
 
 Turns on and off a light emitting diode(LED) connected to digital  
 pin 13, when pressing a pushbutton attached to pin 2. 
 
 
 The circuit:
 * LED attached from pin 13 to ground 
 * pushbutton attached to pin 2 from +5V
 * 10K resistor attached to pin 2 from ground
 
 * Note: on most Arduinos there is already an LED on the board
 attached to pin 13.
 
 
 created 2005
 by DojoDave <http://www.0j0.org>
 modified 30 Aug 2011
 by Tom Igoe
 
 This example code is in the public domain.
 
 http://www.arduino.cc/en/Tutorial/Button
 */

// constants won't change. They're used here to 
// set pin numbers:
const int buttonPin0 = 1;     // the number of the pushbutton pin
const int buttonPin1 = 2;     // the number of the pushbutton pin
const int buttonPin2 = 3;     // the number of the pushbutton pin
const int buttonPin3 = 4;     // the number of the pushbutton pin
const int buttonPin4 = 5;     // the number of the pushbutton pin
const int sliderPin0 = A0;
const int sliderPin1 = A1;
const int sliderPin2 = A2;
const int ledPin =  13;      // the number of the LED pin

// variables for reading the pushbutton status
int buttonState0 = 0;
int buttonState1 = 0;
int buttonState2 = 0;
int buttonState3 = 0;
int buttonState4 = 0;
// variables for checking change in pushbutton status
int oldButtonState0 = 0;
int oldButtonState1 = 0;
int oldButtonState2 = 0;
int oldButtonState3 = 0;
int oldButtonState4 = 0;
// variables for reading slider values
float sliderState0 = 0;
float sliderState1 = 0; 
float sliderState2 = 0;

int mode = 0;
int oldButton = 0;

void setup() {
    // initialize the LED pin as an output:
    pinMode(ledPin, OUTPUT);      
    // initialize the pushbutton pin as an input:
    pinMode(buttonPin0, INPUT);
    pinMode(buttonPin1, INPUT);
    pinMode(buttonPin2, INPUT);
    pinMode(buttonPin3, INPUT);
    pinMode(buttonPin4, INPUT);
    digitalWrite(buttonPin0, HIGH);
    digitalWrite(buttonPin1, HIGH);
    digitalWrite(buttonPin2, HIGH);
    digitalWrite(buttonPin3, HIGH);
    digitalWrite(buttonPin4, HIGH);
    Serial.begin(115200);
}

void loop(){
    // read the state of the pushbutton value:
    buttonState0 = digitalRead(buttonPin0);
    buttonState1 = digitalRead(buttonPin1);
    buttonState2 = digitalRead(buttonPin2);
    buttonState3 = digitalRead(buttonPin3);
    buttonState4 = digitalRead(buttonPin4);

    // check if the pushbutton is pressed.
    // if it is, the buttonState is HIGH:
    if (buttonState0 == LOW && oldButtonState0 == HIGH) {
        mode = !mode;
        digitalWrite(ledPin, HIGH);
    } else if (buttonState1 == LOW && oldButtonState1 == HIGH) {
        Serial.println(1);
        digitalWrite(ledPin, HIGH);
    } else if (buttonState2 == LOW && oldButtonState2 == HIGH) {
        Serial.println(2);
        digitalWrite(ledPin, HIGH);
    } else if (buttonState3 == LOW && oldButtonState3 == HIGH) {
        Serial.println(3);
        digitalWrite(ledPin, HIGH);
    } else if (buttonState4 == LOW && oldButtonState4 == HIGH && buttonState1 == LOW) {
        // If button 1 is pressed first, then button 4, a 5 will be sent
        Serial.println(5);
        digitalWrite(ledPin, HIGH);
    } else if (buttonState4 == LOW && oldButtonState4 == HIGH) {
        Serial.println(4);
        digitalWrite(ledPin, HIGH);
    }

    if (buttonState0 == HIGH && buttonState1 == HIGH && buttonState2 == HIGH && buttonState3 == HIGH && buttonState4 == HIGH) {
        digitalWrite(ledPin, LOW);
    }

    if (mode) {
        sliderState0 = analogRead(sliderPin0);
        sliderState1 = analogRead(sliderPin1);
        sliderState2 = analogRead(sliderPin2);

        Serial.print("s:");
        Serial.print((700 - sliderState0)/700);
        Serial.print(":");
        Serial.print((700 - sliderState1)/700);
        Serial.print(":");
        Serial.println((700 - sliderState2)/700);
        delay(10);
    }

    oldButtonState0 = buttonState0;
    oldButtonState1 = buttonState1;
    oldButtonState2 = buttonState2;
    oldButtonState3 = buttonState3;
    oldButtonState4 = buttonState4;
    delay(1);
}
