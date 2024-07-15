const int pin = 4;

void setup() {
    // put your setup code here, to run once:
    pinMode(pin, OUTPUT);
}

void loop() {
    // put your main code here, to run repeatedly:
    digitalWrite(pin, HIGH);
    delay(3000);
    digitalWrite(pin, LOW);
    delay(3000);
}
