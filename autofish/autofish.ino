#include <Keyboard.h>
#include <Mouse.h>

int buttonPin = 9;  // Set a button to any pin
int dumpPin = 8;    // Set a button to any pin


void setup() {
  pinMode(buttonPin, INPUT);      // Set the button as an input
  pinMode(dumpPin, INPUT);        // Set the button as an input
  digitalWrite(buttonPin, HIGH);  // Pull the button high
  digitalWrite(dumpPin, HIGH);    // Pull the button high
}


// circular buffer.
// we'll store as char.
struct Buffer {
  // assume even so only need start and end.
  // at least for now.

  static const int bufferSize = 1024;
  char deltas[bufferSize];
  int startIndex = 0;
  int length = 0;
  bool enabled = true;

  void enable(bool enable) {
    enabled = enable;
  }

  addSample(char value) {
    if (!enabled) {
      return;
    }

    deltas[(startIndex + length) & (bufferSize - 1)] = value;

    if (length < bufferSize) {
      ++length;
    } else {
      ++startIndex;                    // rolling buffer.
      startIndex &= (bufferSize - 1);  // wrap!
    }
  }
};

Buffer buffer;
int stopIndex = -1;
unsigned long stoppedMs = -1;  // means not set yet.
unsigned long downMs = 0;
unsigned long samples = 0;


void loop() {

  // note we can read more samples now.
  long val = analogRead(A0) - 511;

  if ((abs(val) > 50) && (stopIndex == -1) && (downMs == 0) && buffer.enabled) {
    // this is a self trigger.
    downMs = millis();
    samples = 0;
    // assume full, we will only read bufferSize - 200 samples.
    stopIndex = (buffer.startIndex + (buffer.bufferSize - 64)) & (buffer.bufferSize - 1);
    stoppedMs = 0;
    //    Serial.print(F("now"));
    //Serial.println(val);
  }

  if ((buffer.startIndex == stopIndex) && (stoppedMs == 0) && buffer.enabled) {
    stoppedMs = millis();
    Serial.print(F("stopped at index "));
    Serial.print(stopIndex);
    long stopped = (stoppedMs - downMs) * 2;
    Serial.print(F("Stopped Ms: "));
    Serial.println(stopped);

    buffer.enable(false);
  }

  buffer.addSample(val);

  //  Serial.println(abs(val));

  if (digitalRead(buttonPin) == 0)  // if the button goes low
  {
    stopIndex = -1;
    stoppedMs = -1;  // means not set yet.
    downMs = 0;
    samples = 0;
    buffer.enabled = true;
    Serial.println(F("Resetting!"));
  }

  // dump it and reenable.
  if (digitalRead(dumpPin) == 0 && buffer.length > 0 && !buffer.enabled) {
    Serial.println(F("Dumping"));

    //Keyboard.print("index, value\n");
    // dump them all.
    for (int element = 0; element < buffer.length; ++element) {
      int index = (buffer.startIndex + element) & (buffer.bufferSize - 1);
      //writeInt(element);
      //Keyboard.write(KEY_RIGHT_ARROW);
      //delay(10);
      //Keyboard.print(", ");
      writeInt(buffer.deltas[index]);
      delay(10);
      Keyboard.write(KEY_DOWN_ARROW);
      delay(10);
      //Keyboard.write(KEY_HOME);
      //delay(10);
    }
    delay(1000);
    buffer.enable(true);
  }
}

void writeInt(int val) {
  if (val < 0) {
    Keyboard.write('-');
    val = -val;
  }
  // easier to decompose backwards then write out the other way.
  char buffer[16];
  int index = 0;
  do {
    buffer[index++] = '0' + (val % 10);
    val /= 10;
  } while (val > 0);
  // and write it out backwards.
  while (--index >= 0) {
    Keyboard.write(buffer[index]);
  }
}
