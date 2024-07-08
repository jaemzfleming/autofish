#include <Keyboard.h>
#include <Mouse.h>

int buttonPin = 9;  // Set a button to any pin
int dumpPin = 8;    // Set a button to any pin

enum class State {
  LISTENING,
  TRACKING,
  REELING_IN,
  CASTING,
};

State state = State::LISTENING;

//#define DEBUG

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


#ifdef DEBUG
Buffer buffer;
int stopIndex = -1;
unsigned long stoppedMs = -1;  // means not set yet.
unsigned long downMs = 0;
unsigned long samples = 0;
#endif

// the current envelope value.
long envelope = 0;
// current chunk.
long chunk = 0;
long chunkIndex = 0;
long chunkVal = 0;

long lastChunkVal = 0;
bool tracking = false;
long sampleTimeout = 1;
long failures = 0;
long successes = 0;
bool highers[5] = { true, true, false, false, false };

void loop() {

  // note we can read more samples now.
  long val = analogRead(A0) - 511;

#ifdef DEBUG
  if ((abs(val) > 50) && (stopIndex == -1) && (downMs == 0) && buffer.enabled) {
    // this is a self trigger.
    downMs = millis();
    samples = 0;
    // assume full, we will only read bufferSize - 200 samples.
    stopIndex = (buffer.startIndex + (buffer.bufferSize - 64)) & (buffer.bufferSize - 1);
    stoppedMs = 0;
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

#endif

  envelope = max(envelope * .99, abs(val));

  if (sampleTimeout > 0) {
    --sampleTimeout;
    if (sampleTimeout == 0) {
      switch (state) {
        case State::LISTENING:
          Serial.println(F("Listening..."));
          break;
        case State::REELING_IN:
          Serial.println(F("Reeling In"));
          break;
        case State::CASTING:
          Serial.println(F("Casting"));
          break;
      }
    }
  } else {
    switch (state) {
      case State::LISTENING:
        {
          if (abs(val) > 50) {
            //    Serial.print(F("now"));
            //Serial.println(val);
            state = State::TRACKING;
            chunkIndex = 0;
            chunk = 0;
            chunkVal = 0;
            lastChunkVal = 0;
          }
          break;
        }
      case State::TRACKING:
        {
          chunkVal += envelope;
          ++chunk;
          if (chunk == 64) {
            // keep going or fail.
            bool success = (chunkVal >= lastChunkVal) == highers[chunkIndex];
            if (!success) {
              ++failures;
              Serial.print(F("Failed!!! at "));
              Serial.println(chunkIndex);
              printSuccesses();
              state = State::LISTENING;
              sampleTimeout = 4000;  // wait a second.
            } else {
              lastChunkVal = chunkVal;
              chunk = 0;
              chunkVal = 0;
              ++chunkIndex;
              if (chunkIndex == 5) {
                Serial.println("Success!");
                ++successes;
                printSuccesses();
                state = State::REELING_IN;
                sampleTimeout = 2000;  // wait a quarter second before reeling in.
                break;
              }
            }
          }
          break;
        }
      case State::REELING_IN:
        {
          Mouse.click(MOUSE_RIGHT);
          sampleTimeout = 8000;  // wait a second before casting.
          state = State::CASTING;
          break;
        }
      case State::CASTING:
        {
          Mouse.click(MOUSE_RIGHT);
          sampleTimeout = 16000;  // wait a second before listening.
          state = State::LISTENING;
          break;
        }
    };
  }

#ifdef DEBUG
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

    //writeChunksOf64();
    //writeEnvelope();
    writeEnvelopeChunksOf64();

    delay(1000);
    buffer.enable(true);
  }
#endif
}

void printSuccesses() {
  Serial.print(F("Successes/Failures: "));
  Serial.print(successes);
  Serial.print(F("/"));
  Serial.println(failures);
}



#ifdef DEBUG
void writeBuffer() {
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
}

void writeEnvelope() {
  int envelope = 0;

  for (int element = 0; element < buffer.length; ++element) {
    int index = (buffer.startIndex + element) & (buffer.bufferSize - 1);

    envelope = max(envelope * .99, abs(buffer.deltas[index]));
    writeInt(envelope);
    delay(10);
    Keyboard.write(KEY_DOWN_ARROW);
    delay(10);
  }
}

void writeEnvelopeChunksOf64() {
  int envelope = 0;

  int chunk = 0;
  int chunkVal = 0;

  for (int element = 0; element < buffer.length; ++element) {
    int index = (buffer.startIndex + element) & (buffer.bufferSize - 1);

    envelope = max(envelope * .99, abs(buffer.deltas[index]));

    chunkVal += envelope;

    ++chunk;
    if (chunk == 64) {
      chunkVal /= chunk;
      while (chunk-- > 0) {
        writeInt(chunkVal);
        delay(10);
        Keyboard.write(KEY_DOWN_ARROW);
        delay(10);
      }
      // start over.
      chunk = 0;
      chunkVal = 0;
    }
  }

  chunkVal /= chunk;
  while (chunk-- > 0) {
    writeInt(envelope);
    delay(10);
    Keyboard.write(KEY_DOWN_ARROW);
    delay(10);
  }
}



void writeChunksOf64() {

  int chunk = 0;
  int maxVal = 0;

  for (int element = 0; element < buffer.length; ++element) {
    int index = (buffer.startIndex + element) & (buffer.bufferSize - 1);

    maxVal = max(maxVal, abs(buffer.deltas[index]));
    ++chunk;
    if (chunk == 64) {
      while (chunk-- > 0) {
        writeInt(maxVal);
        delay(10);
        Keyboard.write(KEY_DOWN_ARROW);
        delay(10);
      }

      // start over.
      chunk = 0;
      maxVal = 0;
    }
    //Keyboard.write(KEY_HOME);
    //delay(10);
  }

  while (chunk-- > 0) {
    writeInt(maxVal);
    delay(10);
    Keyboard.write(KEY_DOWN_ARROW);
    delay(10);
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
#endif