#include <Keyboard.h>
#include <Mouse.h>

//#define DEBUG

int pausePin = 9;     // Set a button to any pin
int trainingPin = 8;  // Set a button to any pin

int audioPin = A0;
int opticalPin = A1;

enum class State {
  LISTENING,
  TRACKING,
  PRE_LOOK,
  REELING_IN,
  POST_LOOK,
  DISCARD,
  CASTING,
};

State state = State::LISTENING;


void setup() {
  pinMode(pausePin, INPUT);         // Set the button as an input
  pinMode(trainingPin, INPUT);      // Set the button as an input
  digitalWrite(pausePin, HIGH);     // Pull the button high
  digitalWrite(trainingPin, HIGH);  // Pull the button high
}


#ifdef DEBUG
Buffer buffer;
int stopIndex = -1;
unsigned long stoppedMs = -1;  // means not set yet.
unsigned long downMs = 0;
unsigned long samples = 0;
#endif

// the current envelope value.
long envelope = 0;

struct PatternMatcher {

  static const int numElements = 16;
  static const int samplesPerElement = 64;  // per element.

  PatternMatcher() {
    for (int i = 0; i < numElements; ++i) {
      pattern[i] = 0;
    }
  }

  void clear() {
    numPatterns = 0;
  }

  void startSampling() {
    currentSample = 0;
    currentElement = 0;
    currentPattern[currentElement] = 0;
    success = false;
  }

  // Adds a sample, returns true when finished.
  bool addSample(int value) {
    currentPattern[currentElement] += abs(value);
    ++currentSample;
    if (currentSample == samplesPerElement) {
      currentSample = 0;
      ++currentElement;
      if (currentElement == numElements) {
        // we are done!  Find the max, normalize, etc.
        int maxVal = 0;
        for (int i = 0; i < numElements; ++i) {
          maxVal = max(maxVal, currentPattern[i]);
        }

        float squaredErr = 0;

        Serial.print(" current: ");
        for (int i = 0; i < numElements; ++i) {
          float tmp = float(currentPattern[i]) / float(maxVal);
          float err = tmp - pattern[i];
          squaredErr += err * err;
          Serial.print(tmp);
          Serial.print(F(", "));
        }
        squaredErr /= numElements;
        Serial.println("");
        Serial.print(" squared error: ");
        Serial.println(squaredErr);

        success = squaredErr < threshold;
        return true;

      } else {
        currentPattern[currentElement] = 0;
      }
    }

    // not done.
    return false;
  }

  // current pattern is good, integrate it.
  void integratePattern() {
    // we are done!  Find the max, normalize, etc.
    int maxVal = 0;
    for (int i = 0; i < numElements; ++i) {
      maxVal = max(maxVal, currentPattern[i]);
    }
    for (int i = 0; i < numElements; ++i) {
      float tmp = float(currentPattern[i]) / float(maxVal);
      pattern[i] = (pattern[i] * numPatterns + tmp) / (numPatterns + 1);
    }
    // print the new pattern.
    Serial.print(" pattern ");
    Serial.print(numPatterns);
    Serial.print(": ");
    for (int i = 0; i < numElements; ++i) {
      Serial.print(pattern[i]);
      Serial.print(F(", "));
    }
    Serial.println("");
    ++numPatterns;
  }

  bool success = false;

  // current pattern sample.
  int currentSample = 0;

  // the current pattern we are gathering.  We match them all.
  // before deciding.
  int currentPattern[numElements];
  // the averaged pattern.
  float pattern[numElements];
  // how many so far, need for adding into.
  int numPatterns = 0;
  // if squared error beats this, then we do it.
  int currentElement = 0;  // which element we are adding to currently.
  // for matching, during training, starts high.
  float threshold = 1;
};


PatternMatcher pattern;

long sampleTimeout = 10;
long failures = 0;
long successes = 0;
float preLookVal = 0;  // before we fished.
bool paused = false;
bool training = false;

void loop() {

  // note we can read more samples now.
  long val = analogRead(audioPin) - 511;

  //  Serial.println(analogRead(opticalPin));
  if (digitalRead(pausePin) == 0) {
    delay(1000);
    return;
  }

  bool t = digitalRead(trainingPin) == 0;
  if (t != training) {
    if (t) {
      Serial.println(F("Start Training"));
      pattern.clear();
      pattern.threshold = 1.0f;
    } else {
      Serial.println(F("End Training"));
      // todo determine, maybe by keeping some filtered versions.
      pattern.threshold = .03f;
    }
    training = t;
  }

  envelope = max(envelope * .99, abs(val));

  if (sampleTimeout > 0) {
    --sampleTimeout;
    if (sampleTimeout == 0) {
      switch (state) {
        case State::LISTENING:
          Serial.println(F("Listening..."));
          break;
        case State::PRE_LOOK:
          Serial.println(F("Pre look"));
          break;
        case State::REELING_IN:
          Serial.println(F("Reeling In"));
          break;
        case State::POST_LOOK:
          Serial.println(F("Post look"));
          break;
        case State::DISCARD:
          Serial.println(F("Discard"));
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
            pattern.startSampling();
          }
          break;
        }
      case State::TRACKING:
        {
          // returns true when finished.
          if (pattern.addSample(envelope)) {
            if (pattern.success) {
              Serial.println(" Success!");
              ++successes;
              state = State::PRE_LOOK;
              sampleTimeout = 2000;  // wait a quarter second before reeling in.
            } else {
              ++failures;
              Serial.println(F(" Failed!!!"));
              state = State::LISTENING;
              sampleTimeout = 4000;  // wait a second.
            }
            printSuccesses();
          }
          break;
        }
      case State::PRE_LOOK:
        {
          preLookVal = analogRead(opticalPin);
          Serial.print(F(" preLookVal: "));
          Serial.println(preLookVal);
          sampleTimeout = 1;
          state = State::REELING_IN;
          break;
        }
      case State::REELING_IN:
        {
          Mouse.click(MOUSE_RIGHT);
          sampleTimeout = 8000;  // wait a second before looking, so it lands.
          state = State::POST_LOOK;
          break;
        }
      case State::POST_LOOK:
        {
          float postLookVal = analogRead(opticalPin);
          Serial.print(F("postLookVal: "));
          Serial.println(postLookVal);
          sampleTimeout = 1;  // wait a second before casting.

          // if 10% higher.8k9
          if (postLookVal > preLookVal * 1.15f) {
            state = State::DISCARD;
            if (training) {
              Serial.println(" Integrating pattern");
              pattern.integratePattern();
            }
          } else {
            state = State::CASTING;
            if (training) {
              Serial.println(" Discarding pattern");
            }
            // TODO note that it was a failure, and we don't have to discard anything.
          }

          break;
        }
      case State::DISCARD:
        {
          // We'll roll discard in for now.
          Keyboard.write('8');
          delay(200);
          Keyboard.write('k');
          delay(200);
          Keyboard.write('9');
          delay(200);
          state = State::CASTING;
          sampleTimeout = 4000;
          break;
        }
      case State::CASTING:
        {
          // We'll roll discard in for now.
          Keyboard.write('8');
          delay(200);
          Keyboard.write('k');
          delay(200);
          Keyboard.write('9');
          delay(200);

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