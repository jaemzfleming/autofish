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

class SerialWrapper {
public:
  template<typename T>
  SerialWrapper& operator<<(const T& data) {
    Serial.print(data);
    return *this;
  }

  template<size_t N>
  SerialWrapper& operator<<(const int (&array)[N]) {
    *this << "(";
    for (size_t i = 0; i < N; i++) {
      *this << array[i];
      if (i < N - 1) {
        *this << ", ";
      }
    }
    *this << ")";
    return *this;
  }

  template<size_t N>
  SerialWrapper& operator<<(const float (&array)[N]) {
    *this << "(";
    for (size_t i = 0; i < N; i++) {
      *this << array[i];
      if (i < N - 1) {
        *this << ", ";
      }
    }
    *this << ")";
    return *this;
  }
};

SerialWrapper sout;

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
    threshold = 0;
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
        lastErr = 0;

        for (int i = 0; i < numElements; ++i) {
          currentPattern[i] /= samplesPerElement;
          float err = float(currentPattern[i]) - pattern[i];
          lastErr += err * err;
        }
        lastErr /= numElements;
        lastErr = sqrt(lastErr);

        success = training || (lastErr < threshold);
        if (training) {
          sout << F(" Done! err: ") << lastErr;
        } else {
          sout << (success ? F(" Match: ") : F(" No Match: ")) << lastErr << (success ? F(" < ") : F(" > ")) << threshold;
        }
        sout << ' ' << currentPattern << '\n';

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
    // we are done!
    for (int i = 0; i < numElements; ++i) {
      pattern[i] = (pattern[i] * numPatterns + currentPattern[i]) / (numPatterns + 1);
    }
    // print the new pattern.
    sout << F(" pattern ") << numPatterns << F(": ") << pattern << '\n';
    ++numPatterns;
  }

  bool success = false;

  float lastErr = 0;

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
  bool training = false;
};


PatternMatcher pattern;

float successErr = 0;  // filtered.
float failureErr = 0;  // filtered.

long successes = 0;
long failures = 0;
long falsePositives = 0;

// during training, every 5 bumps up the initial threshold.
int falsePositiveStreak = 0;

long sampleTimeout = 10;
float preLookVal = 0;  // before we fished.
// we'll keep raising this until we get a true value.
float initialThreshold = 50;

void loop() {

  // note we can read more samples now.
  long val = analogRead(audioPin) - 511;

  //  Serial.println(analogRead(opticalPin));
  if (digitalRead(pausePin) == 0) {
    //Serial.println(analogRead(opticalPin));
    delay(1000);
    return;
  }

  bool t = digitalRead(trainingPin) == 0;
  if (t != pattern.training) {
    if (t) {
      sout << F("Start Training\n");
      pattern.clear();
      initialThreshold = 50;
      successErr = 0;
      failureErr = 0;
    } else {
      // todo determine, maybe by keeping some filtered versions.
      pattern.threshold = (successErr + failureErr) / 2.0f;
      sout << F("End Training, thresh: ") << pattern.threshold << '\n';
      successes = 0;
      failures = 0;
      falsePositives = 0;
    }
    pattern.training = t;
  }

  envelope = max(envelope * .99, abs(val));

  if (sampleTimeout > 0) {
    --sampleTimeout;
    if (sampleTimeout == 0) {
      switch (state) {
        case State::LISTENING:
          sout << F("LISTENING...\n");
          break;
        case State::PRE_LOOK:
          sout << F("PRE LOOK\n");
          break;
        case State::REELING_IN:
          sout << F("REELING IN\n");
          break;
        case State::POST_LOOK:
          sout << F("POST LOOK\n");
          break;
        case State::DISCARD:
          sout << F("DISCARD\n");
          break;
        case State::CASTING:
          sout << F("CASTING\n");
          break;
      }
    }
  } else {
    switch (state) {
      case State::LISTENING:
        {
          if (abs(val) > initialThreshold) {
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
              ++successes;
              state = State::PRE_LOOK;
              sampleTimeout = 1;  // wait a quarter second before reeling in.
            } else {
              ++failures;
              state = State::LISTENING;
              sampleTimeout = 4000;  // wait a second.
            }
            if (!pattern.training) {
              printStats();
            }
          }
          break;
        }
      case State::PRE_LOOK:
        {
          preLookVal = analogRead(opticalPin);
          sampleTimeout = 500;
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
          sampleTimeout = 1;

          const bool present = (postLookVal > preLookVal * 1.3f && postLookVal > 20);

          if (!pattern.training && !present) {
            ++falsePositives;
          }

          sout << ' ' << (present ? F("Present! ") : F("Not Present! ")) << F("Pre: ") << preLookVal << F(", Post: ") << postLookVal << '\n';

          // if 50% higher.8k9.  I know, it's rash but it gets bright.
          if (present) {
            state = State::DISCARD;
            if (pattern.training) {
              sout << F(" Integrating");
              pattern.integratePattern();
              sout << F(" lastErr: ") << pattern.lastErr << F(", old successErr: ") << successErr << '\n';
              // need at least one already.
              if (pattern.numPatterns > 1) {
                successErr = (successErr == 0) ? pattern.lastErr : (successErr * .9f + pattern.lastErr * .1f);
                printSuccessFailureErrs();
              }
              falsePositiveStreak = 0;
            }
          } else {
            state = State::DISCARD;
            if (pattern.training) {
              sout << F(" Discarding pattern\n");
              sout << F(" lastErr: ") << pattern.lastErr << F(", old failureErr: ") << failureErr << F(", FP Streak: ") << falsePositiveStreak << '\n';

              if (pattern.numPatterns > 0) {
                failureErr = (failureErr == 0) ? pattern.lastErr : (failureErr * .9f + pattern.lastErr * .1f);
                printSuccessFailureErrs();

                if (++falsePositiveStreak >= 5) {
                  falsePositiveStreak = 0;
                  initialThreshold *= 1.1f;
                  sout << F(" Init threshold now ") << initialThreshold << '\n';
                }
              }

              if (pattern.numPatterns == 0) {
                initialThreshold *= 1.1f;
                sout << F(" Init threshold now ") << initialThreshold << '\n';
              }
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

// TODO add false positives, a match but no fish.
void printStats() {
  sout << F(" Matches: ") << successes << F(", Failures: ") << failures << F(", False Positives: ") << falsePositives << '\n';
}

void printSuccessFailureErrs() {
  sout << F(" new SuccessErr: ") << successErr << F(", new FailureErr: ") << failureErr << '\n';
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