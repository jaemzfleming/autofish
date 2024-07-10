#include <Keyboard.h>
#include <Mouse.h>

//#define DEBUG
#define DEBUG2  // dumps up to 10 valid envelopes.

int pausePin = 9;     // set high to pause
int trainingPin = 8;  // set high to train
int outputPin = 7;    // set high to dump envelopes to spreadsheet

int audioPin = A0;
int opticalPin = A1;

void writeInt(int val, int decimal = 0);
void writeFloat(float f);

enum class State {
  LISTENING,
  TRACKING,
  PRE_LOOK,
  REELING_IN,
  POST_LOOK,
  DROP,
  CASTING,
};

State state = State::LISTENING;

struct SW {
  static const char* RED;
  static const char* GREEN;
  static const char* YELLOW;
  static const char* BLUE;
  static const char* DEF;
  static const char* CLS;
};
const char* SW::RED = "\033[31m";
const char* SW::GREEN = "\033[32m";
const char* SW::YELLOW = "\033[33m";
const char* SW::BLUE = "\033[34m";
const char* SW::DEF = "\033[0m";
const char* SW::CLS = "\033[2J\033[H";

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
  pinMode(pausePin, INPUT);     // Set the button as an input
  pinMode(trainingPin, INPUT);  // Set the button as an input
  pinMode(outputPin, INPUT);    // Set the button as an input

  /*
  // TEST
  Serial.print("\033[31mRed Text\033[0m\n");
  Serial.print("\033[32mGreen Text\033[0m\n");
  Serial.print("\033[33mYellow Text\033[0m\n");
  Serial.print("\033[34mBlue Text\033[0m\n");
  */
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
  // for matching, must be less to pass.
  float threshold = 0;
  bool training = false;
};

// Start doing kmeans.
struct KMeans {

  struct Sample {
    uint8_t samples[PatternMatcher::numElements];
    int8_t patternIndex = -1;    // in the patterns.
    float minSquaredError = -1;  // the minimum err squared for this pattern.
  };

  struct Pattern {
    float pattern[PatternMatcher::numElements];
    uint8_t count = 0;  // how many were averaged into here.

    // Average us into this.
    void integrateInto(const uint8_t samples[PatternMatcher::numElements]) {
      for (int i = 0; i < PatternMatcher::numElements; ++i) {
        pattern[i] = (pattern[i] * count + samples[i]) / (count + 1);
      }
      ++count;
    }
  };

  void reset() {
    index = 0;  // start over.
  }

  // Add a pattern if we can.
  void addPattern(int pattern[PatternMatcher::numElements]) {
    if (index < numSamples) {
      copyInto(pattern, samples[index].samples);
      ++index;
      sout << F("  KMeans copying ") << index << '/' << numSamples << F(" pattern\n");
      if (index == numSamples) {
        sout << "COMPUTING!!!\n";
        compute();
      }

    } else {
      sout << F("  KMeans ") << index << F(" ready to write\n");
    }
  }

  void copyInto(int src[PatternMatcher::numElements], uint8_t dst[PatternMatcher::numElements]) {
    for (int i = 0; i < PatternMatcher::numElements; ++i) {
      dst[i] = static_cast<uint8_t>(src[i]);
    }
  }

  // once we get going..
  float squaredError(const uint8_t src[PatternMatcher::numElements], const float pattern[PatternMatcher::numElements]) const {
    float err2 = 0;
    for (int i = 0; i < PatternMatcher::numElements; ++i) {
      float err = float(src[i]) - pattern[i];
      err2 += err * err;
    }
    sout << "   computed squared error is " << err2 << "\n";
    err2 /= PatternMatcher::numElements;
    sout << "   computed squared error per element is " << err2 << "\n";

    return err2;
  }

  // Computes the best fit for this sample, including setting min error and best pattern index
  void findBestFit(Sample& s) {
    s.minSquaredError = squaredError(s.samples, patterns[0].pattern);
    s.patternIndex = 0;
    for (int p = 1; p < K; ++p) {
      float err = squaredError(s.samples, patterns[p].pattern);
      if (err < s.minSquaredError) {
        s.minSquaredError = err;
        s.patternIndex = p;
      }
    }
  }

  // do the k means.
  void compute() {
    // Pick initial four seeds but don't reuse them.
    // Each is the greatest summed squared err from all the previous.
    for (auto& s : samples) {
      s.patternIndex = -1;
      s.minSquaredError = -2.0f;
    }
    // @TEST
    for (int i = 0; i < numSamples; ++i) {
      Sample& s = samples[i];
      sout << F(" sample ") << i << F(", index: ") << s.patternIndex << F(", err: ") << s.minSquaredError << '\n';
    }

    for (int destPattern = 0; destPattern < K; ++destPattern) {
      sout << "dest pattern =" << destPattern << '\n';
      float maxErr = -1;
      int worstSample = 0;
      for (int i = 0; i < numSamples; ++i) {
        Sample& s = samples[i];
        // skip used ones.
        if (s.patternIndex != -1) {
          sout << "skipping sample " << i << ", used\n";
          continue;
        }
        float err = 0;
        for (int p = 0; p < destPattern; ++p) {
          err += squaredError(s.samples, patterns[p].pattern);
        }
        sout << "err for sample " << i << " is " << err << '\n';
        if (err > maxErr) {
          maxErr = err;
          worstSample = i;
          sout << "is worst than max, setting worst\n";
        }
      }
      auto& s = samples[worstSample];
      sout << "worst was " << worstSample << "\n";
      patterns[destPattern].count = 0;
      patterns[destPattern].integrateInto(s.samples);
      s.patternIndex = destPattern;
      s.minSquaredError = maxErr;

      // @TEST quit after one, so we can see which one was used.
      if (destPattern == 1) {
        return;
      }
    }

    // while in a loop, maybe only do it while max error decreases?
    // or 16 steps, whichever comes first.
    do {

      // For each sample find the closest match
      // and mark it
      for (auto& s : samples) {
        findBestFit(s);
      }
    } while (false);
  }

  void write() const {

    // Row 0: The Legend row,
    Keyboard.print(F("index"));
    writeNextCol();

    // Normal samples.
    for (int i = 0; i < index; ++i) {
      Keyboard.print(F("env"));
      Keyboard.print(i);
      writeNextCol();
    }

    // patterns last if set.
    for (int i = 0; i < K; ++i) {
      if (patterns[i].count > 0) {
        Keyboard.print(F("pat"));
        Keyboard.print(i);
        writeNextCol();
      }
    }
    writeNextRow();

    // All the other rows.  Final two are fake row, shows which pattern.
    for (int element = 0; element <= PatternMatcher::numElements + 1; ++element) {

      bool patternRow = (element == PatternMatcher::numElements);
      bool errRow = (element == PatternMatcher::numElements + 1);

      if (errRow) {
        // skip a row, data, not part of graph.
        writeNextRow();
      }

      Keyboard.print(element);
      writeNextCol();

      for (auto& s : samples) {
        if (errRow) {
          writeFloat(s.minSquaredError);
        } else if (patternRow) {
          writeInt(s.patternIndex * 25);
        } else {
          writeInt(s.samples[element]);
        }
        writeNextCol();
      }

      // patterns last if set.
      for (int i = 0; i < K; ++i) {

        if (patterns[i].count > 0) {
          if (errRow) {
            writeFloat(0);  // no errror.
          } else if (patternRow) {
            writeInt(i * 25);
          } else {
            writeFloat(patterns[i].pattern[element]);
          }
          writeNextCol();
        }
      }
      // Next row.
      writeNextRow();
    }
  }

  void writeNextRow() {
    // Next row.
    Keyboard.write(KEY_DOWN_ARROW);
    delay(10);
    Keyboard.write(KEY_HOME);
    delay(10);
  }

  void writeNextCol() {
    delay(10);
    Keyboard.write(KEY_RIGHT_ARROW);
    delay(10);
  }


  static const uint8_t numSamples = 4;  // for kmeans. (32 not enough)
  static const uint8_t K = 4;
  uint8_t index = 0;  // for counting.

  Sample samples[numSamples];

  // and K patterns since it's k means and all.   Maybe we could/should keep these separate
  // from the samples.
  Pattern patterns[K];
};

KMeans kmeans;


PatternMatcher pattern;

// these should go into the class.
float successErr = 0;  // filtered.
float failureErr = 0;  // filtered.

// so should this.
long successes = 0;
long failures = 0;
long falsePositives = 0;

// during training, every 5 bumps up the initial threshold.
int falsePositiveStreak = 0;

long sampleTimeout = 10;
float preLookVal = 0;  // before we fished.
// we'll keep raising this until we get a true value.
float initialThreshold = 50;

// when we last cast, used for a timeout during listening
// The lure can get stuck on something in the water
// and just float around until we pull it out.
unsigned long lastCastMs = 0;

bool paused = false;

void loop() {

  // note we can read more samples now.
  long val = analogRead(audioPin) - 511;

  //  Serial.println(analogRead(opticalPin));
  if (digitalRead(pausePin)) {

    if (!paused) {
      sout << F("PAUSED\n");
      paused = true;
    }
    delay(100);
  } else if (paused) {
    sout << F("UNPAUSED\n");
    paused = false;
  }

  // see if training, still works if paused.
  if (digitalRead(trainingPin) != pattern.training) {
    pattern.training = !pattern.training;
    if (pattern.training) {
      sout << SW::CLS << F("START TRAINING\n");
      pattern.clear();
      initialThreshold = 50;
      successErr = 0;
      failureErr = 0;
      kmeans.reset();
    } else {
      // todo determine, maybe by keeping some filtered versions.
      pattern.threshold = (successErr + failureErr) / 2.0f;
      sout << F("END TRAINING, thresh: ") << pattern.threshold << '\n';
      successes = 0;
      failures = 0;
      falsePositives = 0;
    }
  }

  // only process if not paused.
  if (!paused) {
    envelope = max(envelope * .99, abs(val));

    if (sampleTimeout > 0) {
      --sampleTimeout;
      if (sampleTimeout == 0) {
        sout << SW::BLUE;
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
          case State::DROP:
            sout << F("DROP\n");
            break;
          case State::CASTING:
            sout << F("CASTING\n");
            break;
        }
        sout << SW::DEF;
      }
    } else {
      switch (state) {
        case State::LISTENING:
          {
            // timeout, we are stuck on something.
            if (millis() - lastCastMs > 60000) {
              sout << F(" LastCast Timeout exceeding, reeling in\n");
              state = State::REELING_IN;
              sampleTimeout = 1;
            }

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

            if (present) {
              sout << SW::GREEN << F(" Present! ");
            } else {
              sout << SW::RED << F(" Not Present! ");
            }

            sout << F("Pre: ") << preLookVal << F(", Post: ") << postLookVal << SW::DEF << '\n';

            // if 50% higher.8k9.  I know, it's rash but it gets bright.
            if (present) {
              state = State::DROP;
              if (pattern.training) {
                sout << SW::GREEN << F("  Integrating") << SW::DEF;
                pattern.integratePattern();
                sout << F("  lastErr: ") << pattern.lastErr << F(", old successErr: ") << successErr << '\n';
                // need at least one already.
                if (pattern.numPatterns > 1) {
                  successErr = (successErr == 0) ? pattern.lastErr : (successErr * .9f + pattern.lastErr * .1f);
                  printSuccessFailureErrs();
                }
                falsePositiveStreak = 0;

                if (pattern.training) {
                  // copy to tthe debug patterns
                  kmeans.addPattern(pattern.currentPattern);
                }
              }
            } else {
              state = State::DROP;
              if (pattern.training) {
                sout << SW::RED << F("  Discarding pattern\n") << SW::DEF;
                sout << F("  lastErr: ") << pattern.lastErr << F(", old failureErr: ") << failureErr << F(", FP Streak: ") << falsePositiveStreak << '\n';

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
        case State::DROP:
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
            lastCastMs = millis();
            break;
          }
      };
    }
  }


  if (pattern.training && digitalRead(outputPin)) {
    kmeans.write();
    delay(4000);
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
  sout << F("  new SuccessErr: ") << successErr << F(", new FailureErr: ") << failureErr << '\n';
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
#endif

#if defined(DEBUG) || defined(DEBUG2)

void writeInt(int val, int decimal) {
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
    if (index == decimal) {
      Keyboard.write('.');
    }
  }
}
void writeFloat(float f) {
  writeInt(static_cast<int>(f * 100.0f), 2);
}


#endif