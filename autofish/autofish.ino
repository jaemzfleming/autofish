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
  CHECK_FALSE_NEGATIVE,
  DROP,
  CASTING,
};

State state = State::LISTENING;

struct SW {
  static const char* BLACK;
  static const char* RED;
  static const char* GREEN;
  static const char* YELLOW;
  static const char* BLUE;
  static const char* MAGENTA;
  static const char* CYAN;
  static const char* WHITE;

  static const char* BRIGHT_BLACK;
  static const char* BRIGHT_RED;
  static const char* BRIGHT_GREEN;
  static const char* BRIGHT_YELLOW;
  static const char* BRIGHT_BLUE;
  static const char* BRIGHT_MAGENTA;
  static const char* BRIGHT_CYAN;
  static const char* BRIGHT_WHITE;

  static const char* DARK_GREEN;

  static const char* DEF;
  static const char* CLS;
};

const char* SW::BLACK = "\033[30m";
const char* SW::RED = "\033[31m";
const char* SW::GREEN = "\033[32m";
const char* SW::YELLOW = "\033[33m";
const char* SW::BLUE = "\033[34m";
const char* SW::MAGENTA = "\033[35m";
const char* SW::CYAN = "\033[36m";
const char* SW::WHITE = "\033[37m";

const char* SW::BRIGHT_BLACK = "\033[90m";
const char* SW::BRIGHT_RED = "\033[91m";
const char* SW::BRIGHT_GREEN = "\033[92m";
const char* SW::BRIGHT_YELLOW = "\033[93m";
const char* SW::BRIGHT_BLUE = "\033[94m";
const char* SW::BRIGHT_MAGENTA = "\033[95m";
const char* SW::BRIGHT_CYAN = "\033[96m";
const char* SW::BRIGHT_WHITE = "\033[97m";

const char* SW::DARK_GREEN = "\033[38;5;22m";  // A darker green from the 256 color palette

const char* SW::DEF = "\033[0m";
const char* SW::CLS = "\033[2J\033[H";

class SerialWrapper {
public:

  template<size_t N>
  SerialWrapper& operator<<(const uint8_t (&array)[N]) {
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

  template<typename T>
  SerialWrapper& operator<<(const T& data) {
    Serial.print(data);
    return *this;
  }
};

SerialWrapper sout;

void setup() {
  pinMode(pausePin, INPUT);     // Set the button as an input
  pinMode(trainingPin, INPUT);  // Set the button as an input
  pinMode(outputPin, INPUT);    // Set the button as an input
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

const int numElements = 16;

struct AccumulatingBuffer {

  static const int samplesPerElement = 64;  // per element.

  void startAccumulating() {
    currentElement = 0;
    currentSample = 0;
    internalData[currentElement] = 0;
  }
  // Adds a sample, returns true when finished.
  bool addSubElement(int value) {
    internalData[currentElement] += abs(value);
    ++currentSample;
    if (currentSample == samplesPerElement) {
      currentSample = 0;
      ++currentElement;
      if (currentElement == numElements) {
        for (int i = 0; i < numElements; ++i) {
          data[i] = static_cast<uint8_t>(internalData[i] / samplesPerElement);
        }
        return true;
      } else {
        internalData[currentElement] = 0;
        return false;
      }
    }
    return false;
  }

  uint8_t data[numElements];

protected:
  int internalData[numElements];
  int currentElement = 0;
  int currentSample = 0;
};

AccumulatingBuffer accBuffer;

// Patterns you can match against.
struct Patterns {

  struct BestFitVal {
    int index;
    float errorSquared;
  };

  static const uint8_t K = 4;  // should be four, but 2 for now.

  struct Pattern {
    int count = 0;         // how many were averaged into the data.
    float averageErr = 0;  // non squared, the average err from this pattern once matched.
    float errStdDev = 0;   // what the variance was.
    float data[numElements];

    // Average us into this.
    void integrateInto(const uint8_t samples[numElements]) {
      for (int i = 0; i < numElements; ++i) {
        data[i] = (data[i] * count + samples[i]) / (count + 1);
      }
      ++count;
    }

    // The squared error against us.
    float squaredError(const uint8_t src[numElements]) const {
      float err2 = 0;
      for (int i = 0; i < numElements; ++i) {
        float err = float(src[i]) - data[i];
        err2 += err * err;
      }
      err2 /= numElements;

      return err2;
    }
  };

  void resetCounts() {
    for (auto& p : patterns) {
      p.count = 0;
    }
  };

  // Finds the least squared error.  Returning index and error.
  BestFitVal findClosest(const uint8_t src[numElements]) const {
    float minErr = patterns[0].squaredError(src);
    int bestIndex = 0;
    for (int p = 1; p < K; ++p) {
      float err = patterns[p].squaredError(src);
      if (err < minErr) {
        minErr = err;
        bestIndex = p;
      }
    }
    return { bestIndex, minErr };
  }

  // Does this count as a match (currently 2 stdevs since it shouldn't be gaussian, should be better, maybe 1 would be best?)
  bool isMatch(const uint8_t src[numElements]) {
    float minStdDevs = 10e3;
    float bestErr = 10e3;
    int bestPattern = 0;

    for (int i = 0; i < K; ++i) {
      auto& p = patterns[i];
      float err = sqrtf(p.squaredError(src));
      float stddevs = (err - p.averageErr) / p.errStdDev;
      if (stddevs < minStdDevs) {
        minStdDevs = stddevs;
        bestErr = err;
        bestPattern = i;
      }
    }
    bool match = minStdDevs <= 2;

    if (match) {
      sout << " " << SW::DARK_GREEN << F("Match!");
    } else {
      sout << " " << SW::RED << F("Not Match!");
    }

    sout << SW::DEF << " " << F(" stddevs: ") << minStdDevs << F(", pattern: ") << bestPattern << F(", err: ") << bestErr << '\n';

    return match;
  }

  // and K patterns since it's k means and all.   Maybe we could/should keep these separate
  // from the samples.
  Pattern patterns[K];
};

Patterns patterns;

// Start doing kmeans.
struct KMeans {

  static const uint8_t numSamples = 64;  // for kmeans. (32 not enough)

  struct Sample {
    uint8_t samples[numElements];
    int8_t patternIndex = -1;    // in the patterns.
    float minSquaredError = -1;  // the minimum err squared for this pattern.
  };

  void startTraining() {
    sampleIndex = 0;  // start over.
  }

  bool isTraining() const {
    return sampleIndex < numSamples;
  }

  bool hasValidSamples() const {
    return sampleIndex > 0;
  }

  // Add a pattern if we can, returns true when finished.
  void addSample(const uint8_t sample[numElements]) {
    if (sampleIndex < numSamples) {
      auto& s = samples[sampleIndex];
      for (size_t i = 0; i < numElements; ++i) {
        s.samples[i] = sample[i];
      }
      ++sampleIndex;
      sout << F("  ") << sampleIndex << '/' << numSamples << F(" samples acquired\n");
      if (sampleIndex == numSamples) {
        sout << "COMPUTING!!!\n";
        compute();
      }
    } else {
      sout << F("  KMeans ") << sampleIndex << F(" ready to write\n");
    }
  }

  // Computes the best fit for this sample, including setting min error and best pattern index
  // returns true if the best fit changed.
  bool findBestFit(Sample& s) const {
    int8_t oldPatternIndex = s.patternIndex;
    auto val = patterns.findClosest(s.samples);
    s.patternIndex = val.index;
    s.minSquaredError = val.errorSquared;
    return s.patternIndex != oldPatternIndex;
  }

  // do the k means.
  void compute() {
    // Pick initial four seeds but don't reuse them.
    // Each is the greatest summed squared err from all the previous.
    for (auto& s : samples) {
      s.patternIndex = -1;
      s.minSquaredError = -2.0f;
    }

    for (int destPattern = 0; destPattern < Patterns::K; ++destPattern) {
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
          err += patterns.patterns[p].squaredError(s.samples);
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
      patterns.patterns[destPattern].count = 0;
      patterns.patterns[destPattern].integrateInto(s.samples);
      s.patternIndex = destPattern;
      s.minSquaredError = maxErr;
    }

    // force them to mark changed for the first time
    for (auto& s : samples) {
      s.patternIndex = -1;
    }

    int steps = 0;
    bool changed = false;
    do {

      sout << F("Doing an average\n");
      changed = false;
      // For each sample find the closest match
      // and mark it, noting if any changed.
      for (auto& s : samples) {
        changed |= findBestFit(s);
      }

      patterns.resetCounts();

      // Average the samples into the patterns
      for (auto& s : samples) {
        patterns.patterns[s.patternIndex].integrateInto(s.samples);
      }

    } while (changed && ++steps < 16);

    // Per pattern, compute the average err (not squared) and the variance so that we can use that as criteria.
    for (int i = 0; i < Patterns::K; ++i) {
      auto& p = patterns.patterns[i];
      float err = 0;
      for (auto& s : samples) {
        if (s.patternIndex == i) {
          err += sqrtf(s.minSquaredError);
        }
      }
      p.averageErr = err / p.count;
      float variance = 0;
      // variance.
      for (auto& s : samples) {
        if (s.patternIndex == i) {
          float delta = sqrtf(s.minSquaredError) - p.averageErr;
          variance += delta * delta;
        }
      }
      p.errStdDev = sqrtf(variance / (p.count - 1));
    }
  }

  void write() const {

    // Row 0: The Legend row,
    Keyboard.print(F("index"));
    writeNextCol();

    // Normal samples.
    for (int i = 0; i < sampleIndex; ++i) {
      Keyboard.print(F("env"));
      Keyboard.print(i);
      writeNextCol();
    }

    // patterns last if set.
    for (int i = 0; i < Patterns::K; ++i) {
      if (patterns.patterns[i].count > 0) {
        Keyboard.print(F("pat"));
        Keyboard.print(i);
        writeNextCol();
      }
    }
    writeNextRow();

    enum class RowType {
      NORMAL = 0,
      PATTERN,
      ERR,
      STDDEV
    };

    // All the other rows.  Final two are fake row, shows which pattern.
    for (int element = 0; element <= numElements + 2; ++element) {

      RowType rowType = (element < numElements) ? RowType::NORMAL : RowType(1 + element - numElements);

      bool patternRow = (element == numElements);
      bool errRow = (element == numElements + 1);
      bool varianceRow = (element == numElements + 2);

      if (errRow) {
        // skip a row, data, not part of graph.
        writeNextRow();
      }

      Keyboard.print(element);
      writeNextCol();

      for (auto& s : samples) {
        switch (rowType) {
          case RowType::NORMAL:
            writeInt(s.samples[element]);
            break;
          case RowType::PATTERN:
            writeInt(s.patternIndex * 25);
            break;
          case RowType::ERR:
            writeFloat(sqrtf(s.minSquaredError));
            break;
          case RowType::STDDEV:
            Keyboard.print(" ");
            break;
        };
        writeNextCol();
      }

      // patterns last if set.
      for (int i = 0; i < Patterns::K; ++i) {
        const auto& p = patterns.patterns[i];
        if (p.count > 0) {

          switch (rowType) {
            case RowType::NORMAL:
              writeFloat(p.data[element]);
              break;
            case RowType::PATTERN:
              writeInt(i * 25);
              break;
            case RowType::ERR:
              writeFloat(p.averageErr);  // show the error.
              break;
            case RowType::STDDEV:
              writeFloat(p.errStdDev);
              break;
          };
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

  // the samples.
  Sample samples[numSamples];

protected:
  uint8_t sampleIndex = 0;  // for counting.
};

KMeans kmeans;

// When evaluating matches, what we get.
struct Stats {
  int truePositives = 0;
  int trueNegatives = 0;
  int falsePositives = 0;
  int falseNegatives = 0;

  reset() {
    truePositives = 0;
    trueNegatives = 0;
    falsePositives = 0;
    falseNegatives = 0;
  }

  void print() {
    sout << F(" True Positives: ") << truePositives << F(", True Negatives: ") << trueNegatives << F(", False Positives: ") << falsePositives << F(", False Negatives: ") << falseNegatives << '\n';
    sout << F(" FPR: ") << 100.0f * falsePositives / (trueNegatives + falsePositives) << " FNR: " << 100.0f * falseNegatives / (truePositives + falseNegatives) << '\n';
  }
};

Stats stats;

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

  // Reset training.
  if (digitalRead(trainingPin)) {
    kmeans.startTraining();
    sout << SW::CLS << F("START TRAINING\n");
    initialThreshold = 50;
    delay(1000);  // debounce.
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
          case State::CHECK_FALSE_NEGATIVE:
            sout << F("CHECK FALSE NEGATIVE\n");
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
              break;
            }

            if (abs(val) > initialThreshold) {
              state = State::TRACKING;
              accBuffer.startAccumulating();
              // explicitly fall through so we get that first one.
            } else {
              break;
            }
          }
        case State::TRACKING:
          {
            // returns true when finished.
            if (accBuffer.addSubElement(envelope)) {
              if (kmeans.isTraining()) {
                state = State::PRE_LOOK;
                sampleTimeout = 1;
                sout << F("  ") << accBuffer.data << '\n';
              } else {
                //are we a match?
                bool success = patterns.isMatch(accBuffer.data);
                if (success) {
                  ++stats.truePositives;  // assume true until shown otherwise.
                  state = State::PRE_LOOK;
                  sampleTimeout = 1;  // wait a quarter second before reeling in.
                } else {
                  ++stats.trueNegatives;
                  // check for false negitaves (todo make a switch.)
                  bool checkForFalseNegative = true;
                  if (checkForFalseNegative) {
                    state = State::CHECK_FALSE_NEGATIVE;
                    sampleTimeout = 1;
                  } else {
                    state = State::LISTENING;
                    sampleTimeout = 4000;  // wait a second.
                  }
                }
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
        case State::CHECK_FALSE_NEGATIVE:
        case State::POST_LOOK:
          {
            float postLookVal = analogRead(opticalPin);

            const bool present = (postLookVal > preLookVal * 1.3f && postLookVal > 20);

            if (present) {
              sout << SW::DARK_GREEN << F(" Present! ");
            } else {
              sout << SW::RED << F(" Not Present! ");
            }
            sout << F("Pre: ") << preLookVal << F(", Post: ") << postLookVal << SW::DEF << '\n';

            if (!kmeans.isTraining()) {
              // this was false, not true.
              if (!present) {
                --stats.truePositives;
                ++stats.falsePositives;
              } else if (state == State::CHECK_FALSE_NEGATIVE) {
                --stats.trueNegatives;
                ++stats.falseNegatives;
              }
              stats.print();

            } else {
              if (present) {
                // reset the false positive stream.
                falsePositiveStreak = 0;
                // add a new sample for kmeans.
                kmeans.addSample(accBuffer.data);

                // if done, we reset the stats.
                if (!kmeans.isTraining()) {
                  stats.reset();
                }
                // not present.
              } else {
                sout << SW::RED << F("  Discarding pattern\n") << SW::DEF;
                sout << F("  False positive streak: ") << falsePositiveStreak << '\n';

                // if no valid samples yet.
                if (!kmeans.hasValidSamples() || ++falsePositiveStreak >= 5) {
                  falsePositiveStreak = 0;
                  initialThreshold *= 1.1f;
                  sout << F("  Init threshold now ") << initialThreshold << '\n';
                }
              }
            }

            state = State::DROP;
            sampleTimeout = 1;

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

  if (digitalRead(outputPin)) {
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