#include <FastLED.h>
#include <vector>

#define NUM_LEDS_HEAD 29 // Number of LEDs on head
#define NUM_LEDS_BODY 71 // Number of LEDs on body
#define DATA_PIN_HEAD 4  // Teensy connection for head
#define DATA_PIN_BODY 3  // Teensy connection for body
#define BRIGHTNESS 255   // Brightness of LEDs (0-255)
#define LED_TYPE WS2811  // LED type
#define COLOR_ORDER RGB  // Color order for WS2811 LEDs

typedef enum
{
  HEAD,
  BODY
} LED_ARRAY;

typedef struct
{
  LED_ARRAY arrayIndex;
  int min_index;
  int max_index;
  CRGB defaultColor;
  CRGB *led_array;
} LED_SEGMENT_RANGE;

typedef enum
{
  HEART_0,
  HEART_1,
  HEART_2,
  HEART_3,
  RIGHT_ARM,
  RIGHT_CUFF,
  RIGHT_FINGER,
  BELT,
  LEFT_FINGER,
  LEFT_CUFF,
  LEFT_ARM,
  COLLAR,
  TASSEL,
  HAT,
  TRIM,
  LEFT_EYE,
  NOSE,
  RIGHT_EYE,
  MOUTH
} grinchSegmentIndex;

LED_SEGMENT_RANGE grinchSegments[] = {
    {BODY, 0, 0, CRGB::Red},     // heart 0
    {BODY, 1, 10, CRGB::Red},    // heart 1
    {BODY, 11, 26, CRGB::Red},   // heart 2
    {BODY, 27, 44, CRGB::Red},   // heart 3
    {BODY, 45, 49, CRGB::Red},   // right arm
    {BODY, 50, 50, CRGB::White}, // right cuff
    {BODY, 51, 51, CRGB::Green}, // right finger
    {BODY, 52, 57, CRGB::White}, // belt
    {BODY, 58, 58, CRGB::Green}, // left finger
    {BODY, 59, 59, CRGB::White}, // left cuff
    {BODY, 60, 63, CRGB::Red},   // left arm
    {BODY, 64, 70, CRGB::White}, // collar
    {HEAD, 0, 1, CRGB::White},   // tassel
    {HEAD, 2, 11, CRGB::Red},    // hat
    {HEAD, 12, 18, CRGB::White}, // trim
    {HEAD, 19, 19, CRGB::Red},   // left eye
    {HEAD, 20, 20, CRGB::Green}, // nose
    {HEAD, 21, 21, CRGB::Red},   // right eye
    {HEAD, 22, 28, CRGB::Green}, // mouth
};

CRGB leds_head[NUM_LEDS_HEAD];
CRGB leds_body[NUM_LEDS_BODY];

void selfTest()
{
  fill_solid(leds_head, NUM_LEDS_HEAD, CRGB::Red);
  fill_solid(leds_body, NUM_LEDS_BODY, CRGB::Red);
  FastLED.show();
  delay(500);
  FastLED.clear(true); // Ensure all LEDs start off
  fill_solid(leds_head, NUM_LEDS_HEAD, CRGB::Green);
  fill_solid(leds_body, NUM_LEDS_BODY, CRGB::Green);
  FastLED.show();
  delay(500);
  FastLED.clear(true); // Ensure all LEDs start off
  fill_solid(leds_head, NUM_LEDS_HEAD, CRGB::Blue);
  fill_solid(leds_body, NUM_LEDS_BODY, CRGB::Blue);
  FastLED.show();
  delay(500);
  FastLED.clear(true); // Ensure all LEDs start off
  fill_solid(leds_head, NUM_LEDS_HEAD, CRGB::White);
  fill_solid(leds_body, NUM_LEDS_BODY, CRGB::White);
  FastLED.show();
  delay(500);
  FastLED.clear(true); // Ensure all LEDs start off
  delay(1000);
}

void setup()
{
  FastLED.addLeds<LED_TYPE, DATA_PIN_HEAD, COLOR_ORDER>(leds_head, NUM_LEDS_HEAD).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<LED_TYPE, DATA_PIN_BODY, COLOR_ORDER>(leds_body, NUM_LEDS_BODY).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.clear(true); // Ensure all LEDs start off
  FastLED.show();
  delay(100);
  selfTest();

  // Serial.begin(9600); // Baud rate must match monitor_speed in platformio.ini

  // Loop over all grinchSegments
  int segmentCount = sizeof(grinchSegments) / sizeof(grinchSegments[0]); // Calculate the number of segments

  for (int i = 0; i < segmentCount; ++i)
  {
    LED_SEGMENT_RANGE &segment = grinchSegments[i];

    // Initialize LED array pointer based on the array index
    if (segment.arrayIndex == HEAD)
    {
      segment.led_array = leds_head;
    }
    else if (segment.arrayIndex == BODY)
    {
      segment.led_array = leds_body;
    }
  }
}

void flashSegment(grinchSegmentIndex bodyPart, int time = 100, bool clear = false)
{
  LED_SEGMENT_RANGE &segment = grinchSegments[bodyPart];
  int i;
  for (i = segment.min_index; i <= segment.max_index; i++)
  {
    segment.led_array[i] = segment.defaultColor;
  }
  FastLED.show();
  delay(time);

  if (clear)
  {
    for (i = segment.min_index; i <= segment.max_index; i++)
    {
      segment.led_array[i] = CRGB::Black;
    }
  }
  delay(time);
}

void warmup()
{
  flashSegment(TASSEL);
  flashSegment(HAT);
  flashSegment(TRIM);
  flashSegment(RIGHT_EYE);
  flashSegment(LEFT_EYE);
  flashSegment(NOSE);
  flashSegment(MOUTH);
  flashSegment(COLLAR);
  flashSegment(RIGHT_ARM);
  flashSegment(LEFT_ARM);
  flashSegment(RIGHT_CUFF);
  flashSegment(LEFT_CUFF);
  flashSegment(RIGHT_FINGER);
  flashSegment(LEFT_FINGER);
  flashSegment(HEART_0, 100, true);
  flashSegment(HEART_1);
  flashSegment(HEART_2);
  flashSegment(HEART_3);
  flashSegment(BELT);
  delay(2000);
  FastLED.clear(true);
  delay(500);
}

void heartBeat()
{
  flashSegment(TASSEL, 0);
  flashSegment(HAT, 0);
  flashSegment(TRIM, 200);
  flashSegment(RIGHT_EYE, 0);
  flashSegment(LEFT_EYE, 0);
  flashSegment(NOSE, 0);
  flashSegment(MOUTH, 200);
  flashSegment(COLLAR, 0);
  flashSegment(RIGHT_ARM, 0);
  flashSegment(LEFT_ARM, 0);
  flashSegment(RIGHT_CUFF, 0);
  flashSegment(LEFT_CUFF, 0);
  flashSegment(RIGHT_FINGER, 0);
  flashSegment(LEFT_FINGER, 0);
  flashSegment(BELT, 200);
  for (int i = 0; i < 6; i++)
  {
    flashSegment(HEART_0, 100, true);
    flashSegment(HEART_1, 100, true);
    flashSegment(HEART_2, 100, true);
    flashSegment(HEART_3, 200, true);
  }
  delay(1000);
  FastLED.clear(true);
  // delay(500);
}

void allOn()
{
  // Loop over all grinchSegments
  int segmentCount = sizeof(grinchSegments) / sizeof(grinchSegments[0]);
  for (int i = 0; i < segmentCount; ++i)
  {
    LED_SEGMENT_RANGE &segment = grinchSegments[i];
    for (int j = segment.min_index; j <= segment.max_index; j++)
    {
      segment.led_array[j] = segment.defaultColor;
    }
  }
  FastLED.show();
  delay(500);
}

// Structure to hold information about each sparkling LED
struct Sparkle
{
  int index;               // LED index
  unsigned long startTime; // When the sparkle started
  bool isActive;           // If the sparkle is currently active
};

// Define LEDGroup structure
struct LEDGroup
{
  grinchSegmentIndex index;
  unsigned long sparkleDuration;
  unsigned long minTimeBetweenSparkles;
  int initialBrightness;
  unsigned long lastSparkleAttempt;
  unsigned long chaseSpeed;
  int trailLength;
  bool isReversed;
  std::vector<Sparkle> activeSparkles;
};

// Function to sparkle a single LED group
void sparkleLEDGroup(LEDGroup &group)
{
  unsigned long currentTime = millis();
  LED_SEGMENT_RANGE &segment = grinchSegments[group.index];

  // Attempt to add a new sparkle at intervals without blocking others
  if (currentTime - group.lastSparkleAttempt >= group.minTimeBetweenSparkles)
  {
    int firstLED = segment.min_index;
    int lastLED = segment.max_index;

    Sparkle newSparkle = {
        .index = static_cast<int>(random(lastLED - firstLED + 1) + firstLED), // Random LED index within the group
        .startTime = currentTime,                                             // Start time
        .isActive = true                                                      // Mark as active
    };
    group.activeSparkles.push_back(newSparkle); // Add new sparkle to the group's list
    group.lastSparkleAttempt = currentTime;     // Update the last sparkle attempt time
  }

  // Update all active sparkles in the group
  for (auto &sparkle : group.activeSparkles)
  {
    if (sparkle.isActive)
    {
      unsigned long elapsed = currentTime - sparkle.startTime;
      if (elapsed < group.sparkleDuration)
      {
        // Calculate the duration of one toggle cycle
        unsigned long toggleCycleDuration = group.sparkleDuration / 5; // 3 toggles = 6 states (on/off)

        // Determine the current state (toggle index)
        int toggleIndex = (elapsed / toggleCycleDuration) % 2; // 0 for high, 1 for low

        // Set brightness based on the toggle index
        float fadeRatio = (toggleIndex == 0) ? 1.0 : 0.5; // High (1.0) or low (0.5)

        // Apply fadeRatio to the color
        CRGB color = segment.defaultColor;
        color.r *= fadeRatio;
        color.g *= fadeRatio;
        color.b *= fadeRatio;

        // Apply brightness to the LED
        segment.led_array[sparkle.index] = color;
      }
      else
      {
        // End sparkle after duration
        segment.led_array[sparkle.index] = segment.defaultColor / 2;
        sparkle.isActive = false; // Mark as inactive
      }
    }
  }

  // Remove inactive sparkles
  group.activeSparkles.erase(
      std::remove_if(group.activeSparkles.begin(), group.activeSparkles.end(),
                     [](const Sparkle &s)
                     {
                       return !s.isActive;
                     }),
      group.activeSparkles.end());
}

// Function to chase a single LED group
void chaseLEDGroup(LEDGroup &group)
{
  unsigned long currentTime = millis();
  LED_SEGMENT_RANGE &segment = grinchSegments[group.index];

  // Calculate the total number of LEDs in the segment
  int firstLED = segment.min_index;
  int lastLED = segment.max_index;
  int numLEDs = lastLED - firstLED + 1;

  // Calculate the LED position based on time and chase speed
  unsigned long elapsed = currentTime - group.lastSparkleAttempt; // Time since the effect started
  int basePosition = (elapsed / group.chaseSpeed) % numLEDs;      // Base position without direction

  // Adjust position based on direction
  int ledPosition = group.isReversed ? (numLEDs - 1 - basePosition) : basePosition;

  // Reset all LEDs to default color (or off)
  for (int i = firstLED; i <= lastLED; i++)
  {
    segment.led_array[i] = segment.defaultColor / 8; // Dimmed default color
  }

  // Light up the current LED in the chase
  segment.led_array[firstLED + ledPosition] = segment.defaultColor;

  // Optionally, light up trailing LEDs for a smoother effect
  int trailLength = group.trailLength; // Number of LEDs to light up behind the current LED
  for (int i = 1; i <= trailLength; i++)
  {
    int trailPos = group.isReversed
                       ? (ledPosition + i) % numLEDs            // Forward when reversed
                       : (ledPosition - i + numLEDs) % numLEDs; // Backward otherwise

    float fadeRatio = 1.0 - (float)i / (trailLength + 1); // Decrease brightness for trailing LEDs
    CRGB color = segment.defaultColor;
    color.r *= fadeRatio;
    color.g *= fadeRatio;
    color.b *= fadeRatio;

    segment.led_array[firstLED + trailPos] = color;
  }
}

// Function to chase a single LED group
void smileLEDGroup(LEDGroup &group)
{
  unsigned long currentTime = millis();
  LED_SEGMENT_RANGE &segment = grinchSegments[group.index];

  // Calculate the total number of LEDs in the segment
  int firstLED = segment.min_index;
  int lastLED = segment.max_index;
  int numLEDStates = (lastLED - firstLED + 1) / 2 + 2; // with 7 leds, need 5 states including all off as one of them

  // Calculate the LED position based on time and chase speed
  unsigned long elapsed = currentTime - group.lastSparkleAttempt; // Time since the effect started
  int position = (elapsed / group.chaseSpeed) % numLEDStates;     // Base position without direction

  for (int j = firstLED; j <= lastLED; j++)
  {
    segment.led_array[j] = segment.defaultColor / 16;
  }

  for (int i = 0; i < position; i++)
  {
    segment.led_array[firstLED + 3 - i] = segment.defaultColor;
    segment.led_array[firstLED + 3 + i] = segment.defaultColor;
  }
}

void heartbeatLEDGroup(LEDGroup &group)
{
  unsigned long currentTime = millis();
  LED_SEGMENT_RANGE &segment = grinchSegments[group.index];

  // Calculate the total number of LEDs in the segment
  int firstLED = segment.min_index;
  int lastLED = segment.max_index;

  // Time since the heartbeat started
  unsigned long elapsed = currentTime - group.lastSparkleAttempt;

  // Total duration of one heartbeat cycle
  unsigned long heartbeatDuration = 1500; // group.heartbeatDuration;

  // Normalize elapsed time within the heartbeat cycle
  float cyclePosition = (float)(elapsed % heartbeatDuration) / heartbeatDuration;

  // Calculate brightness based on a heartbeat curve
  float brightness = 0.0;
  if (cyclePosition < 0.25) // First strong beat
  {
    brightness = sin(PI * (cyclePosition / 0.25)); // Quick rise and fall
  }
  else if (cyclePosition < 0.5) // Second smaller beat
  {
    brightness = 0.5 * sin(PI * ((cyclePosition - 0.25) / 0.25)); // Smaller pulse
  }
  else // Rest phase
  {
    brightness = 0.0;
  }

  // Apply brightness to all LEDs in the group
  for (int i = firstLED; i <= lastLED; i++)
  {
    CRGB color = segment.defaultColor;
    color.r *= brightness;
    color.g *= brightness;
    color.b *= brightness;
    segment.led_array[i] = color;
  }

  // Reset the start time for the next heartbeat
  if (elapsed >= heartbeatDuration)
  {
    group.lastSparkleAttempt = currentTime;
  }
}

void whiteSparkleV3()
{
  // { grinchSegmentIndex, sparkleDuration, minTimeBetweenSparkles, initialBrightness, lastSparkleAttempt, chaseSpeed, trailLength, isReversed}
  std::vector<LEDGroup> ledSparkleGroup = {
      {TASSEL, 200, 1000, 127, 0, 0, 0, false, {}},
      {TRIM, 200, 100, 127, 0, 0, 0, false, {}},
      {COLLAR, 200, 100, 127, 0, 0, 0, false, {}},
      {RIGHT_CUFF, 200, 2000, 127, 0, 0, 0, false, {}},
      {LEFT_CUFF, 200, 2000, 127, 0, 0, 0, false, {}},
      {BELT, 200, 100, 127, 0, 0, 0, false, {}},
  };

  std::vector<LEDGroup> ledChaseGroup = {
      {HAT, 200, 1000, 127, 0, 50, 3, false, {}},
      {LEFT_ARM, 200, 1000, 127, 0, 100, 0, true, {}},
      {RIGHT_ARM, 200, 1000, 127, 0, 100, 0, false, {}},
      {HEART_3, 200, 1000, 127, 0, 50, 3, false, {}},
  };

  std::vector<LEDGroup> ledBeatGroup = {
      {HEART_1, 200, 1000, 127, 0, 50, 3, false, {}},
  };

  std::vector<LEDGroup> ledSmileGroup = {
      {MOUTH, 200, 1000, 127, 0, 200, 0, false, {}},
  };

  FastLED.clear(true);
  flashSegment(RIGHT_EYE, 0);
  flashSegment(LEFT_EYE, 0);
  flashSegment(NOSE, 0);
  flashSegment(RIGHT_FINGER, 0);
  flashSegment(LEFT_FINGER, 0);

  // Initialize sparkle groups
  for (auto &group : ledSparkleGroup)
  {
    LED_SEGMENT_RANGE &segment = grinchSegments[group.index];
    CRGB initialColor = segment.defaultColor / 2;
    for (int i = segment.min_index; i <= segment.max_index; i++)
    {
      segment.led_array[i] = initialColor;
    }
  }

  // Get the start time
  unsigned long startTime = millis();

  while (true)
  {
    // Calculate elapsed time
    unsigned long elapsedTime = millis() - startTime;

    // Exit loop after 10 seconds
    if (elapsedTime >= 10000)
    {
      break;
    }

    for (auto &group : ledSparkleGroup)
    {
      sparkleLEDGroup(group);
    }

    for (auto &group : ledChaseGroup)
    {
      chaseLEDGroup(group);
    }

    for (auto &group : ledBeatGroup)
    {
      heartbeatLEDGroup(group);
    }
    for (auto &group : ledSmileGroup)
    {
      smileLEDGroup(group);
    }

    // Show LED updates and maintain 120 FPS loop using FastLED.delay()
    FastLED.show();
    FastLED.delay(8); // Approximate 120 frames per second
  }

  FastLED.clear(true);
}

void loop()
{
  warmup();
  heartBeat();
  // allOn();
  whiteSparkleV3();
}
