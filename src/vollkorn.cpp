#include "daisy_seed.h"
#include "daisysp.h"
#include "dev/oled_sh1106.h"
#include "hid/disp/graphics_common.h"
#include "hid/disp/oled_display.h"
#include "util/oled_fonts.h"

#define MAX_DELAY static_cast<size_t>(48000 * 2.0f)  // 2 second delay
#define MAX_GRAINS 18                                // More grains for richer texture
#define MIN_GRAIN_SIZE 0.02f                         // 20ms minimum
#define MAX_GRAIN_SIZE 0.5f                          // 500ms maximum
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

using namespace daisy;
using namespace daisysp;
using Display = OledDisplay<SH1106I2c128x64Driver>;

DaisySeed hw;
Display oled;

size_t writePos = 0;

bool splash;

struct Grain {
  bool active;
  float position;   // Position in delay buffer
  float length;     // Length in samples
  float age;        // Current age
  float pan;        // Stereo position
  float amplitude;  // Volume envelope
  float pitch;      // Pitch variation
  float direction;  // 1.0 for forward, -1.0 for reverse
} __attribute__((aligned(4)));

struct GrainVisual {
  int x;
  int y;
  int size;
  float brightness;
} __attribute__((aligned(4)));

// Delay lines in SDRAM
DelayLine<float, MAX_DELAY> DSY_SDRAM_BSS delayL;
DelayLine<float, MAX_DELAY> DSY_SDRAM_BSS delayR;

struct {
  float sample_rate;
  float mix;        // Wet/dry
  float time;       // Delay time
  float feedback;   // Delay feedback
  float grainSize;  // 20-500ms
  float density;    // Spawn probability
  float spread;     // Stereo spread
  float pitch;      // Pitch variation

  Grain grains[MAX_GRAINS];
  GrainVisual grainVisuals[MAX_GRAINS];
} state;

Switch stageSwitch;
Switch freezeSwitch;

enum AdcChannel {
  mixKnob = 0,
  timeKnob = 1,
  feedbackKnob = 2,
  sizeKnob = 3,
  densityKnob = 4,
  spreadKnob = 5,
  pitchVarKnob = 6,
  NUM_ADC_CHANNELS
};

MappedFloatValue mixValue(0.0f, 1.0f, 0.5f);

void showSplashScreen() {
  if (!splash) {
    return;
  }
  oled.Fill(false);

  oled.WriteStringAligned("S P V C E", Font_11x18, oled.GetBounds(), Alignment::centered, true);
  oled.WriteStringAligned("by david werth", Font_6x7, oled.GetBounds(), Alignment::bottomCentered, true);
  oled.Update();
  hw.DelayMs(3210);
  oled.Fill(false);
  oled.Update();
  splash = false;
}

void showInterface() {
  oled.Fill(false);

  // Add interval mode display
  const char* modeNames[] = {"UNISON", "OCTAVES", "FIFTHS", "OCT+5TH", "2 OCTAVES"};
  int modeIndex = static_cast<int>(state.pitch * (4.0f / 0.97f));  // Compensate for ADC range
  modeIndex = fmin(modeIndex, 4);                                  // Clamp to prevent array overflow
  oled.SetCursor(2, 2);
  oled.WriteString(modeNames[modeIndex], Font_6x7, true);

  // Draw grain visualizations with enhanced 3D effect
  for (int i = 0; i < MAX_GRAINS; i++) {
    if (state.grains[i].active) {
      GrainVisual& vis = state.grainVisuals[i];
      Grain& grain = state.grains[i];

      float lifePhase = grain.age / grain.length;
      float brightness = (1.0f - lifePhase) * vis.brightness;

      if (brightness > 0.0f) {
        // Center point
        oled.DrawPixel(vis.x, vis.y, true);

        // Inner ring
        if (brightness > 0.3f && vis.size > 1) {
          for (int angle = 0; angle < 8; angle++) {
            float rad = angle * M_PI / 4.0f;
            int dx = roundf(cosf(rad) * vis.size);
            int dy = roundf(sinf(rad) * vis.size);
            oled.DrawPixel(vis.x + dx, vis.y + dy, true);
          }
        }

        // Outer glow
        if (brightness > 0.6f && vis.size > 2) {
          for (int angle = 0; angle < 4; angle++) {
            float rad = angle * M_PI / 2.0f;
            int dx = roundf(cosf(rad) * vis.size * 1.5f);
            int dy = roundf(sinf(rad) * vis.size * 1.5f);
            oled.DrawPixel(vis.x + dx, vis.y + dy, true);
          }
        }
      }
    }
  }
  oled.Update();
}

inline float ProcessGrain(Grain& grain) {
  // Increment grain age
  grain.age += 1.0f;

  // Normalized phase (0 to 1)
  const float phase = grain.age / grain.length;

  // Hanning window envelope for smooth grains
  const float envelope = 0.5f * (1.0f - cosf(phase * M_PI * 2.0f));

  // Simple read position calculation - direction controls forward/backward,
  // pitch controls speed independently
  float readPos = grain.position + (grain.age * grain.pitch * grain.direction);

  // Wrap around the delay buffer
  readPos = fmodf(readPos, static_cast<float>(MAX_DELAY));
  if (readPos < 0)
    readPos += MAX_DELAY;

  // Read from delay lines with interpolation
  float sampL = delayL.Read(readPos);
  float sampR = delayR.Read(readPos);

  return (sampL + sampR) * 0.5f * envelope * grain.amplitude;
}

void spawnGrainVisual(int grainIndex) {
  GrainVisual& vis = state.grainVisuals[grainIndex];
  Grain& grain = state.grains[grainIndex];

  float margin = 20;
  float centerX = SCREEN_WIDTH / 2.0f;
  float centerY = SCREEN_HEIGHT / 2.0f;
  float spread = state.spread * (SCREEN_WIDTH - 2 * margin);

  // Calculate initial position - invert pan mapping to match audio
  float rawX = centerX - (grain.pan - 0.5f) * spread;
  // Add slight curve to Y position for more depth
  float yOffset = (Random::GetFloat() - 0.5f) * (SCREEN_HEIGHT - 20);
  float rawY = centerY + yOffset * (1.0f - abs(grain.pan - 0.5f) * 0.5f);

  // Calculate distance from center
  float dx = rawX - centerX;
  float dy = rawY - centerY;
  float distance = sqrtf(dx * dx + dy * dy);

  // Constrain to circle if outside
  const float maxRadius = 25.0f;
  if (distance > maxRadius) {
    float scale = maxRadius / distance;
    dx *= scale;
    dy *= scale;
    vis.x = centerX + dx;
    vis.y = centerY + dy;
  } else {
    vis.x = rawX;
    vis.y = rawY;
  }

  // Size varies based on Y position to create depth perception
  float depthFactor = 1.0f - (abs(vis.y - centerY) / (SCREEN_HEIGHT * 0.5f));
  vis.size = 1 + static_cast<int>(depthFactor * 4);
  vis.brightness = 0.3f + (depthFactor * 0.7f);
}

void SpawnGrain() {
  // Calculate input amplitude (average of both channels)
  float inputAmplitude = 0.0f;
  for (size_t i = 0; i < 32; i++)  // Using block size of 32
  {
    float sampleL = delayL.Read(writePos - i);
    float sampleR = delayR.Read(writePos - i);
    inputAmplitude += (fabsf(sampleL) + fabsf(sampleR)) * 0.5f;
  }
  inputAmplitude /= 32.0f;

  // Only spawn if input amplitude is above threshold
  if (inputAmplitude < 0.01f)  // Adjust threshold as needed
    return;

  // Count active grains
  int activeGrains = 0;
  for (int i = 0; i < MAX_GRAINS; i++) {
    if (state.grains[i].active)
      activeGrains++;
  }

  // Only spawn if we're under 75% capacity to prevent overload
  if (activeGrains < (MAX_GRAINS * 3) / 4) {
    for (int i = 0; i < MAX_GRAINS; i++) {
      if (!state.grains[i].active) {
        Grain& g = state.grains[i];
        g.active = true;

        // Calculate base position from delay time
        float basePos = state.time * state.sample_rate;

        // Add random time spread based on feedback knob (repurposed as time spread)
        // Maximum spread is the full delay buffer length
        float maxSpread = state.sample_rate * 2.0f;     // 2 seconds max spread
        float timeSpread = state.feedback * maxSpread;  // feedback is now our spread control
        float randomOffset = (Random::GetFloat() * 2.0f - 1.0f) * timeSpread;

        // Calculate final position with proper wrapping
        float delayPos = writePos - basePos + randomOffset;

        // Ensure proper wrapping for negative positions
        while (delayPos < 0) {
          delayPos += MAX_DELAY;
        }
        g.position = fmodf(delayPos, MAX_DELAY);

        g.length = state.grainSize * state.sample_rate;
        g.age = 0;
        g.pan = 0.5f + (state.spread * (Random::GetFloat() - 0.5f));
        g.amplitude = 0.7f + (Random::GetFloat() * 0.3f);

        // Define interval modes (from simple to complex)
        const float intervalModesBackwards[][5] = {
            {2.0f, 2.0f, 2.0f, 2.0f, 2.0f},   // Mode 0: Unison only
            {0.5f, 1.0f, 3.0f, 0.0f, 0.0f},   // Mode 1: Octaves (-12, 0, +12 semitones)
            {0.33f, 1.0f, 4.0f, 0.0f, 0.0f},  // Mode 2: Fifths (-7, 0, +19 semitones)
            {0.5f, 0.33f, 1.0f, 4.0f, 3.0f},  // Mode 3: Octaves and fifths mixed
            {0.33f, 0.5f, 1.0f, 3.0f, 4.0f}   // Mode 4: Full range of intervals
        };
        const float intervalModesForwards[][5] = {
            {-1.0f, -1.0f, -1.0f, -1.0f, -1.0f},  // Mode 0: Unison only
            {0.0f, 1.0f, 0.0f, -1.0f, 0.0f},      // Mode 1: Octaves (-12, 0, +12 semitones)
            {0.5f, 0.0f, 0.0f, 0.5f, 0.0f},       // Mode 2: Fifths (-7, 0, +19 semitones)
            {0.5f, 0.33f, 1.0f, 4.0f, 3.0f},      // Mode 3: Octaves and fifths mixed
            {0.33f, 0.5f, 1.0f, 3.0f, 4.0f}       // Mode 4: Full range of intervals
        };

        const int intervalsPerMode[] = {1, 3, 3, 5, 5};

        // Select mode based on pitch knob
        int modeIndex = static_cast<int>(state.pitch * (4.0f / 0.97f));  // Compensate for ADC range
        modeIndex = daisysp::fmin(modeIndex,
                                  4);  // Clamp to prevent array overflow

        // Randomize direction (50% chance of reverse) 1.0f is backwards, -1.0f is forwards
        // g.direction = (Random::GetFloat() > 0.8f) ? 1.0f : -1.0f;
        g.direction = 1.0f;

        // Select one interval from the current mode using the correct count
        int intervalIndex = static_cast<int>(Random::GetFloat() * intervalsPerMode[modeIndex]);
        float semitones = g.direction == -1.0f ? intervalModesForwards[modeIndex][intervalIndex]
                                               : intervalModesBackwards[modeIndex][intervalIndex];
        g.pitch = semitones;

        spawnGrainVisual(i);
        spawnGrainVisual(i);
        break;  // Exit the loop after spawning one grain
      }
    }
  }
}

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size) {
  hw.adc.Start();

  // Normalize ADC readings from 0-0.97 to 0-1.0
  auto normalizeKnob = [](float value) { return std::fmin(value / 0.97f, 1.0f); };

  float currentKnobValue = normalizeKnob(hw.adc.GetFloat(mixKnob));

  mixValue.Set(currentKnobValue);
  state.mix = mixValue.Get();

  // Always update other grain parameters
  state.feedback = normalizeKnob(hw.adc.GetFloat(feedbackKnob)) * 0.7f;
  state.time = 0.05f + (normalizeKnob(hw.adc.GetFloat(timeKnob)) * 1.95f);
  state.grainSize = MIN_GRAIN_SIZE + (normalizeKnob(hw.adc.GetFloat(sizeKnob)) * (MAX_GRAIN_SIZE - MIN_GRAIN_SIZE));
  float rawDensity = normalizeKnob(hw.adc.GetFloat(densityKnob));
  state.density = (rawDensity * rawDensity * rawDensity) * 0.008f;
  state.spread = normalizeKnob(hw.adc.GetFloat(spreadKnob));
  state.pitch = normalizeKnob(hw.adc.GetFloat(pitchVarKnob));

  for (size_t i = 0; i < size; i++) {
    float wetL = 0.0f;
    float wetR = 0.0f;

    // Spawn new grains based on density
    if (Random::GetFloat() < state.density) {
      SpawnGrain();
    }

    // Process active grains
    for (int g = 0; g < MAX_GRAINS; g++) {
      if (state.grains[g].active) {
        if (state.grains[g].age >= state.grains[g].length) {
          state.grains[g].active = false;
        } else {
          float grain_out = ProcessGrain(state.grains[g]);
          wetL += grain_out * (1.0f - state.grains[g].pan);
          wetR += grain_out * state.grains[g].pan;
        }
      }
    }

    // Mix dry and wet signals
    out[0][i] = in[0][i] * (1.0f - state.mix) + wetL * state.mix;
    out[1][i] = in[1][i] * (1.0f - state.mix) + wetR * state.mix;

    // Write to delay lines (only feedback the dry signal)
    delayL.Write(in[0][i] * (1.0f + state.feedback));
    delayR.Write(in[1][i] * (1.0f + state.feedback));
  }
}

int main(void) {
  hw.Configure();
  hw.Init();
  delayL.Init();
  delayR.Init();
  hw.SetAudioBlockSize(32);  // Small block size for lower latency

  state.sample_rate = hw.AudioSampleRate();

  Display::Config disp_config;
  oled.Init(disp_config);

  // Initialize ADC
  AdcChannelConfig adc_config[NUM_ADC_CHANNELS];
  adc_config[AdcChannel::mixKnob].InitSingle(seed::D21);
  adc_config[AdcChannel::timeKnob].InitSingle(seed::D15);
  adc_config[AdcChannel::feedbackKnob].InitSingle(seed::D16);
  adc_config[AdcChannel::sizeKnob].InitSingle(seed::D17);
  adc_config[AdcChannel::densityKnob].InitSingle(seed::D18);
  adc_config[AdcChannel::spreadKnob].InitSingle(seed::D19);
  adc_config[AdcChannel::pitchVarKnob].InitSingle(seed::D20);
  hw.adc.Init(adc_config, NUM_ADC_CHANNELS);

  // Initialize grains and visuals
  for (int i = 0; i < MAX_GRAINS; i++) {
    state.grains[i].active = false;
    state.grainVisuals[i].x = 0;
    state.grainVisuals[i].y = 0;
    state.grainVisuals[i].size = 2;
    state.grainVisuals[i].brightness = 0.0f;
  }

  stageSwitch.Init(seed::D22, 1000.0f);
  freezeSwitch.Init(seed::D23, 1000.0f);

  hw.adc.Start();
  hw.StartAudio(AudioCallback);

  splash = true;

  while (1) {
    showSplashScreen();
    showInterface();
  }
}
