#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <ctype.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>
#include "src/web/web_page.h"

// Keyestudio KS5016 ESP32 PLUS / ESP32-WROOM-32.
// Prototipo experimental: no es un dispositivo medico.

constexpr uint8_t SENSOR_PIN = 34;  // ADC1, entrada solamente, compatible con WiFi.
constexpr uint8_t MOTOR_PIN = 25;    // D3 / GPIO25
constexpr uint32_t SAMPLE_RATE_HZ = 50;
constexpr uint32_t SAMPLE_PERIOD_MS = 1000 / SAMPLE_RATE_HZ;
constexpr uint16_t ADC_MAX_VALUE = 4095;

// ====================== PARAMETROS AJUSTABLES ======================
// Adquisicion: ocho conversiones y media recortada (descarta min y max).
constexpr uint8_t ADC_OVERSAMPLES = 8;
// Banda PPG: los coeficientes de los biquads inferiores son 0,5-5 Hz a 50 Hz.
constexpr float MIN_SIGNAL_AMPLITUDE_ADC = 4.0f;
constexpr float CALIBRATION_NOISE_MULTIPLIER = 3.0f;
constexpr float ADAPTIVE_THRESHOLD_FRACTION = 0.38f;
constexpr float IBI_OUTLIER_FRACTION = 0.28f;
constexpr uint32_t REFRACTORY_MS = 300;
constexpr uint32_t MAX_IBI_MS = 2000;  // 30 BPM para un intervalo individual.
constexpr uint32_t MAX_PEAK_GAP_MS = 4500;
constexpr uint32_t BPM_HOLD_MS = 6000;
constexpr uint32_t DETECTOR_SETTLE_MS = 2000;
constexpr uint8_t MIN_INTERVALS_FOR_BPM = 4;
constexpr uint8_t MIN_INTERVALS_FOR_AVERAGE = 8;
// Respiracion derivada de tres modulaciones de la misma PPG (RIIV, RIAV, RIFV).
// Ventanas de 32-60 s a 5 Hz, como las empleadas en validaciones de RR por PPG.
constexpr uint8_t RESPIRATION_SAMPLE_HZ = 5;
constexpr uint8_t RESPIRATION_DOWNSAMPLE = SAMPLE_RATE_HZ / RESPIRATION_SAMPLE_HZ;
constexpr size_t RESPIRATION_WINDOW = 60 * RESPIRATION_SAMPLE_HZ;
constexpr size_t RESPIRATION_MIN_SAMPLES = 32 * RESPIRATION_SAMPLE_HZ;
constexpr float RESPIRATION_MIN_RPM = 6.0f;
constexpr float RESPIRATION_MAX_RPM = 36.0f;
constexpr float RESPIRATION_STEP_RPM = 0.5f;
constexpr size_t RESPIRATION_FREQUENCIES = 61;
constexpr uint32_t DERIVED_UPDATE_MS = 5000;
constexpr uint32_t STRESS_WINDOW_MS = 60000;
constexpr uint32_t STRESS_MIN_WINDOW_MS = 30000;
constexpr uint32_t DERIVED_RESULT_GRACE_MS = 15000;
constexpr size_t PRV_HISTORY = 220;
// ===================================================================

constexpr char FIRMWARE_VERSION[] = "leire-tdr-web-1.7.0-44-live";

constexpr char AP_SSID[] = "fotopletismografia";
constexpr char AP_PASSWORD[] = "12345678";
constexpr char LOG_PATH[] = "/calibration.log";
constexpr char LOG_OLD_PATH[] = "/calibration.previous.log";
constexpr size_t MAX_LOG_BYTES = 64 * 1024;

constexpr uint32_t PREPARE_MS = 3000;
constexpr uint32_t CALIBRATION_SAMPLE_MS = 10000;
constexpr size_t CALIBRATION_CAPACITY = 510;
constexpr size_t MIN_CALIBRATION_SAMPLES = 450;
constexpr size_t SIGNAL_WINDOW = 100;  // 2 s para amplitud/umbral adaptativos.
constexpr size_t IBI_HISTORY = 9;
// Seis segundos de señal: suficiente para ver varios ciclos completos incluso
// con un pulso lento. El navegador pide después únicamente las muestras nuevas.
constexpr size_t GRAPH_HISTORY = 300;

WebServer server(80);
Preferences preferences;
portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;

struct Biquad {
  float b0, b1, b2, a1, a2;
  float x1 = 0, x2 = 0, y1 = 0, y2 = 0;

  float process(float x) {
    const float y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
    x2 = x1;
    x1 = x;
    y2 = y1;
    y1 = y;
    return y;
  }

  void reset() { x1 = x2 = y1 = y2 = 0; }
};

Biquad highPass{0.95654323f, -1.91308645f, 0.95654323f,
                -1.91119707f, 0.91497583f};
Biquad lowPass{0.06745527f, 0.13491055f, 0.06745527f,
               -1.14298050f, 0.41280160f};

enum class CalibrationPhase : uint8_t {
  IDLE,
  PREPARE_FINGER,
  SAMPLE_FINGER,
  PREPARE_NO_FINGER,
  SAMPLE_NO_FINGER,
  COMPLETE,
  FAILED
};

struct PublicState {
  uint16_t raw = 0;
  float filtered = 0;
  bool calibrated = false;
  bool finger = false;
  CalibrationPhase phase = CalibrationPhase::IDLE;
  uint32_t phaseStartedMs = 0;
  const char *calibrationMessage = "Inicia el mapeo del sensor.";
  float bpmCurrent = 0;
  float bpmAverage = 0;
  bool bpmValid = false;
  bool bpmAverageReady = false;
  uint16_t beatCount = 0;
  uint32_t pulseAccepted = 0;
  uint32_t pulseRejected = 0;
  uint32_t pulseRecovered = 0;
  float quality = 0;
  const char *qualityState = "no_finger";
  float respirationRate = 0;
  float respirationConfidence = 0;
  float respirationWindowSeconds = 0;
  bool respirationValid = false;
  uint8_t respirationSources = 0;
  float stressScore = 0;
  float stressConfidence = 0;
  float stressWindowSeconds = 0;
  float rmssdMs = 0;
  float baevskyStressIndex = 0;
  bool stressValid = false;
  bool stressUsesRespiration = false;
  const char *stressState = "unavailable";
  float threshold = 0;
  bool peak = false;
  uint32_t samples = 0;
  uint32_t missed = 0;
};

PublicState publicState;
// OLED 0.91 pulgadas ver1.6, SSD1306 128x32, BUS I2C de la STEAMakers.
// Solo la tarea OLED utiliza Wire; nunca se transmite I2C dentro del sampler.
constexpr uint8_t OLED_SDA = 21;
constexpr uint8_t OLED_SCL = 22;
constexpr uint32_t OLED_FRAME_MS = 50;  // 20 FPS, ~12 ms por imagen a 400 kHz.
constexpr uint8_t OLED_ROTATION = 2;   // Giro de 180 grados para su montaje.
Adafruit_SSD1306 oled(128, 32, &Wire, -1, 400000, 400000);
bool oledReady = false;
uint8_t oledAddress = 0;
float graphRing[GRAPH_HISTORY] = {0};
size_t graphIndex = 0;
size_t graphCount = 0;
volatile bool calibrationStartRequested = false;

struct CalibrationLogEntry {
  uint32_t uptimeMs;
  const char *result;
  uint16_t fingerCount;
  uint16_t noFingerCount;
  uint16_t fingerMedian;
  uint16_t noFingerMedian;
  uint16_t fingerMad;
  uint16_t noFingerMad;
  uint16_t fingerMin;
  uint16_t fingerMax;
  uint16_t noFingerMin;
  uint16_t noFingerMax;
  uint16_t difference;
  uint16_t robustNoise;
  uint32_t missed;
};

CalibrationLogEntry pendingLog{};
volatile bool pendingLogReady = false;
bool filesystemReady = false;
bool preferencesReady = false;

CalibrationPhase phase = CalibrationPhase::IDLE;
uint32_t phaseStartedMs = 0;
const char *phaseMessage = "Inicia el mapatge del sensor.";
uint16_t fingerSamples[CALIBRATION_CAPACITY];
uint16_t noFingerSamples[CALIBRATION_CAPACITY];
uint16_t scratch[CALIBRATION_CAPACITY];
size_t fingerSampleCount = 0;
size_t noFingerSampleCount = 0;
uint16_t fingerMedian = 0, noFingerMedian = 0;
uint16_t fingerMad = 0, noFingerMad = 0;
uint16_t fingerMin = ADC_MAX_VALUE, fingerMax = 0;
uint16_t noFingerMin = ADC_MAX_VALUE, noFingerMax = 0;
uint32_t missedAtCalibrationStart = 0;

uint16_t rawValue = 0;
float filteredValue = 0;
float dcValue = 0;
bool dcInitialized = false;
bool calibrated = false;
bool fingerPresent = false;
bool fingerCandidate = false;
uint32_t fingerCandidateSinceMs = 0;
uint32_t fingerPresentSinceMs = 0;

float signalRing[SIGNAL_WINDOW] = {0};
size_t signalIndex = 0, signalCount = 0;
float recentAmplitude = 0;
float adaptiveThreshold = 0;
float signalPeakLevel = 0;
float noisePeakLevel = 0;
float detectorPrevious2 = 0, detectorPrevious1 = 0;
uint32_t detectorPrevious1Ms = 0;
uint16_t polarityLearningSamples = 0;
float positiveEnvelope = 0, negativeEnvelope = 0;
int8_t signalPolarity = 1;
float ppgValue = 0;
bool peakThisSample = false;
volatile bool serialPlotterEnabled = false;
uint32_t lastPeakMs = 0;
uint32_t lastReliablePulseMs = 0;
uint32_t pulseAcceptedCount = 0;
uint32_t pulseRejectedCount = 0;
uint32_t pulseRecoveredCount = 0;
uint16_t ibis[IBI_HISTORY] = {0};
size_t ibiCount = 0, ibiIndex = 0;
float bpmCurrent = 0, bpmAverage = 0, bpmSum = 0;
uint16_t bpmAverageCount = 0;
bool bpmValid = false, bpmAverageReady = false;
float signalQuality = 0;

// Tres series respiratorias a 5 Hz: intensidad (RIIV), amplitud (RIAV) y
// frecuencia del pulso (RIFV). Se fusionan solo si sus espectros concuerdan.
float respirationFeatures[3][RESPIRATION_WINDOW] = {{0}};
float respirationWork[RESPIRATION_WINDOW] = {0};
float respirationClean[RESPIRATION_WINDOW] = {0};
float respirationSpectrum[RESPIRATION_FREQUENCIES] = {0};
uint8_t respirationSignalQuality[RESPIRATION_WINDOW] = {0};
size_t respirationIndex = 0, respirationCount = 0;
uint8_t respirationDecimator = 0;
float latestPulseAmplitude = 0;
uint16_t latestAcceptedIbi = 0;
float respirationRate = 0, respirationConfidence = 0;
bool respirationValid = false;
uint8_t respirationSources = 0;
float respirationCandidate = 0;
uint8_t respirationCandidateStreak = 0;
uint32_t respirationLastSuccessMs = 0;
uint32_t lastDerivedUpdateMs = 0, derivedStartedMs = 0;
bool derivedCollecting = false;

uint16_t prvIntervals[PRV_HISTORY] = {0};
uint32_t prvTimes[PRV_HISTORY] = {0};
size_t prvIndex = 0, prvCount = 0;
float stressScore = 0, stressConfidence = 0;
float rmssdMs = 0, baevskyStressIndex = 0;
bool stressValid = false;
bool stressUsesRespiration = false;
uint32_t stressLastSuccessMs = 0;
const char *stressState = "unavailable";

uint32_t totalSamples = 0, missedSamples = 0;
uint64_t lastSampleUs = 0;
char serialCommand[24];
size_t serialCommandLength = 0;

// ====================== BIOFEEDBACK MOTOR ======================
// Guia 4-4 durante aproximadamente un minuto (ocho ciclos):
//   inspira: doble vibracion; expira: una vibracion.
// Las vibraciones solo anuncian el cambio de fase; el motor permanece apagado
// durante el resto de cada intervalo para no enmascarar la medida PPG.
constexpr uint32_t BREATHING_PHASE_MS = 4000;
constexpr uint32_t BREATHING_CYCLE_MS = 2 * BREATHING_PHASE_MS;
constexpr uint8_t BREATHING_CYCLES = 8;
constexpr uint32_t BREATHING_TOTAL_MS = BREATHING_CYCLES * BREATHING_CYCLE_MS;
constexpr uint32_t MOTOR_PULSE_MS = 180;
constexpr uint32_t MOTOR_PULSE_GAP_MS = 140;
constexpr uint32_t POST_ANALYSIS_MIN_MS = 32000;
constexpr uint32_t DURING_SAMPLE_MS = 1000;
constexpr uint8_t DURING_MIN_SAMPLES = 8;

volatile bool breathingStartRequested = false;
volatile bool breathingRunning = false;
volatile bool postAnalysisPulseResetRequested = false;
uint8_t breathingPhase = 0;
uint32_t breathingStartedMs = 0;
uint32_t breathingPhaseStartedMs = 0;
bool motorOn = false;
bool postAnalysisRunning = false;
bool comparisonReady = false;
uint32_t postAnalysisStartedMs = 0;
float beforeBpm = 0, beforeRespiration = 0, beforeStress = 0;
float duringBpm = 0, duringRespiration = 0, duringStress = 0;
float afterBpm = 0, afterRespiration = 0, afterStress = 0;
float duringBpmSum = 0, duringRespirationSum = 0, duringStressSum = 0;
uint16_t duringSampleCount = 0;
uint32_t lastDuringSampleMs = 0;
bool duringMeasurementReady = false;

void setMotor(bool on) {
  motorOn = on;
  digitalWrite(MOTOR_PIN, on ? HIGH : LOW);
}

void startBreathingPattern(uint32_t now) {
  breathingRunning = true;
  breathingPhase = 1;
  breathingStartedMs = now;
  breathingPhaseStartedMs = now;
  duringBpm = duringRespiration = duringStress = 0;
  duringBpmSum = duringRespirationSum = duringStressSum = 0;
  duringSampleCount = 0;
  lastDuringSampleMs = 0;
  duringMeasurementReady = false;
  setMotor(true);
}

void stopBreathingPattern() {
  breathingRunning = false;
  breathingPhase = 0;
  setMotor(false);
}

void updateBreathingPattern(uint32_t now) {
  if (breathingStartRequested && !breathingRunning) {
    breathingStartRequested = false;
    startBreathingPattern(now);
  }

  if (!breathingRunning) return;

  // Un reloj absoluto evita que las peticiones web acumulen retraso entre fases.
  const uint32_t elapsed = now - breathingStartedMs;
  if (elapsed >= BREATHING_TOTAL_MS) {
    stopBreathingPattern();
    if (duringSampleCount) {
      duringBpm = duringBpmSum / duringSampleCount;
      duringRespiration = duringRespirationSum / duringSampleCount;
      duringStress = duringStressSum / duringSampleCount;
      duringMeasurementReady = duringSampleCount >= DURING_MIN_SAMPLES;
    }
    // El muestreador reiniciara tambien los intervalos de pulso. Asi la
    // medicion posterior no reutiliza ningun latido de la linea base.
    postAnalysisPulseResetRequested = true;
    postAnalysisRunning = true;
    postAnalysisStartedMs = now;
    return;
  }
  const uint32_t cycleElapsed = elapsed % BREATHING_CYCLE_MS;
  const uint8_t phase = cycleElapsed < BREATHING_PHASE_MS ? 1 : 2;
  const uint32_t phaseElapsed =
      cycleElapsed - static_cast<uint32_t>(phase - 1) * BREATHING_PHASE_MS;
  breathingPhase = phase;
  breathingPhaseStartedMs = now - phaseElapsed;

  const bool doubleCue = phase == 1;
  const uint32_t secondPulseStart = MOTOR_PULSE_MS + MOTOR_PULSE_GAP_MS;
  const bool firstPulse = phaseElapsed < MOTOR_PULSE_MS;
  const bool secondPulse = doubleCue && phaseElapsed >= secondPulseStart &&
                           phaseElapsed < secondPulseStart + MOTOR_PULSE_MS;
  setMotor(firstPulse || secondPulse);
}

void collectBreathingMeasurement(uint32_t now) {
  if (!breathingRunning ||
      (lastDuringSampleMs && now - lastDuringSampleMs < DURING_SAMPLE_MS))
    return;
  lastDuringSampleMs = now;

  PublicState state;
  portENTER_CRITICAL(&stateMux);
  state = publicState;
  portEXIT_CRITICAL(&stateMux);
  if (!state.finger || !state.bpmValid || !state.respirationValid ||
      !state.stressValid)
    return;

  duringBpmSum += state.bpmCurrent;
  duringRespirationSum += state.respirationRate;
  duringStressSum += state.stressScore;
  ++duringSampleCount;
  duringBpm = duringBpmSum / duringSampleCount;
  duringRespiration = duringRespirationSum / duringSampleCount;
  duringStress = duringStressSum / duringSampleCount;
  duringMeasurementReady = duringSampleCount >= DURING_MIN_SAMPLES;
}



bool calibrationRunning(CalibrationPhase value) {
  return value == CalibrationPhase::PREPARE_FINGER ||
         value == CalibrationPhase::SAMPLE_FINGER ||
         value == CalibrationPhase::PREPARE_NO_FINGER ||
         value == CalibrationPhase::SAMPLE_NO_FINGER;
}

const char *phaseName(CalibrationPhase value) {
  switch (value) {
    case CalibrationPhase::PREPARE_FINGER: return "prepare_finger";
    case CalibrationPhase::SAMPLE_FINGER: return "sample_finger";
    case CalibrationPhase::PREPARE_NO_FINGER: return "prepare_no_finger";
    case CalibrationPhase::SAMPLE_NO_FINGER: return "sample_no_finger";
    case CalibrationPhase::COMPLETE: return "complete";
    case CalibrationPhase::FAILED: return "failed";
    default: return "idle";
  }
}

uint8_t phaseProgress(CalibrationPhase value, uint32_t started, uint32_t now) {
  if (value == CalibrationPhase::COMPLETE) return 100;
  if (!calibrationRunning(value)) return 0;
  constexpr uint32_t total = 2 * PREPARE_MS + 2 * CALIBRATION_SAMPLE_MS;
  uint32_t base = 0;
  if (value == CalibrationPhase::SAMPLE_FINGER) base = PREPARE_MS;
  else if (value == CalibrationPhase::PREPARE_NO_FINGER) base = PREPARE_MS + CALIBRATION_SAMPLE_MS;
  else if (value == CalibrationPhase::SAMPLE_NO_FINGER) base = 2 * PREPARE_MS + CALIBRATION_SAMPLE_MS;
  return min<uint32_t>(99, ((base + now - started) * 100UL) / total);
}

int compareUint16(const void *a, const void *b) {
  const uint16_t left = *static_cast<const uint16_t *>(a);
  const uint16_t right = *static_cast<const uint16_t *>(b);
  return left < right ? -1 : left > right ? 1 : 0;
}

uint16_t medianCopy(const uint16_t *values, size_t count) {
  if (!count) return 0;
  memcpy(scratch, values, count * sizeof(uint16_t));
  qsort(scratch, count, sizeof(uint16_t), compareUint16);
  return count & 1 ? scratch[count / 2]
                   : static_cast<uint16_t>((static_cast<uint32_t>(scratch[count / 2 - 1]) + scratch[count / 2]) / 2);
}

uint16_t madOf(const uint16_t *values, size_t count, uint16_t median) {
  for (size_t i = 0; i < count; ++i) scratch[i] = values[i] > median ? values[i] - median : median - values[i];
  qsort(scratch, count, sizeof(uint16_t), compareUint16);
  return count & 1 ? scratch[count / 2]
                   : static_cast<uint16_t>((static_cast<uint32_t>(scratch[count / 2 - 1]) + scratch[count / 2]) / 2);
}

float medianIbi() {
  if (!ibiCount) return 0;
  uint16_t ordered[IBI_HISTORY];
  memcpy(ordered, ibis, ibiCount * sizeof(uint16_t));
  qsort(ordered, ibiCount, sizeof(uint16_t), compareUint16);
  return ordered[ibiCount / 2];
}

float robustMeanIbi() {
  if (!ibiCount) return 0;
  uint16_t ordered[IBI_HISTORY];
  memcpy(ordered, ibis, ibiCount * sizeof(uint16_t));
  qsort(ordered, ibiCount, sizeof(uint16_t), compareUint16);
  const size_t first = ibiCount >= 5 ? 1 : 0;
  const size_t last = ibiCount >= 5 ? ibiCount - 1 : ibiCount;
  uint32_t sum = 0;
  for (size_t i = first; i < last; ++i) sum += ordered[i];
  return static_cast<float>(sum) / (last - first);
}

float ibiVariation(float median) {
  if (!ibiCount || median <= 0) return 1;
  uint16_t deviation[IBI_HISTORY];
  for (size_t i = 0; i < ibiCount; ++i) deviation[i] = abs(static_cast<int>(ibis[i]) - static_cast<int>(median));
  qsort(deviation, ibiCount, sizeof(uint16_t), compareUint16);
  return deviation[ibiCount / 2] / median;
}

void resetDerivedAnalysis() {
  if (derivedCollecting || respirationCount || prvCount || respirationValid || stressValid) {
    memset(respirationFeatures, 0, sizeof(respirationFeatures));
    memset(respirationSignalQuality, 0, sizeof(respirationSignalQuality));
    memset(prvIntervals, 0, sizeof(prvIntervals));
    memset(prvTimes, 0, sizeof(prvTimes));
  }
  respirationIndex = respirationCount = 0;
  respirationDecimator = 0;
  latestPulseAmplitude = 0;
  latestAcceptedIbi = 0;
  respirationRate = respirationConfidence = 0;
  respirationValid = false;
  respirationSources = 0;
  respirationCandidate = 0;
  respirationCandidateStreak = 0;
  respirationLastSuccessMs = 0;
  prvIndex = prvCount = 0;
  stressScore = stressConfidence = 0;
  rmssdMs = baevskyStressIndex = 0;
  stressValid = false;
  stressUsesRespiration = false;
  stressLastSuccessMs = 0;
  stressState = "unavailable";
  lastDerivedUpdateMs = derivedStartedMs = 0;
  derivedCollecting = false;
}

void recordPrvInterval(uint16_t interval, uint32_t now) {
  latestAcceptedIbi = interval;
  if (!derivedCollecting) return;
  prvIntervals[prvIndex] = interval;
  prvTimes[prvIndex] = now;
  prvIndex = (prvIndex + 1) % PRV_HISTORY;
  if (prvCount < PRV_HISTORY) ++prvCount;
}

float derivedSignalEvidence() {
  if (!respirationCount) return 0;
  const size_t start =
      (respirationIndex + RESPIRATION_WINDOW - respirationCount) % RESPIRATION_WINDOW;
  float sum = 0;
  for (size_t i = 0; i < respirationCount; ++i) {
    sum += respirationSignalQuality[(start + i) % RESPIRATION_WINDOW];
  }
  return sum / respirationCount;
}

float medianSmall(float *values, size_t count) {
  for (size_t i = 1; i < count; ++i) {
    const float value = values[i];
    size_t position = i;
    while (position && values[position - 1] > value) {
      values[position] = values[position - 1];
      --position;
    }
    values[position] = value;
  }
  return values[count / 2];
}

bool respiratorySpectralPeak(uint8_t feature, float &rate, float &quality) {
  if (respirationCount < RESPIRATION_MIN_SAMPLES) return false;
  const size_t start =
      (respirationIndex + RESPIRATION_WINDOW - respirationCount) % RESPIRATION_WINDOW;
  float seriesMinimum = respirationFeatures[feature][start];
  float seriesMaximum = seriesMinimum;
  for (size_t i = 0; i < respirationCount; ++i) {
    const float value =
        respirationFeatures[feature][(start + i) % RESPIRATION_WINDOW];
    respirationClean[i] = value;
    seriesMinimum = min(seriesMinimum, value);
    seriesMaximum = max(seriesMaximum, value);
  }

  // Un Hampel corto elimina movimientos aislados sin borrar la oscilacion lenta
  // de la respiracion. La literatura de RR por PPG recomienda tratar estos
  // saltos antes de buscar la frecuencia dominante.
  const float outlierFloor = max(1e-4f, 0.015f * (seriesMaximum - seriesMinimum));
  for (size_t i = 0; i < respirationCount; ++i) {
    float neighborhood[7];
    float deviations[7];
    size_t neighborhoodCount = 0;
    const size_t first = i > 3 ? i - 3 : 0;
    const size_t last = min(respirationCount - 1, i + 3);
    for (size_t j = first; j <= last; ++j) {
      neighborhood[neighborhoodCount++] = respirationClean[j];
    }
    const float median = medianSmall(neighborhood, neighborhoodCount);
    for (size_t j = first, k = 0; j <= last; ++j, ++k) {
      deviations[k] = fabsf(respirationClean[j] - median);
    }
    const float mad = medianSmall(deviations, neighborhoodCount);
    const float threshold = max(outlierFloor, 4.4478f * mad);
    respirationWork[i] = fabsf(respirationClean[i] - median) > threshold
        ? median : respirationClean[i];
  }

  const float midpoint = 0.5f * (respirationCount - 1);
  float mean = 0;
  for (size_t i = 0; i < respirationCount; ++i) {
    mean += respirationWork[i];
  }
  mean /= respirationCount;
  float covariance = 0, timeVariance = 0;
  for (size_t i = 0; i < respirationCount; ++i) {
    const float centeredTime = i - midpoint;
    covariance += centeredTime * (respirationWork[i] - mean);
    timeVariance += centeredTime * centeredTime;
  }
  const float slope = timeVariance > 0 ? covariance / timeVariance : 0;
  float energy = 0;
  for (size_t i = 0; i < respirationCount; ++i) {
    const float window = 0.5f - 0.5f * cosf(2.0f * PI * i / (respirationCount - 1));
    respirationWork[i] =
        (respirationWork[i] - mean - slope * (i - midpoint)) * window;
    energy += respirationWork[i] * respirationWork[i];
  }
  if (energy < 1e-6f) return false;

  size_t bestIndex = 0;
  float bestPower = 0;
  for (size_t frequencyIndex = 0;
       frequencyIndex < RESPIRATION_FREQUENCIES; ++frequencyIndex) {
    const float rpm = RESPIRATION_MIN_RPM + frequencyIndex * RESPIRATION_STEP_RPM;
    const float step = 2.0f * PI * (rpm / 60.0f) / RESPIRATION_SAMPLE_HZ;
    const float cosineStep = cosf(step);
    const float sineStep = sinf(step);
    float cosine = 1.0f;
    float sine = 0.0f;
    float real = 0, imaginary = 0;
    for (size_t i = 0; i < respirationCount; ++i) {
      real += respirationWork[i] * cosine;
      imaginary -= respirationWork[i] * sine;
      const float nextCosine = cosine * cosineStep - sine * sineStep;
      sine = sine * cosineStep + cosine * sineStep;
      cosine = nextCosine;
      if ((i & 63U) == 63U) {
        const float norm = sqrtf(cosine * cosine + sine * sine);
        if (norm > 0) { cosine /= norm; sine /= norm; }
      }
    }
    const float power = real * real + imaginary * imaginary;
    respirationSpectrum[frequencyIndex] = power;
    if (power > bestPower) {
      bestPower = power;
      bestIndex = frequencyIndex;
    }
  }

  float secondPower = 0;
  for (size_t i = 0; i < RESPIRATION_FREQUENCIES; ++i) {
    if (abs(static_cast<int>(i) - static_cast<int>(bestIndex)) <= 4) continue;
    secondPower = max(secondPower, respirationSpectrum[i]);
  }
  const float prominence = bestPower / max(secondPower, 1e-9f);
  const float coherence = constrain(
      2.0f * bestPower / (respirationCount * energy), 0.0f, 1.0f);
  const float prominenceQuality =
      constrain((prominence - 1.10f) / 1.90f, 0.0f, 1.0f);
  const float coherenceQuality = constrain(coherence / 0.18f, 0.0f, 1.0f);
  const float collectionQuality =
      constrain(derivedSignalEvidence() / 65.0f, 0.35f, 1.0f);
  quality = 100.0f * sqrtf(prominenceQuality * coherenceQuality) *
            collectionQuality;
  rate = RESPIRATION_MIN_RPM + bestIndex * RESPIRATION_STEP_RPM;
  return prominence >= 1.12f && coherence >= 0.025f && quality >= 15.0f;
}

void rejectRespirationEstimate(uint32_t now) {
  respirationSources = 0;
  if (respirationLastSuccessMs &&
      now - respirationLastSuccessMs <= DERIVED_RESULT_GRACE_MS) {
    respirationConfidence = max(25.0f, respirationConfidence * 0.82f);
    respirationValid = true;
  } else {
    respirationConfidence = 0;
    respirationValid = false;
  }
}

void estimateRespiration(uint32_t now) {
  float rates[3] = {0}, qualities[3] = {0};
  bool usable[3] = {false};
  uint8_t usableCount = 0;
  for (uint8_t feature = 0; feature < 3; ++feature) {
    usable[feature] = respiratorySpectralPeak(feature, rates[feature], qualities[feature]);
    if (usable[feature]) ++usableCount;
  }
  if (!usableCount) { rejectRespirationEstimate(now); return; }

  int8_t pairA = -1, pairB = -1;
  float smallestDifference = 1000;
  for (uint8_t a = 0; a < 3; ++a) {
    if (!usable[a]) continue;
    for (uint8_t b = a + 1; b < 3; ++b) {
      if (!usable[b]) continue;
      const float difference = fabsf(rates[a] - rates[b]);
      if (difference < smallestDifference) {
        smallestDifference = difference;
        pairA = a;
        pairB = b;
      }
    }
  }
  float estimate = 0;
  float spectral = 0;
  float agreement = 0;
  bool consensus = pairA >= 0 && smallestDifference <= 4.0f;
  respirationSources = 0;
  if (consensus) {
    float weightedRate = rates[pairA] * qualities[pairA] +
                         rates[pairB] * qualities[pairB];
    float weight = qualities[pairA] + qualities[pairB];
    float minimumRate = min(rates[pairA], rates[pairB]);
    float maximumRate = max(rates[pairA], rates[pairB]);
    respirationSources = 2;
    for (uint8_t feature = 0; feature < 3; ++feature) {
      if (!usable[feature] || feature == pairA || feature == pairB) continue;
      const float pairMean = 0.5f * (rates[pairA] + rates[pairB]);
      if (fabsf(rates[feature] - pairMean) <= 4.0f) {
        weightedRate += rates[feature] * qualities[feature];
        weight += qualities[feature];
        minimumRate = min(minimumRate, rates[feature]);
        maximumRate = max(maximumRate, rates[feature]);
        ++respirationSources;
      }
    }
    estimate = weightedRate / max(weight, 1.0f);
    spectral = constrain(weight / respirationSources, 0.0f, 100.0f);
    agreement = constrain(
        100.0f * (1.0f - (maximumRate - minimumRate) / 8.0f), 0.0f, 100.0f);
  } else {
    uint8_t bestFeature = 0;
    for (uint8_t feature = 1; feature < 3; ++feature) {
      if (qualities[feature] > qualities[bestFeature]) bestFeature = feature;
    }
    if (!usable[bestFeature] || qualities[bestFeature] < 50.0f) {
      rejectRespirationEstimate(now);
      return;
    }
    estimate = rates[bestFeature];
    spectral = qualities[bestFeature];
    agreement = 55.0f;
    respirationSources = 1;
  }

  if (respirationCandidateStreak &&
      fabsf(estimate - respirationCandidate) <= 3.0f) {
    respirationCandidate = 0.65f * respirationCandidate + 0.35f * estimate;
    if (respirationCandidateStreak < 255) ++respirationCandidateStreak;
  } else {
    respirationCandidate = estimate;
    respirationCandidateStreak = 1;
  }
  if (!consensus && respirationCandidateStreak < 2) {
    rejectRespirationEstimate(now);
    return;
  }

  const float evidence = constrain(
      100.0f * (respirationCount - RESPIRATION_MIN_SAMPLES) /
          (RESPIRATION_WINDOW - RESPIRATION_MIN_SAMPLES),
      0.0f, 100.0f);
  const float signalEvidence = derivedSignalEvidence();
  respirationConfidence = 0.45f * spectral + 0.30f * agreement +
                          0.15f * evidence + 0.10f * signalEvidence;
  if (respirationConfidence < 35.0f) {
    rejectRespirationEstimate(now);
    return;
  }
  respirationRate = respirationValid
      ? 0.72f * respirationRate + 0.28f * respirationCandidate
      : respirationCandidate;
  respirationValid = true;
  respirationLastSuccessMs = now;
}

float baevskyScoreToPercent(float index) {
  if (index <= 50.0f) return constrain(index * 0.40f, 0.0f, 20.0f);
  if (index <= 150.0f) return 20.0f + (index - 50.0f) * 0.25f;
  if (index <= 300.0f) return 45.0f + (index - 150.0f) * (20.0f / 150.0f);
  if (index <= 500.0f) return 65.0f + (index - 300.0f) * 0.085f;
  if (index <= 900.0f) return 82.0f + (index - 500.0f) * 0.045f;
  return 100.0f;
}

float respirationLoadPercent(float rpm) {
  if (rpm <= 6.0f) return 0;
  if (rpm <= 12.0f) return (rpm - 6.0f) * (10.0f / 6.0f);
  if (rpm <= 20.0f) return 10.0f + (rpm - 12.0f) * 5.0f;
  if (rpm <= 30.0f) return 50.0f + (rpm - 20.0f) * 5.0f;
  return 100.0f;
}

float rmssdLoadPercent(float value) {
  // Menor RMSSD implica menor modulacion parasimpatica. Son zonas orientativas
  // para reposo y se combinan con Baevsky; no constituyen una escala clinica.
  if (value <= 12.0f) return 100.0f;
  if (value <= 25.0f) return 100.0f - (value - 12.0f) * (20.0f / 13.0f);
  if (value <= 45.0f) return 80.0f - (value - 25.0f) * 2.0f;
  if (value <= 80.0f) return 40.0f - (value - 45.0f) * (40.0f / 35.0f);
  return 0.0f;
}

void rejectStressEstimate(uint32_t now) {
  if (stressLastSuccessMs &&
      now - stressLastSuccessMs <= DERIVED_RESULT_GRACE_MS) {
    stressConfidence = max(25.0f, stressConfidence * 0.85f);
    stressValid = true;
  } else {
    stressConfidence = 0;
    stressValid = false;
    stressUsesRespiration = false;
    stressState = "unavailable";
  }
}

void estimateStress(uint32_t now) {
  if (!derivedStartedMs || now - derivedStartedMs < STRESS_MIN_WINDOW_MS ||
      prvCount < 18) {
    rejectStressEstimate(now);
    return;
  }

  uint16_t window[PRV_HISTORY];
  size_t count = 0;
  uint32_t oldest = now, newest = 0;
  const size_t start = (prvIndex + PRV_HISTORY - prvCount) % PRV_HISTORY;
  for (size_t i = 0; i < prvCount; ++i) {
    const size_t source = (start + i) % PRV_HISTORY;
    if (now - prvTimes[source] > STRESS_WINDOW_MS) continue;
    window[count++] = prvIntervals[source];
    oldest = min(oldest, prvTimes[source]);
    newest = max(newest, prvTimes[source]);
  }
  if (count < 18 || newest - oldest < 25000) {
    rejectStressEstimate(now);
    return;
  }

  uint16_t minimum = window[0], maximum = window[0];
  double squaredDifferences = 0;
  uint16_t bins[25] = {0};
  for (size_t i = 0; i < count; ++i) {
    minimum = min(minimum, window[i]);
    maximum = max(maximum, window[i]);
    const int bin = constrain((static_cast<int>(window[i]) - 300) / 50, 0, 24);
    ++bins[bin];
    if (i) {
      const float difference = static_cast<float>(window[i]) - window[i - 1];
      squaredDifferences += difference * difference;
    }
  }
  size_t modeBin = 0;
  for (size_t i = 1; i < 25; ++i) {
    if (bins[i] > bins[modeBin]) modeBin = i;
  }
  const float modeSeconds = (325.0f + 50.0f * modeBin) / 1000.0f;
  // El muestreo a 50 Hz cuantiza los intervalos en pasos de 20 ms. Un suelo
  // de 50 ms evita dividir por cero sin convertir esa cuantizacion en precision.
  const float rangeSeconds = max((maximum - minimum) / 1000.0f, 0.05f);
  if (modeSeconds <= 0) return;
  const float amplitudeOfMode = 100.0f * bins[modeBin] / count;
  baevskyStressIndex = amplitudeOfMode /
                       (2.0f * modeSeconds * rangeSeconds);
  rmssdMs = sqrtf(squaredDifferences / (count - 1));

  const float baevskyPercent = baevskyScoreToPercent(baevskyStressIndex);
  const float cardiacScore =
      0.75f * baevskyPercent + 0.25f * rmssdLoadPercent(rmssdMs);
  stressUsesRespiration = respirationValid;
  stressScore = stressUsesRespiration
      ? 0.85f * cardiacScore +
        0.15f * respirationLoadPercent(respirationRate)
      : cardiacScore;
  stressScore = constrain(stressScore, 0.0f, 100.0f);

  const float spanSeconds = (newest - oldest) / 1000.0f;
  const float durationEvidence = constrain(
      50.0f + 50.0f * (spanSeconds - 25.0f) / 35.0f, 50.0f, 100.0f);
  const float intervalEvidence = constrain(100.0f * count / 50.0f, 35.0f, 100.0f);
  const float signalEvidence = constrain(derivedSignalEvidence(), 25.0f, 100.0f);
  stressConfidence = 0.38f * durationEvidence +
                     0.32f * intervalEvidence +
                     0.30f * signalEvidence;
  if (stressUsesRespiration) {
    stressConfidence = 0.85f * stressConfidence +
                       0.15f * respirationConfidence;
  }
  stressValid = stressConfidence >= 35.0f;
  if (stressValid) stressLastSuccessMs = now;
  if (!stressValid) {
    stressState = "unavailable";
  } else if (stressScore < 20.0f) {
    stressState = "very_low";
  } else if (stressScore < 40.0f) {
    stressState = "low";
  } else if (stressScore < 60.0f) {
    stressState = "moderate";
  } else if (stressScore < 80.0f) {
    stressState = "high";
  } else {
    stressState = "very_high";
  }
}

void updateDerivedMetrics(uint32_t now) {
  // La respiracion guiada forma parte de la medicion: se mantienen activas las
  // ventanas de respiracion y estres durante todo el ejercicio. Solo los cambios
  // duros de contexto descartan el analisis.
  const bool eligible = calibrated && fingerPresent &&
                        !calibrationRunning(phase);
  if (!eligible) {
    resetDerivedAnalysis();
    return;
  }
  if (!derivedCollecting) {
    derivedCollecting = true;
    derivedStartedMs = now;
  }
  if (++respirationDecimator >= RESPIRATION_DOWNSAMPLE) {
    respirationDecimator = 0;
    respirationFeatures[0][respirationIndex] = dcValue;
    const size_t previous =
        (respirationIndex + RESPIRATION_WINDOW - 1) % RESPIRATION_WINDOW;
    respirationFeatures[1][respirationIndex] = latestPulseAmplitude > 0
        ? latestPulseAmplitude
        : respirationCount ? respirationFeatures[1][previous] : 0.0f;
    respirationFeatures[2][respirationIndex] = latestAcceptedIbi
        ? latestAcceptedIbi
        : respirationCount ? respirationFeatures[2][previous] : 800.0f;
    const float sampleQuality = bpmValid
        ? signalQuality : 0.65f * signalQuality;
    respirationSignalQuality[respirationIndex] = static_cast<uint8_t>(
        constrain(sampleQuality, 0.0f, 100.0f));
    respirationIndex = (respirationIndex + 1) % RESPIRATION_WINDOW;
    if (respirationCount < RESPIRATION_WINDOW) ++respirationCount;
  }
  if (!lastDerivedUpdateMs || now - lastDerivedUpdateMs >= DERIVED_UPDATE_MS) {
    lastDerivedUpdateMs = now;
    estimateRespiration(now);
    estimateStress(now);
  }
}

void resetPulseAnalysis() {
  memset(signalRing, 0, sizeof(signalRing));
  signalIndex = signalCount = 0;
  recentAmplitude = adaptiveThreshold = 0;
  signalPeakLevel = noisePeakLevel = 0;
  detectorPrevious2 = detectorPrevious1 = 0;
  detectorPrevious1Ms = 0;
  polarityLearningSamples = 0;
  positiveEnvelope = negativeEnvelope = 0;
  signalPolarity = 1;
  ppgValue = 0;
  peakThisSample = false;
  lastPeakMs = 0;
  lastReliablePulseMs = 0;
  memset(ibis, 0, sizeof(ibis));
  ibiCount = ibiIndex = 0;
  bpmCurrent = bpmAverage = bpmSum = 0;
  bpmAverageCount = 0;
  bpmValid = bpmAverageReady = false;
  signalQuality = 0;
  resetDerivedAnalysis();
}

void setFinger(bool present, uint32_t now) {
  if (fingerPresent == present) return;
  fingerPresent = present;
  fingerCandidate = present;
  fingerCandidateSinceMs = now;
  fingerPresentSinceMs = present ? now : 0;
  resetPulseAnalysis();
  if (!serialPlotterEnabled) Serial.println(present ? "FINGER_DETECTED" : "FINGER_REMOVED");
}

void beginCalibration(uint32_t now) {
  calibrated = false;
  phase = CalibrationPhase::PREPARE_FINGER;
  phaseStartedMs = now;
  phaseMessage = "Col·loca el dit quan acabi el compte enrere.";
  fingerSampleCount = noFingerSampleCount = 0;
  fingerMedian = noFingerMedian = fingerMad = noFingerMad = 0;
  fingerMin = noFingerMin = ADC_MAX_VALUE;
  fingerMax = noFingerMax = 0;
  missedAtCalibrationStart = missedSamples;
  setFinger(false, now);
  resetPulseAnalysis();
  if (!serialPlotterEnabled) Serial.println("CALIBRATION_STARTED");
}

void queueCalibrationLog(const char *result, uint16_t difference, uint16_t noise, uint32_t now) {
  portENTER_CRITICAL(&stateMux);
  pendingLog = {now, result, static_cast<uint16_t>(fingerSampleCount),
                static_cast<uint16_t>(noFingerSampleCount), fingerMedian,
                noFingerMedian, fingerMad, noFingerMad, fingerMin, fingerMax,
                noFingerMin, noFingerMax, difference, noise,
                missedSamples - missedAtCalibrationStart};
  pendingLogReady = true;
  portEXIT_CRITICAL(&stateMux);
}

void evaluateCalibration(uint32_t now) {
  const char *result = "insufficient_samples";
  uint16_t difference = 0;
  uint32_t noise = 0;
  bool valid = fingerSampleCount >= MIN_CALIBRATION_SAMPLES &&
               noFingerSampleCount >= MIN_CALIBRATION_SAMPLES;
  if (valid) {
    fingerMedian = medianCopy(fingerSamples, fingerSampleCount);
    noFingerMedian = medianCopy(noFingerSamples, noFingerSampleCount);
    fingerMad = madOf(fingerSamples, fingerSampleCount, fingerMedian);
    noFingerMad = madOf(noFingerSamples, noFingerSampleCount, noFingerMedian);
    difference = abs(static_cast<int>(fingerMedian) - static_cast<int>(noFingerMedian));
    noise = static_cast<uint32_t>(fingerMad) + noFingerMad;
    const bool saturated = fingerMedian < 30 || noFingerMedian < 30 ||
                           fingerMedian > ADC_MAX_VALUE - 30 || noFingerMedian > ADC_MAX_VALUE - 30;
    if (saturated) {
      valid = false;
      result = "saturated";
      phaseMessage = "Sensor saturat: revisa el divisor i utilitza només 3,3 V.";
    } else if (difference < 12 || difference < 5UL * noise) {
      valid = false;
      result = "low_contrast";
      phaseMessage = "Contrast insuficient: ajusta la llum, el dit o el divisor.";
    } else {
      result = "ok";
      phaseMessage = "Mapatge vàlid. Col·loca el dit per mesurar el pols.";
    }
  } else {
    phaseMessage = "Han faltat mostres; repeteix el calibratge.";
  }
  calibrated = valid;
  phase = valid ? CalibrationPhase::COMPLETE : CalibrationPhase::FAILED;
  phaseStartedMs = now;
  dcInitialized = false;
  resetPulseAnalysis();
  const uint16_t loggedNoise =
      noise > UINT16_MAX ? UINT16_MAX : static_cast<uint16_t>(noise);
  queueCalibrationLog(result, difference, loggedNoise, now);
  if (!serialPlotterEnabled) {
    Serial.printf("CALIBRATION_%s finger=%u no_finger=%u mad=%u,%u samples=%u,%u\n",
                  valid ? "OK" : "FAILED", fingerMedian, noFingerMedian,
                  fingerMad, noFingerMad, fingerSampleCount, noFingerSampleCount);
  }
}

void updateCalibration(uint32_t now) {
  const uint32_t elapsed = now - phaseStartedMs;
  switch (phase) {
    case CalibrationPhase::PREPARE_FINGER:
      if (elapsed >= PREPARE_MS) {
        phase = CalibrationPhase::SAMPLE_FINGER;
        phaseStartedMs = now;
        phaseMessage = "Mantén el dit immòbil durant 10 segons.";
      }
      break;
    case CalibrationPhase::SAMPLE_FINGER:
      if (elapsed >= CALIBRATION_SAMPLE_MS) {
        phase = CalibrationPhase::PREPARE_NO_FINGER;
        phaseStartedMs = now;
        phaseMessage = "Retira el dit quan acabi el compte enrere.";
      }
      break;
    case CalibrationPhase::PREPARE_NO_FINGER:
      if (elapsed >= PREPARE_MS) {
        phase = CalibrationPhase::SAMPLE_NO_FINGER;
        phaseStartedMs = now;
        phaseMessage = "Mantén el fotoresistor lliure durant 10 segons.";
      }
      break;
    case CalibrationPhase::SAMPLE_NO_FINGER:
      if (elapsed >= CALIBRATION_SAMPLE_MS) evaluateCalibration(now);
      break;
    default: break;
  }
}

void updateFinger(uint32_t now) {
  if (!calibrated || calibrationRunning(phase)) return;
  const int32_t denominator = static_cast<int32_t>(fingerMedian) - noFingerMedian;
  if (!denominator) return;
  const float presence = (dcValue - noFingerMedian) / denominator;
  bool desired = fingerPresent;
  bool triggered = false;
  if (!fingerPresent && presence > 0.62f) { desired = true; triggered = true; }
  else if (fingerPresent && presence < 0.38f) { desired = false; triggered = true; }
  if (!triggered) {
    fingerCandidate = fingerPresent;
    fingerCandidateSinceMs = now;
  } else if (fingerCandidate != desired) {
    fingerCandidate = desired;
    fingerCandidateSinceMs = now;
  } else if (now - fingerCandidateSinceMs >= (desired ? 350UL : 1200UL)) {
    setFinger(desired, now);
  }
}

float minimumUsefulAmplitude() {
  // La MAD con dedo contiene parte del propio pulso; la MAD sin dedo estima
  // mejor el suelo de ruido optico/electrico del montaje.
  const float calibratedNoise = max(1.0f, static_cast<float>(noFingerMad));
  return max(MIN_SIGNAL_AMPLITUDE_ADC,
             calibratedNoise * CALIBRATION_NOISE_MULTIPLIER);
}

const char *qualityStateName() {
  if (!fingerPresent) return "no_finger";
  if (recentAmplitude < minimumUsefulAmplitude()) return "weak_signal";
  if (!bpmValid) return "stabilizing";
  return "valid";
}

void updateSignalQuality() {
  if (!fingerPresent) {
    signalQuality = 0;
    return;
  }
  const float minimum = minimumUsefulAmplitude();
  const float amplitudeScore = constrain(50.0f * recentAmplitude / minimum, 0.0f, 100.0f);
  const float regularityScore = ibiCount >= 3
      ? constrain(100.0f * (1.0f - 4.0f * ibiVariation(medianIbi())), 0.0f, 100.0f)
      : 25.0f * ibiCount;
  const float evidenceScore = constrain(25.0f * ibiCount, 0.0f, 100.0f);
  signalQuality = 0.45f * amplitudeScore + 0.40f * regularityScore +
                  0.15f * evidenceScore;
}

bool acceptPeak(uint32_t peakMs, float peakAmplitude) {
  if (!lastPeakMs) {
    lastPeakMs = peakMs;
    latestPulseAmplitude = peakAmplitude;
    ++pulseAcceptedCount;
    return true;
  }

  const uint32_t rawInterval = peakMs - lastPeakMs;
  if (rawInterval < REFRACTORY_MS) {
    ++pulseRejectedCount;
    return false;
  }
  if (rawInterval > MAX_PEAK_GAP_MS) {
    // Tras un hueco largo, este pico se convierte en el nuevo ancla. Se conserva
    // el BPM mostrado durante su margen, pero no se inventa un intervalo.
    lastPeakMs = peakMs;
    latestPulseAmplitude = peakAmplitude;
    ++pulseRejectedCount;
    return true;
  }

  uint32_t interval = rawInterval;
  uint8_t recoveredBeats = 0;
  if (ibiCount >= 3) {
    const float previousMedian = medianIbi();
    const float lowerBound = previousMedian * (1.0f - IBI_OUTLIER_FRACTION);
    const float upperBound = previousMedian * (1.0f + IBI_OUTLIER_FRACTION);
    if (rawInterval < lowerBound) {
      // Pico precoz: suele ser ruido o la incisura dicrótica. No cambia el ancla.
      noisePeakLevel = 0.875f * noisePeakLevel + 0.125f * peakAmplitude;
      ++pulseRejectedCount;
      if (!serialPlotterEnabled) {
        Serial.printf("PULSE_REJECTED ibi=%lu expected=%.0f reason=early\n",
                      static_cast<unsigned long>(rawInterval), previousMedian);
      }
      return false;
    }
    if (rawInterval > upperBound) {
      const uint8_t periods = constrain(
          static_cast<int>(lroundf(rawInterval / previousMedian)), 2, 3);
      const float corrected = static_cast<float>(rawInterval) / periods;
      if (fabsf(corrected - previousMedian) <= previousMedian * 0.20f) {
        interval = lroundf(corrected);
        recoveredBeats = periods - 1;
      } else {
        // El pico parece real pero el intervalo no es fiable. Reanclar aquí evita
        // que un solo rechazo convierta también el latido siguiente en un hueco.
        lastPeakMs = peakMs;
        latestPulseAmplitude = peakAmplitude;
        ++pulseRejectedCount;
        if (!serialPlotterEnabled) {
          Serial.printf("PULSE_REJECTED ibi=%lu expected=%.0f reason=resync\n",
                        static_cast<unsigned long>(rawInterval), previousMedian);
        }
        return true;
      }
    }
  }
  if (interval < REFRACTORY_MS || interval > MAX_IBI_MS) {
    lastPeakMs = peakMs;
    latestPulseAmplitude = peakAmplitude;
    ++pulseRejectedCount;
    return true;
  }

  lastPeakMs = peakMs;
  latestPulseAmplitude = peakAmplitude;
  ++pulseAcceptedCount;
  pulseRecoveredCount += recoveredBeats;
  ibis[ibiIndex] = interval;
  ibiIndex = (ibiIndex + 1) % IBI_HISTORY;
  if (ibiCount < IBI_HISTORY) ++ibiCount;
  recordPrvInterval(interval, peakMs);

  // La media recortada conserva la robustez de la mediana y reduce los saltos
  // de cuantizacion de 20 ms propios del muestreo a 50 Hz.
  const float targetBpm = 60000.0f / robustMeanIbi();
  if (bpmCurrent > 0 && lastReliablePulseMs &&
      peakMs - lastReliablePulseMs <= BPM_HOLD_MS) {
    const float maximumStep = max(3.0f, bpmCurrent * 0.08f);
    const float boundedTarget = constrain(
        targetBpm, bpmCurrent - maximumStep, bpmCurrent + maximumStep);
    bpmCurrent = 0.78f * bpmCurrent + 0.22f * boundedTarget;
  } else {
    bpmCurrent = targetBpm;
  }
  lastReliablePulseMs = peakMs;
  bpmValid = ibiCount >= MIN_INTERVALS_FOR_BPM &&
             peakMs - fingerPresentSinceMs >= 4000 &&
             recentAmplitude >= minimumUsefulAmplitude();

  const float instantBpm = 60000.0f / interval;
  if (ibiCount >= 3 && fabsf(instantBpm - targetBpm) <= targetBpm * 0.25f) {
    bpmSum += instantBpm;
    ++bpmAverageCount;
    bpmAverage = bpmSum / bpmAverageCount;
  }
  bpmAverageReady = bpmAverageCount >= MIN_INTERVALS_FOR_AVERAGE &&
                    peakMs - fingerPresentSinceMs >= 10000;
  if (!serialPlotterEnabled) {
    Serial.printf("PULSE_OK ibi=%lu bpm=%.1f target=%.1f recovered=%u count=%u\n",
                  static_cast<unsigned long>(interval), bpmCurrent, targetBpm,
                  recoveredBeats, bpmAverageCount);
  }
  return true;
}

void updatePulse(float filtered, uint32_t now) {
  peakThisSample = false;
  if (!fingerPresent || calibrationRunning(phase)) {
    ppgValue = filtered;
    adaptiveThreshold = 0;
    updateSignalQuality();
    return;
  }

  // Aprende una unica orientacion para que el pulso dominante quede positivo.
  // Solo invierte la PPG real; no sintetiza ni modifica su morfologia.
  if (polarityLearningSamples < SAMPLE_RATE_HZ * 2) {
    positiveEnvelope = max(positiveEnvelope * 0.995f, filtered);
    negativeEnvelope = max(negativeEnvelope * 0.995f, -filtered);
    ++polarityLearningSamples;
    if (polarityLearningSamples == SAMPLE_RATE_HZ * 2) {
      signalPolarity = positiveEnvelope >= negativeEnvelope ? 1 : -1;
    }
  }
  ppgValue = filtered * signalPolarity;

  signalRing[signalIndex] = ppgValue;
  signalIndex = (signalIndex + 1) % SIGNAL_WINDOW;
  if (signalCount < SIGNAL_WINDOW) ++signalCount;
  float windowMinimum = signalRing[0], windowMaximum = signalRing[0];
  for (size_t i = 1; i < signalCount; ++i) {
    windowMinimum = min(windowMinimum, signalRing[i]);
    windowMaximum = max(windowMaximum, signalRing[i]);
  }
  recentAmplitude = windowMaximum - windowMinimum;

  const float minimum = minimumUsefulAmplitude();
  if (signalPeakLevel <= 0) signalPeakLevel = max(minimum, 0.65f * recentAmplitude);
  signalPeakLevel *= 0.999f;  // Permite seguir una amplitud que disminuye lentamente.
  noisePeakLevel *= 0.999f;
  adaptiveThreshold = max(0.18f * minimum,
      noisePeakLevel + ADAPTIVE_THRESHOLD_FRACTION *
      max(0.0f, signalPeakLevel - noisePeakLevel));

  const bool localMaximum = detectorPrevious1 > detectorPrevious2 &&
                            detectorPrevious1 >= ppgValue;
  if (localMaximum) {
    const bool settled = now - fingerPresentSinceMs >= DETECTOR_SETTLE_MS;
    const bool enoughAmplitude = signalCount == SIGNAL_WINDOW &&
                                 recentAmplitude >= minimum;
    if (settled && enoughAmplitude && detectorPrevious1 > adaptiveThreshold) {
      if (acceptPeak(detectorPrevious1Ms, detectorPrevious1)) {
        peakThisSample = true;
        signalPeakLevel = 0.875f * signalPeakLevel +
                          0.125f * detectorPrevious1;
      }
    } else if (detectorPrevious1 > 0) {
      noisePeakLevel = 0.95f * noisePeakLevel + 0.05f * detectorPrevious1;
    }
  }
  detectorPrevious2 = detectorPrevious1;
  detectorPrevious1 = ppgValue;
  detectorPrevious1Ms = now;

  if (lastReliablePulseMs && now - lastReliablePulseMs > BPM_HOLD_MS) {
    bpmValid = false;
  }
  if (lastPeakMs && now - lastPeakMs > MAX_PEAK_GAP_MS) {
    lastPeakMs = 0;
  }
  updateSignalQuality();
}

uint16_t readAveragedAdc() {
  uint32_t sum = 0;
  uint16_t smallest = ADC_MAX_VALUE;
  uint16_t largest = 0;
  for (uint8_t i = 0; i < ADC_OVERSAMPLES; ++i) {
    const uint16_t sample = analogRead(SENSOR_PIN);
    sum += sample;
    smallest = min(smallest, sample);
    largest = max(largest, sample);
  }
  return static_cast<uint16_t>((sum - smallest - largest) /
                               (ADC_OVERSAMPLES - 2));
}

void publishState() {
  portENTER_CRITICAL(&stateMux);
  publicState.raw = rawValue;
  publicState.filtered = ppgValue;
  publicState.calibrated = calibrated;
  publicState.finger = fingerPresent;
  publicState.phase = phase;
  publicState.phaseStartedMs = phaseStartedMs;
  publicState.calibrationMessage = phaseMessage;
  publicState.bpmCurrent = bpmCurrent;
  publicState.bpmAverage = bpmAverage;
  publicState.bpmValid = bpmValid;
  publicState.bpmAverageReady = bpmAverageReady;
  publicState.beatCount = bpmAverageCount;
  publicState.pulseAccepted = pulseAcceptedCount;
  publicState.pulseRejected = pulseRejectedCount;
  publicState.pulseRecovered = pulseRecoveredCount;
  publicState.quality = signalQuality;
  publicState.qualityState = qualityStateName();
  publicState.respirationRate = respirationRate;
  publicState.respirationConfidence = respirationConfidence;
  publicState.respirationWindowSeconds =
      static_cast<float>(respirationCount) / RESPIRATION_SAMPLE_HZ;
  publicState.respirationValid = respirationValid;
  publicState.respirationSources = respirationSources;
  publicState.stressScore = stressScore;
  publicState.stressConfidence = stressConfidence;
  publicState.stressWindowSeconds = derivedStartedMs
      ? min(static_cast<float>(millis() - derivedStartedMs) / 1000.0f, 60.0f)
      : 0;
  publicState.rmssdMs = rmssdMs;
  publicState.baevskyStressIndex = baevskyStressIndex;
  publicState.stressValid = stressValid;
  publicState.stressUsesRespiration = stressUsesRespiration;
  publicState.stressState = stressState;
  publicState.threshold = adaptiveThreshold;
  publicState.peak = peakThisSample;
  publicState.samples = totalSamples;
  publicState.missed = missedSamples;
  graphRing[graphIndex] = ppgValue;
  graphIndex = (graphIndex + 1) % GRAPH_HISTORY;
  if (graphCount < GRAPH_HISTORY) ++graphCount;
  portEXIT_CRITICAL(&stateMux);
}

void samplingTask(void *) {
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    const uint64_t nowUs = esp_timer_get_time();
    if (lastSampleUs) {
      const uint64_t elapsed = nowUs - lastSampleUs;
      if (elapsed > 30000) missedSamples += elapsed / 20000 - 1;
    }
    lastSampleUs = nowUs;
    const uint32_t now = millis();
    if (calibrationStartRequested && !calibrationRunning(phase)) {
      calibrationStartRequested = false;
      beginCalibration(now);
    }
    updateCalibration(now);
    if (postAnalysisPulseResetRequested) {
      postAnalysisPulseResetRequested = false;
      resetPulseAnalysis();
      if (!serialPlotterEnabled) Serial.println("POST_ANALYSIS_STARTED");
    }

    rawValue = readAveragedAdc();
    if (!dcInitialized) { dcValue = rawValue; dcInitialized = true; }
    else dcValue += 0.06f * (rawValue - dcValue);
    if (phase == CalibrationPhase::SAMPLE_FINGER && fingerSampleCount < CALIBRATION_CAPACITY) {
      fingerSamples[fingerSampleCount++] = rawValue;
      fingerMin = min(fingerMin, rawValue); fingerMax = max(fingerMax, rawValue);
    } else if (phase == CalibrationPhase::SAMPLE_NO_FINGER && noFingerSampleCount < CALIBRATION_CAPACITY) {
      noFingerSamples[noFingerSampleCount++] = rawValue;
      noFingerMin = min(noFingerMin, rawValue); noFingerMax = max(noFingerMax, rawValue);
    }
    filteredValue = lowPass.process(highPass.process(rawValue));
    updateFinger(now);
    updatePulse(filteredValue, now);
    updateDerivedMetrics(now);
    ++totalSamples;
    publishState();
    if (serialPlotterEnabled) {
      const float marker = peakThisSample ? max(adaptiveThreshold * 1.5f, ppgValue) : 0;
      Serial.printf("PPG:%.2f\tThreshold:%.2f\tPeak:%.2f\n",
                    ppgValue, adaptiveThreshold, marker);
    }
  }
}

void ensureLogFile() {
  if (!filesystemReady || LittleFS.exists(LOG_PATH)) return;
  File file = LittleFS.open(LOG_PATH, FILE_WRITE);
  if (!file) return;
  file.println("# calibration.log - Keyestudio ESP32 PLUS / GPIO34");
  file.println("# uptime_ms;result;finger_count;no_finger_count;finger_median;no_finger_median;finger_mad;no_finger_mad;finger_min;finger_max;no_finger_min;no_finger_max;difference;robust_noise;missed");
  file.close();
}

void flushPendingLog() {
  CalibrationLogEntry entry;
  bool ready = false;
  portENTER_CRITICAL(&stateMux);
  if (pendingLogReady) { entry = pendingLog; pendingLogReady = false; ready = true; }
  portEXIT_CRITICAL(&stateMux);
  if (!ready || !filesystemReady) return;
  if (LittleFS.exists(LOG_PATH)) {
    File current = LittleFS.open(LOG_PATH, FILE_READ);
    const size_t size = current ? current.size() : 0;
    if (current) current.close();
    if (size > MAX_LOG_BYTES) {
      LittleFS.remove(LOG_OLD_PATH);
      LittleFS.rename(LOG_PATH, LOG_OLD_PATH);
    }
  }
  ensureLogFile();
  File file = LittleFS.open(LOG_PATH, FILE_APPEND);
  if (!file) return;
  file.printf("%lu;%s;%u;%u;%u;%u;%u;%u;%u;%u;%u;%u;%u;%u;%lu\n",
              static_cast<unsigned long>(entry.uptimeMs), entry.result,
              entry.fingerCount, entry.noFingerCount, entry.fingerMedian,
              entry.noFingerMedian, entry.fingerMad, entry.noFingerMad,
              entry.fingerMin, entry.fingerMax, entry.noFingerMin,
              entry.noFingerMax, entry.difference, entry.robustNoise,
              static_cast<unsigned long>(entry.missed));
  file.close();
  if (!serialPlotterEnabled) Serial.printf("LOG_SAVED result=%s\n", entry.result);
}

bool breathingBaselineReady(const PublicState &state) {
  return state.calibrated && state.finger && !calibrationRunning(state.phase) &&
         state.bpmValid && state.respirationValid && state.stressValid;
}

const char *breathingStatus(const PublicState &state) {
  if (breathingRunning || breathingStartRequested) return "running";
  if (postAnalysisRunning) return "analyzing";
  if (!state.calibrated || calibrationRunning(state.phase)) return "needs_calibration";
  if (!state.finger) return "needs_finger";
  if (!state.bpmValid) return "needs_bpm";
  if (!state.respirationValid) return "needs_respiration";
  if (!state.stressValid) return "needs_stress";
  return "ready";
}

void updatePostBreathingAnalysis(uint32_t now) {
  if (!postAnalysisRunning || now - postAnalysisStartedMs < POST_ANALYSIS_MIN_MS)
    return;
  PublicState state;
  portENTER_CRITICAL(&stateMux);
  state = publicState;
  portEXIT_CRITICAL(&stateMux);
  if (!breathingBaselineReady(state)) return;

  afterBpm = state.bpmCurrent;
  afterRespiration = state.respirationRate;
  afterStress = state.stressScore;
  comparisonReady = true;
  postAnalysisRunning = false;
  if (!serialPlotterEnabled) {
    Serial.printf(
        "BREATHING_COMPARISON before_bpm=%.1f during_bpm=%.1f after_bpm=%.1f "
        "before_rpm=%.1f during_rpm=%.1f after_rpm=%.1f "
        "before_stress=%.1f during_stress=%.1f after_stress=%.1f samples=%u\n",
        beforeBpm, duringBpm, afterBpm, beforeRespiration, duringRespiration,
        afterRespiration, beforeStress, duringStress, afterStress,
        duringSampleCount);
  }
}

void handleState() {
  const uint32_t now = millis();
  collectBreathingMeasurement(now);
  updateBreathingPattern(now);
  updatePostBreathingAnalysis(now);
  PublicState state;
  float signal[GRAPH_HISTORY];
  size_t count, index, sendCount;
  const uint32_t since = server.hasArg("since")
                             ? strtoul(server.arg("since").c_str(), nullptr, 10)
                             : 0;
  portENTER_CRITICAL(&stateMux);
  state = publicState;
  count = graphCount;
  index = graphIndex;
  sendCount = count;
  if (since && state.samples >= since) {
    sendCount = min<size_t>(count, state.samples - since);
  }
  for (size_t i = 0; i < sendCount; ++i) {
    signal[i] = graphRing[(index + GRAPH_HISTORY - sendCount + i) % GRAPH_HISTORY];
  }
  portEXIT_CRITICAL(&stateMux);
  String json;
  json.reserve(5000);
  json = "{\"raw\":" + String(state.raw) + ",\"signal\":[";
  for (size_t i = 0; i < sendCount; ++i) { if (i) json += ','; json += String(signal[i], 2); }
  json += "],\"seq\":" + String(state.samples);
  json += ",\"calibrated\":" + String(state.calibrated ? "true" : "false");
  json += ",\"finger\":" + String(state.finger ? "true" : "false");
  json += ",\"calibrating\":" + String(calibrationRunning(state.phase) ? "true" : "false");
  json += ",\"phase\":\"" + String(phaseName(state.phase)) + "\"";
  json += ",\"progress\":" + String(phaseProgress(state.phase, state.phaseStartedMs, millis()));
  json += ",\"cal_message\":\"" + String(state.calibrationMessage) + "\"";
  json += ",\"bpm\":" + String(state.bpmCurrent, 1);
  json += ",\"bpm_valid\":" + String(state.bpmValid ? "true" : "false");
  json += ",\"bpm_average\":" + String(state.bpmAverage, 1);
  json += ",\"bpm_average_ready\":" + String(state.bpmAverageReady ? "true" : "false");
  json += ",\"beats\":" + String(state.beatCount);
  json += ",\"pulse_accepted\":" + String(state.pulseAccepted);
  json += ",\"pulse_rejected\":" + String(state.pulseRejected);
  json += ",\"pulse_recovered\":" + String(state.pulseRecovered);
  json += ",\"quality\":" + String(state.quality, 1);
  json += ",\"quality_state\":\"" + String(state.qualityState) + "\"";
  json += ",\"respiration_rate\":" + String(state.respirationRate, 1);
  json += ",\"respiration_valid\":" + String(state.respirationValid ? "true" : "false");
  json += ",\"respiration_sources\":" + String(state.respirationSources);
  json += ",\"respiration_confidence\":" + String(state.respirationConfidence, 1);
  json += ",\"respiration_window_s\":" + String(state.respirationWindowSeconds, 1);
  json += ",\"stress_score\":" + String(state.stressScore, 1);
  json += ",\"stress_valid\":" + String(state.stressValid ? "true" : "false");
  json += ",\"stress_uses_respiration\":" +
          String(state.stressUsesRespiration ? "true" : "false");
  json += ",\"stress_confidence\":" + String(state.stressConfidence, 1);
  json += ",\"stress_state\":\"" + String(state.stressState) + "\"";
  json += ",\"stress_window_s\":" + String(state.stressWindowSeconds, 1);
  json += ",\"rmssd_ms\":" + String(state.rmssdMs, 1);
  json += ",\"baevsky_si\":" + String(state.baevskyStressIndex, 1);
  json += ",\"threshold\":" + String(state.threshold, 2);
  json += ",\"peak\":" + String(state.peak ? "true" : "false");
  json += ",\"missed\":" + String(state.missed);
  json += ",\"simulated\":false";
  json += ",\"firmware\":\"" + String(FIRMWARE_VERSION) + "\"";
  json += ",\"breathing_running\":" + String(breathingRunning ? "true" : "false");
  json += ",\"breathing_elapsed_ms\":" + String(breathingRunning ? now - breathingStartedMs : 0);
  json += ",\"breathing_phase\":" + String(breathingPhase);
  json += ",\"breathing_phase_elapsed_ms\":" + String(breathingRunning ? now - breathingPhaseStartedMs : 0);
  json += ",\"breathing_ready\":" +
          String(breathingBaselineReady(state) && !breathingRunning &&
                         !breathingStartRequested && !postAnalysisRunning
                     ? "true"
                     : "false");
  json += ",\"breathing_status\":\"" + String(breathingStatus(state)) + "\"";
  json += ",\"breathing_cycle\":" +
          String(breathingRunning
                     ? min<uint8_t>(BREATHING_CYCLES,
                                    (now - breathingStartedMs) /
                                            BREATHING_CYCLE_MS +
                                        1)
                     : 0);
  json += ",\"breathing_cycles\":" + String(BREATHING_CYCLES);
  json += ",\"breathing_total_ms\":" + String(BREATHING_TOTAL_MS);
  json += ",\"post_analysis_running\":" +
          String(postAnalysisRunning ? "true" : "false");
  json += ",\"post_analysis_elapsed_ms\":" +
          String(postAnalysisRunning ? now - postAnalysisStartedMs : 0);
  json += ",\"post_analysis_target_ms\":" + String(POST_ANALYSIS_MIN_MS);
  json += ",\"comparison_ready\":" + String(comparisonReady ? "true" : "false");
  json += ",\"before_bpm\":" + String(beforeBpm, 1);
  json += ",\"before_respiration\":" + String(beforeRespiration, 1);
  json += ",\"before_stress\":" + String(beforeStress, 1);
  json += ",\"during_measurement_ready\":" +
          String(duringMeasurementReady ? "true" : "false");
  json += ",\"during_samples\":" + String(duringSampleCount);
  json += ",\"during_bpm\":" + String(duringBpm, 1);
  json += ",\"during_respiration\":" + String(duringRespiration, 1);
  json += ",\"during_stress\":" + String(duringStress, 1);
  json += ",\"after_bpm\":" + String(afterBpm, 1);
  json += ",\"after_respiration\":" + String(afterRespiration, 1);
  json += ",\"after_stress\":" + String(afterStress, 1);
  json += ",\"motor_on\":" + String(motorOn ? "true" : "false");
  json += ",\"ip\":\"" + WiFi.softAPIP().toString() + "\"}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

void handleCalibrate() {
  PublicState state;
  portENTER_CRITICAL(&stateMux);
  state = publicState;
  const bool accepted = !calibrationRunning(state.phase) &&
                        !calibrationStartRequested && !breathingRunning &&
                        !breathingStartRequested && !postAnalysisRunning;
  if (accepted) calibrationStartRequested = true;
  portEXIT_CRITICAL(&stateMux);
  server.send(accepted ? 202 : 409, "application/json",
              accepted ? "{\"accepted\":true}" : "{\"accepted\":false}");
}

void handleBreathing() {
  bool accepted = false;
  bool busy = false;
  PublicState state;
  portENTER_CRITICAL(&stateMux);
  state = publicState;
  busy = breathingRunning || breathingStartRequested || postAnalysisRunning;
  if (!busy && breathingBaselineReady(state)) {
    beforeBpm = state.bpmCurrent;
    beforeRespiration = state.respirationRate;
    beforeStress = state.stressScore;
    afterBpm = afterRespiration = afterStress = 0;
    comparisonReady = false;
    breathingStartRequested = true;
    accepted = true;
  }
  portEXIT_CRITICAL(&stateMux);
  const int status = accepted ? 202 : busy ? 409 : 422;
  server.send(status, "application/json",
              accepted
                  ? "{\"accepted\":true}"
                  : busy ? "{\"accepted\":false,\"reason\":\"busy\"}"
                         : "{\"accepted\":false,\"reason\":\"baseline\"}");
}

void handleLogDownload() {
  if (!filesystemReady || !LittleFS.exists(LOG_PATH)) {
    server.send(404, "text/plain", "No calibration log yet");
    return;
  }
  File file = LittleFS.open(LOG_PATH, FILE_READ);
  server.sendHeader("Content-Disposition", "attachment; filename=\"calibration.log\"");
  server.streamFile(file, "text/plain");
  file.close();
}

void serviceSerial() {
  while (Serial.available()) {
    const char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      serialCommand[serialCommandLength] = 0;
      if (!strcmp(serialCommand, "LOGS")) {
        if (filesystemReady && LittleFS.exists(LOG_PATH)) {
          File file = LittleFS.open(LOG_PATH, FILE_READ);
          while (file && file.available()) Serial.write(file.read());
          if (file) file.close();
        } else Serial.println("NO_LOG_FILE");
        Serial.println("LOG_DUMP_END");
      } else if (!strcmp(serialCommand, "CALIBRATE")) {
        if (!calibrationRunning(phase)) {
          calibrationStartRequested = true;
          Serial.println("CALIBRATION_REQUESTED");
        } else Serial.println("CALIBRATION_BUSY");
      } else if (!strcmp(serialCommand, "PLOT ON") || !strcmp(serialCommand, "PLOT_ON")) {
        serialPlotterEnabled = true;
        if (preferencesReady) preferences.putBool("plotter", true);
      } else if (!strcmp(serialCommand, "PLOT OFF") || !strcmp(serialCommand, "PLOT_OFF")) {
        serialPlotterEnabled = false;
        if (preferencesReady) preferences.putBool("plotter", false);
        Serial.println("PLOTTER_DISABLED");
      } else if (!strcmp(serialCommand, "HELP")) {
        Serial.println("COMMANDS CALIBRATE LOGS PLOT_ON PLOT_OFF HELP");
      }
      serialCommandLength = 0;
    } else if (serialCommandLength + 1 < sizeof(serialCommand)) {
      serialCommand[serialCommandLength++] = toupper(c);
    }
  }
}

// ====================== PANTALLA OLED ======================
// Tres estados derivados exclusivamente de la telemetria real. El trazado
// inferior es un icono animado, NO un ECG ni la senal PPG adquirida.
uint8_t oledMode(const PublicState &s) {
  if (!s.calibrated || calibrationRunning(s.phase)) return 0;
  if (!s.finger || !s.bpmValid || !isfinite(s.bpmCurrent) ||
      s.bpmCurrent <= 0) return 1;
  return 2;
}

void oledHeart(int cx, int cy, int r) {
  oled.fillCircle(cx - r, cy - r / 2, r, SSD1306_WHITE);
  oled.fillCircle(cx + r, cy - r / 2, r, SSD1306_WHITE);
  oled.fillTriangle(cx - 2 * r, cy, cx + 2 * r, cy,
                    cx, cy + 2 * r, SSD1306_WHITE);
}

void oledFinger(uint32_t now) {
  const int bob = static_cast<int>(sinf(now * 0.004f));
  const int y = 2 + bob;
  // Indice extendido, una, nudillos y palma; toque sobre la base del sensor.
  oled.drawRoundRect(108, y, 8, 18, 3, SSD1306_WHITE);
  oled.drawRoundRect(110, y + 2, 4, 5, 1, SSD1306_WHITE);
  oled.fillRoundRect(103, y + 13, 20, 11, 3, SSD1306_WHITE);
  oled.drawRoundRect(102, 28, 22, 4, 1, SSD1306_WHITE);
  if ((now / 350) % 2) {
    oled.drawLine(98, 22, 101, 25, SSD1306_WHITE);
    oled.drawLine(123, 25, 126, 22, SSD1306_WHITE);
  }
}

void oledLargeMessage(const char *message, uint32_t elapsed) {
  // Letras 10x14 (celda 12x16) en una zona de 96 px, dejando el icono libre.
  // Pausas al principio/final para poder leer la frase completa sin abreviarla.
  const int width = strlen(message) * 12;
  const int travel = max(0, width - 96);
  const uint32_t scrollMs = travel * 35;
  const uint32_t t = elapsed % (3600 + scrollMs);
  const int offset = t < 1800 ? 0 : min(travel, static_cast<int>((t - 1800) / 35));
  oled.setTextSize(2);
  oled.setCursor(-offset, 8);
  oled.print(message);
  oled.fillRect(96, 0, 32, 32, SSD1306_BLACK);
}

float oledPulseShape(float p) {
  // Trazo estilizado para el icono: pico grande y pequeno valle.
  if (p < 0.12f) return -p / 0.12f * 3;
  if (p < 0.24f) return -3 + (p - 0.12f) / 0.12f * 15;
  if (p < 0.38f) return 12 - (p - 0.24f) / 0.14f * 18;
  if (p < 0.53f) return -6 + (p - 0.38f) / 0.15f * 6;
  return 0;
}

void renderOled(const PublicState &s, uint32_t now) {
  oled.clearDisplay();
  oled.setTextWrap(false);
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  const uint8_t mode = oledMode(s);
  static uint8_t previousMode = 255;
  static CalibrationPhase previousPhase = CalibrationPhase::IDLE;
  static uint32_t messageStarted = 0;
  if (previousMode != mode || previousPhase != s.phase) {
    messageStarted = now;
    previousMode = mode;
    previousPhase = s.phase;
  }
  if (mode == 0) {
    if (calibrationRunning(s.phase)) {
      const bool remove = s.phase == CalibrationPhase::PREPARE_NO_FINGER ||
                          s.phase == CalibrationPhase::SAMPLE_NO_FINGER;
      oled.setTextSize(2);
      oled.setCursor(0, 0); oled.print(remove ? "Remove" : "Place");
      oled.setCursor(0, 16); oled.print("finger");
    } else {
      oledLargeMessage("Make the calibration first", now - messageStarted);
    }
    oledFinger(now);
  } else if (mode == 1) {
    oled.setTextSize(2);
    oled.setCursor(0, 0); oled.print("No BPM");
    oled.setCursor(0, 16); oled.print("detected");
    // Movimiento lento de reposo; no pretende indicar un latido detectado.
    const int bob = static_cast<int>(sinf(now * TWO_PI / 4000.0f));
    oledHeart(112, 11 + bob, 6);
    oled.drawFastHLine(105, 10 + bob, 4, SSD1306_BLACK);
    oled.drawFastHLine(115, 10 + bob, 4, SSD1306_BLACK);
    for (int x = 98; x < 128; ++x) {
      oled.drawPixel(x, 28 + static_cast<int>(sinf(x * 0.2f - now * 0.001f)),
                     SSD1306_WHITE);
    }
  } else {
    const int bpm = constrain(static_cast<int>(lroundf(s.bpmCurrent)), 1, 999);
    char text[4];
    snprintf(text, sizeof(text), "%d", bpm);
    const int width = strlen(text) * 24;
    oled.setTextSize(4);
    oled.setCursor(0, 2); oled.print(text);
    oled.setTextSize(1);
    oled.setCursor(1 + width, 22); oled.print("bpm");
    // La animacion sigue el BPM valido; no genera ni modifica medidas.
    const float period = 60000.0f / s.bpmCurrent;
    const float p = fmodf(static_cast<float>(now), period) / period;
    const float beat = p < 0.32f ? sinf(p / 0.32f * PI) : 0;
    oledHeart(112, 10, 5 + static_cast<int>(2 * beat));
    int previousY = 27;
    for (int x = 98; x < 128; ++x) {
      const float phase = fmodf(p + (127 - x) / 30.0f, 1.0f);
      const int y = 27 - static_cast<int>(oledPulseShape(phase) / 3);
      if (x > 98) oled.drawLine(x - 1, previousY, x, y, SSD1306_WHITE);
      previousY = y;
    }
  }
  oled.display();
}

void oledTask(void *) {
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    PublicState state;
    portENTER_CRITICAL(&stateMux);
    state = publicState;
    portEXIT_CRITICAL(&stateMux);
    renderOled(state, millis());
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(OLED_FRAME_MS));
  }
}

void setupOled() {
  Wire.begin(OLED_SDA, OLED_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(20);
  // begin() de Adafruit no verifica presencia: comprobar ACK primero.
  for (uint8_t address = 0x3C; address <= 0x3D; ++address) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) { oledAddress = address; break; }
  }
  if (!oledAddress || !oled.begin(SSD1306_SWITCHCAPVCC, oledAddress, false, false)) {
    Serial.println("OLED_UNAVAILABLE check_SDA21_SCL22_and_SSD1306");
    return;
  }
  oled.setRotation(OLED_ROTATION);
  oled.clearDisplay();
  oled.display();
  oledReady = xTaskCreatePinnedToCore(oledTask, "oled-ui", 4096, nullptr,
                                    1, nullptr, 0) == pdPASS;
  Serial.printf("OLED_%s address=0x%02X size=128x32 rotation=180 fps=20\n",
                oledReady ? "READY" : "TASK_FAILED", oledAddress);
}

void setup() {
  Serial.begin(115200);
  delay(250);
  preferencesReady = preferences.begin("ppg", false);
  if (preferencesReady) serialPlotterEnabled = preferences.getBool("plotter", false);
  pinMode(SENSOR_PIN, INPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  setMotor(false);
  analogReadResolution(12);
  analogSetPinAttenuation(SENSOR_PIN, ADC_11db);
  filesystemReady = LittleFS.begin(true);
  ensureLogFile();

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  const bool ap = WiFi.softAP(AP_SSID, AP_PASSWORD);
  server.on("/", HTTP_GET, [] {
    server.sendHeader("Cache-Control", "no-store");
    server.send_P(200, "text/html; charset=utf-8", HTML_PAGE);
  });
  server.on("/api/state", HTTP_GET, handleState);
  server.on("/api/calibrate", HTTP_POST, handleCalibrate);
  server.on("/api/breathing", HTTP_POST, handleBreathing);
  server.on("/logs/calibration.log", HTTP_GET, handleLogDownload);
  server.onNotFound([] { server.send(404, "text/plain", "Not found"); });
  server.begin();

  xTaskCreatePinnedToCore(samplingTask, "ppg-sampler", 8192, nullptr, 3, nullptr, 1);
  setupOled();
  if (!serialPlotterEnabled) {
    Serial.printf("FIRMWARE %s web_bytes=%u\n", FIRMWARE_VERSION, static_cast<unsigned>(HTML_PAGE_SIZE));
    Serial.printf("BOOT board=keyestudio-esp32-plus gpio=%u littlefs=%s\n", SENSOR_PIN, filesystemReady ? "ready" : "failed");
    Serial.printf("AP_%s ssid=%s ip=%s\n", ap ? "READY" : "FAILED", AP_SSID, WiFi.softAPIP().toString().c_str());
    Serial.println("SERIAL_COMMANDS CALIBRATE LOGS PLOT_ON PLOT_OFF HELP");
  }
}

void loop() {
  uint32_t now = millis();
  collectBreathingMeasurement(now);
  updateBreathingPattern(now);
  updatePostBreathingAnalysis(now);
  server.handleClient();
  flushPendingLog();
  serviceSerial();
  now = millis();
  collectBreathingMeasurement(now);
  updateBreathingPattern(now);
  updatePostBreathingAnalysis(now);
  static uint32_t lastHealth = 0;
  if (!serialPlotterEnabled && millis() - lastHealth >= 5000) {
    lastHealth = millis();
    PublicState state;
    portENTER_CRITICAL(&stateMux);
    state = publicState;
    portEXIT_CRITICAL(&stateMux);
    Serial.printf("HEALTH ap=%u samples=%lu missed=%lu raw=%u heap=%u oled=%u oled_mode=%u bpm=%.1f valid=%u\n",
                  WiFi.softAPgetStationNum(), static_cast<unsigned long>(state.samples),
                  static_cast<unsigned long>(state.missed), state.raw,
                  static_cast<unsigned>(ESP.getFreeHeap()),
                  oledReady, oledMode(state), state.bpmCurrent, state.bpmValid);
  }
  delay(1);
}
