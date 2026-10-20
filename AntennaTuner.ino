/*
 * Automatic Antenna Tuner — ESP32 (LOLIN D32)  v6
 * ==============================================
 * Hardware:
 *   GPIO34 — ADC Forward  (0-3.3V dupa divizor)
 *   GPIO35 — ADC Reflected (0-3.3V dupa divizor)
 *   GPIO21 — I2C SDA -> Tic T825
 *   GPIO22 — I2C SCL -> Tic T825
 *
 * Librarii:
 *   - WiFiManager by tzapu        (Library Manager)
 *   - AsyncTCP                    (GitHub: me-no-dev)
 *   - ESPAsyncWebServer            (GitHub: me-no-dev)
 *
 * Tic T825: Control mode = Serial / I2C / USB  (I2C addr = 14)
 *
 * IMPORTANT Tic Control Center:
 *   Error response -> "Decelerate to zero and hold"  (elimina socul la reset)
 *
 * v6 changes (fixes applied after reviewing v5):
 *   - calc_swr(): the forward/reflected detector outputs are voltages
 *     (see volts_to_watts: power = V^2 * PWR_CAL), so the voltage
 *     reflection coefficient is simply ref/fwd. v5 took sqrt(ref/fwd),
 *     which is only correct if fwd/ref were already power readings —
 *     that made every displayed SWR read higher than the real value.
 *   - build_json(): the web dashboard reads "fwd_w" to show forward
 *     power, but v5 never included that field, so the power bar always
 *     showed 0.0 W even though volts_to_watts() was implemented and
 *     unused. Now computed and sent every poll.
 */

#include <WiFi.h>
#include <WiFiManager.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <math.h>

// ─── Pini ────────────────────────────────────────────────────────────
#define PIN_ADC_FWD   34
#define PIN_ADC_REF   35
#define I2C_SDA       21
#define I2C_SCL       22

// ─── Tic T825 ────────────────────────────────────────────────────────
#define TIC_ADDR      14
#define INVALID_POS   -999999  // valoare santinela pentru I2C esuat

// ─── Viteze motor (microsteps / 10000 secunde) ───────────────────────
// 200000 = 20 steps/s | 2000000 = 200 steps/s | 20000000 = 2000 steps/s
#define SPEED_JOG     200000     // jog manual — lent si precis
#define SPEED_TUNE    20000000   // auto-tune — rapid (2000 steps/s)
#define ACCEL_TUNE    4000000    // acceleratie tune

// ─── ADC ─────────────────────────────────────────────────────────────
#define ADC_SAMPLES   16   // mediere per citire individuala
#define ADC_MAX       4095.0f
#define PWR_MAX       100.0f   // W full scale
#define PWR_CAL       9.18f    // V^2 * CAL = W  (calibreaza dupa tandem match)

// ─── Algoritm sweep ──────────────────────────────────────────────────
// SEARCH_RANGE = cat explorezi in total (microsteps)
// La NEMA17 1/32 microstep: 6400 = 1 rotatie completa
// Daca condensatorul tau face 2 rotatii pentru sweep complet => 12800
#define SEARCH_RANGE  6400   // microsteps total de explorat
#define COARSE_STEP   640    // pas grosier → 6400/640 = 10 masuratori
#define FINE_RANGE    320    // ±pasi in jurul minimului grosier
#define FINE_STEP     32     // pas fin → 2*320/32 = 20 masuratori fine
#define SETTLE_MS     150    // ms stabilizare dupa miscare
#define MIN_SWR_OK    1.3f   // SWR la care ne oprim anticipat
#define SWR_HYSTERESIS 0.97f // noul minim trebuie sa fie cu 3% mai bun
#define FWD_MIN_V     0.05f  // tensiune minima Forward — sub asta = fara semnal RF
#define SANITY_RANGE  50000  // limita software ±50000 fata de start_pos

// ─── Stare globala ────────────────────────────────────────────────────
struct State {
  float   swr        = 99.0f;
  float   fwd_v      = 0.0f;
  float   ref_v      = 0.0f;
  bool    tuning     = false;
  int     progress   = 0;
  float   best_swr   = 99.0f;
  int32_t best_pos   = 0;
  String  status_msg = "Gata";
};

State g;
AsyncWebServer server(80);
volatile int32_t g_jog_vel = 0;  // viteza jog, setata de /manual, aplicata de loop
SemaphoreHandle_t i2c_mutex = NULL;  // protejeaza busul I2C intre tasks

// ─── Tic T825 — comenzi I2C (coduri verificate, doc Pololu sectiunea 8) ──
void tic_write(uint8_t cmd) {
  if (i2c_mutex && xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    Wire.beginTransmission(TIC_ADDR);
    Wire.write(cmd);
    Wire.endTransmission();
    xSemaphoreGive(i2c_mutex);
  }
}

void tic_write32(uint8_t cmd, int32_t val) {
  if (i2c_mutex && xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    Wire.beginTransmission(TIC_ADDR);
    Wire.write(cmd);
    Wire.write((uint8_t)(val        & 0xFF));
    Wire.write((uint8_t)((val >> 8) & 0xFF));
    Wire.write((uint8_t)((val >>16) & 0xFF));
    Wire.write((uint8_t)((val >>24) & 0xFF));
    Wire.endTransmission();
    xSemaphoreGive(i2c_mutex);
  }
}

void tic_energize()        { tic_write(0x85); }
void tic_exit_safe_start() { tic_write(0x83); }
void tic_reset_timeout()   { tic_write(0x8C); }
void tic_set_velocity(int32_t v)   { tic_write32(0xE3, v); }
void tic_set_position(int32_t pos) { tic_write32(0xE0, pos); }
void tic_set_max_speed(uint32_t s) { tic_write32(0xE6, s); }
void tic_set_max_accel(uint32_t a) { tic_write32(0xEA, a); }
void tic_set_max_decel(uint32_t d) { tic_write32(0xE9, d); }

// Citire pozitie reala din Tic (Get variable 0xA1, offset 0x22 = current position)
// Returneaza INVALID_POS daca I2C esueaza
int32_t tic_get_position() {
  if (!i2c_mutex || xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(50)) != pdTRUE)
    return INVALID_POS;
  Wire.beginTransmission(TIC_ADDR);
  Wire.write(0xA1);  // Get variable
  Wire.write(0x22);  // offset: current position (32-bit signed, doc sectiunea 7)
  Wire.endTransmission(false);
  int n = Wire.requestFrom((uint8_t)TIC_ADDR, (uint8_t)4);
  int32_t pos = INVALID_POS;
  if (n >= 4) {
    pos = 0;
    for (int i = 0; i < 4; i++)
      pos |= (int32_t)Wire.read() << (8 * i);
  }
  xSemaphoreGive(i2c_mutex);
  return pos;
}

void tic_init() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  delay(500);
  tic_energize();
  delay(100);
  tic_exit_safe_start();
  tic_set_max_speed(SPEED_JOG);
  tic_set_max_accel(20000);
  tic_set_max_decel(20000);
  tic_set_velocity(0);
  Serial.println("Tic T825 initializat.");
}

// ─── ADC — citire cu mediere ─────────────────────────────────────────
float read_adc(int pin) {
  long sum = 0;
  for (int i = 0; i < ADC_SAMPLES; i++) {
    sum += analogRead(pin);
    delayMicroseconds(500);
  }
  return (sum / (float)ADC_SAMPLES) / ADC_MAX * 3.3f;
}

// ─── Median din 3 citiri ─────────────────────────────────────────────
// Mediana elimina spike-urile RF mai bine decat media, fara cost de timp suplimentar.
float read_adc_median(int pin) {
  float a = read_adc(pin);
  float b = read_adc(pin);
  float c = read_adc(pin);
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;  // valoarea din mijloc
}

// ─── Calcul SWR ──────────────────────────────────────────────────────
// fwd/ref sunt tensiuni (vezi volts_to_watts: putere = V^2 * PWR_CAL),
// deci coeficientul de reflexie este direct ref/fwd — FARA sqrt.
// (sqrt ar fi fost corect doar daca fwd/ref erau deja citiri de putere.)
float calc_swr(float fwd, float ref) {
  if (fwd < FWD_MIN_V) return 99.0f;
  float rho = ref / fwd;
  if (rho >= 0.99f) return 99.0f;
  return (1.0f + rho) / (1.0f - rho);
}

float volts_to_watts(float v){return (v<0.01f)?0.0f:(v*v*PWR_CAL);}

// ─── Timeout adaptiv bazat pe distanta si viteza ─────────────────────
uint32_t calc_timeout(int32_t from, int32_t to) {
  float steps = abs(to - from);
  float speed_steps_per_s = SPEED_TUNE / 10000.0f;
  uint32_t move_ms = (uint32_t)(steps / speed_steps_per_s * 1000.0f);
  return move_ms + 800;  // +800ms buffer pentru rampa acceleratie
}

// ─── Miscare la pozitie exacta cu confirmare I2C ─────────────────────
bool move_to_and_wait(int32_t target) {
  tic_reset_timeout();
  tic_exit_safe_start();
  tic_set_position(target);

  int32_t current = tic_get_position();
  uint32_t timeout_ms = calc_timeout(current, target);
  unsigned long t0 = millis();

  while (millis() - t0 < timeout_ms) {
    tic_reset_timeout();
    tic_exit_safe_start();
    int32_t pos = tic_get_position();
    if (pos == INVALID_POS) { delay(30); yield(); continue; }
    if (abs(pos - target) <= 5) {
      delay(SETTLE_MS);
      return true;
    }
    delay(30);
    yield();
  }
  Serial.println("WARN: timeout la pos=" + String(target));
  return false;
}

// ─── Masurare SWR la pozitie (median din 3) ──────────────────────────
float measure_at(int32_t pos) {
  if (!move_to_and_wait(pos)) {
    Serial.println("WARN: miscare esuata la pos=" + String(pos));
  }
  g.fwd_v = read_adc_median(PIN_ADC_FWD);
  g.ref_v = read_adc_median(PIN_ADC_REF);
  float swr = calc_swr(g.fwd_v, g.ref_v);
  g.swr = swr;
  int32_t real = tic_get_position();
  // Serial.printf verbose dezactivat — cauza partial pentru reset aleatoriu
  // (buffer serial plin in combinatie cu WiFi stack)
  return swr;
}

// ─── AUTO TUNE ────────────────────────────────────────────────────────
//
//  Imbunatatiri fata de versiunile anterioare:
//  - Verifica semnal RF inainte de a incepe
//  - Sweep bidirecțional: start - range/2 → start + range/2
//    (evita ratarea minimului daca acesta e "in spate" fata de pozitia curenta)
//  - Histerezis relativ 3%: noul best trebuie sa fie cu 3% mai bun
//  - Median din 3 per masurare (elimina spike-uri RF)
//  - Timeout adaptiv la miscare (distanta / viteza + buffer)
//  - Limita software sanity check cu abort explicit
//  - Fine sweep cu clampare la limitele sanity
//  - Reset viteza la jog dupa finalizare
//
void auto_tune_task(void* param) {
  g.tuning   = true;
  g.best_swr = 99.0f;
  g.progress = 0;

  // ── Verifica semnal RF ────────────────────────────────────────────
  g.status_msg = "Verificare semnal RF...";
  float test_fwd = read_adc(PIN_ADC_FWD);
  if (test_fwd < FWD_MIN_V) {
    g.status_msg = "Eroare: fara semnal RF! Transmite pe transceiver.";
    Serial.println("ABORT: fara semnal RF (fwd=" + String(test_fwd, 3) + "V)");
    goto tune_done;
  }
  Serial.println("Semnal RF OK: fwd=" + String(test_fwd, 3) + "V");

  {
    // ── Viteza mare pentru sweep rapid ───────────────────────────────
    tic_set_max_speed(SPEED_TUNE);
    tic_set_max_accel(ACCEL_TUNE);
    tic_set_max_decel(ACCEL_TUNE);

    // ── Pozitie start (citita real din Tic) ──────────────────────────
    int32_t start_pos = tic_get_position();
    if (start_pos == INVALID_POS) {
      g.status_msg = "Eroare: I2C Tic nu raspunde!";
      goto tune_done;
    }

    // Sweep bidirecțional: porneste cu SEARCH_RANGE/2 in urma pozitiei curente
    int32_t sweep_start = start_pos - SEARCH_RANGE / 2;
    int32_t sweep_end   = start_pos + SEARCH_RANGE / 2;

    // Limita sanity software (nu constrain silentios — abort explicit)
    int32_t lo = start_pos - SANITY_RANGE;
    int32_t hi = start_pos + SANITY_RANGE;
    if (sweep_start < lo || sweep_end > hi) {
      g.status_msg = "Eroare: SEARCH_RANGE depaseste limita sanity!";
      Serial.println("ABORT: sweep out of sanity bounds");
      goto tune_done;
    }

    int num_coarse = SEARCH_RANGE / COARSE_STEP;
    int32_t coarse_best_pos = sweep_start;
    float   coarse_best_swr = 99.0f;

    // ── Faza 1: Sweep grosier bidirecțional ──────────────────────────
    g.status_msg = "Sweep grosier...";
    Serial.printf("Sweep: %d -> %d  (%d masuratori)\n",
                  (int)sweep_start, (int)sweep_end, num_coarse + 1);

    for (int i = 0; i <= num_coarse; i++) {
      if (!g.tuning) goto tune_done;

      int32_t pos = sweep_start + (int32_t)i * COARSE_STEP;
      float swr = measure_at(pos);

      // Histerezis relativ: accepta noul minim doar daca e cu 3% mai bun
      if (swr < coarse_best_swr * SWR_HYSTERESIS) {
        coarse_best_swr = swr;
        coarse_best_pos = pos;
      }

      g.best_swr = coarse_best_swr;
      g.best_pos = coarse_best_pos;
      g.progress = (int)(i * 60.0f / num_coarse);

      if (coarse_best_swr <= MIN_SWR_OK) {
        Serial.println("Early stop coarse: SWR=" + String(coarse_best_swr, 2));
        goto move_to_best;
      }
    }

    {
      // ── Faza 2: Sweep fin in jurul minimului grosier ─────────────────
      g.status_msg = "Sweep fin...";

      // Clampare fine sweep la limitele sanity (evita depasire accidentala)
      int32_t fine_start = max(coarse_best_pos - FINE_RANGE, lo);
      int32_t fine_end   = min(coarse_best_pos + FINE_RANGE, hi);
      int num_fine = (int)((fine_end - fine_start) / FINE_STEP);

      Serial.printf("Fine sweep: %d -> %d  (%d masuratori)\n",
                    (int)fine_start, (int)fine_end, num_fine + 1);

      int32_t fine_best_pos = coarse_best_pos;
      float   fine_best_swr = coarse_best_swr;

      for (int i = 0; i <= num_fine; i++) {
        if (!g.tuning) goto tune_done;

        int32_t pos = fine_start + (int32_t)i * FINE_STEP;
        float swr = measure_at(pos);

        if (swr < fine_best_swr * SWR_HYSTERESIS) {
          fine_best_swr = swr;
          fine_best_pos = pos;
        }

        g.best_swr = fine_best_swr;
        g.best_pos = fine_best_pos;
        g.progress = 60 + (int)(i * 35.0f / num_fine);

        if (fine_best_swr <= MIN_SWR_OK) {
          Serial.println("Early stop fine: SWR=" + String(fine_best_swr, 2));
          break;
        }
      }
    }

move_to_best:
    // ── Faza 3: Mergi la pozitia optima ─────────────────────────────
    g.status_msg = "Mutare la minim...";
    Serial.printf("Best: pos=%d SWR=%.2f\n", (int)g.best_pos, g.best_swr);

    move_to_and_wait(g.best_pos);

    // Masurare finala de confirmare (median din 3)
    g.fwd_v = read_adc_median(PIN_ADC_FWD);
    g.ref_v = read_adc_median(PIN_ADC_REF);
    g.swr   = calc_swr(g.fwd_v, g.ref_v);
    g.progress = 100;

    if (g.swr <= MIN_SWR_OK)
      g.status_msg = "Acord reusit! SWR: " + String(g.swr, 2);
    else
      g.status_msg = "Minim: SWR " + String(g.swr, 2)
                     + "  (best: " + String(g.best_swr, 2) + ")";

    Serial.println("=== DONE ===  SWR final: " + String(g.swr, 2));
  }

tune_done:
  // Reset viteza la valorile pentru jog manual
  tic_set_max_speed(SPEED_JOG);
  tic_set_max_accel(20000);
  tic_set_max_decel(20000);
  g.tuning = false;
  vTaskDelete(NULL);
}

// ─── JSON status ─────────────────────────────────────────────────────
String build_json() {
  String st = g.status_msg;
  st.replace("\"", "'");
  int32_t pos = tic_get_position();
  String j = "{";
  j += "\"swr\":"      + String(g.swr, 3)      + ",";
  j += "\"fwd\":"      + String(g.fwd_v, 3)    + ",";
  j += "\"ref\":"      + String(g.ref_v, 3)    + ",";
  j += "\"fwd_w\":"    + String(volts_to_watts(g.fwd_v), 2) + ",";
  j += "\"position\":" + String(pos == INVALID_POS ? 0 : pos) + ",";
  j += "\"tuning\":"   + String(g.tuning ? "true" : "false")  + ",";
  j += "\"progress\":" + String(g.progress)    + ",";
  j += "\"best_swr\":" + String(g.best_swr, 2) + ",";
  j += "\"best_pos\":" + String(g.best_pos)    + ",";
  j += "\"status\":\""  + st                   + "\"";
  j += "}";
  return j;
}

// ─── HTML Dashboard ───────────────────────────────────────────────────
const char HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="ro">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
<title>Antenna Tuner</title>
<link href="https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Oswald:wght@400;600&display=swap" rel="stylesheet">
<style>
:root{
  --bg0:#0a0c0e;--bg1:#111416;--bg2:#181b1e;--bg3:#1f2327;
  --border:#2a2f34;--acc:#e8851a;--grn:#3ec97a;--red:#e83a3a;
  --yel:#e8c21a;--txt:#d4cfc5;--mut:#6b7177;
  --mono:'Share Tech Mono',monospace;--disp:'Oswald',sans-serif;
}
*{box-sizing:border-box;margin:0;padding:0}
body{
  background:var(--bg0);color:var(--txt);font-family:var(--mono);
  padding:10px;display:flex;flex-direction:column;gap:8px;
  max-width:760px;margin:0 auto;min-height:100vh;
  background-image:
    repeating-linear-gradient(0deg,transparent,transparent 39px,rgba(42,47,52,.25) 40px),
    repeating-linear-gradient(90deg,transparent,transparent 39px,rgba(42,47,52,.08) 40px);
  background-size:40px 40px;
}
header{
  display:flex;align-items:center;justify-content:space-between;
  border-bottom:1px solid var(--border);padding-bottom:8px;flex-shrink:0;
}
.logo{font-family:var(--disp);font-size:18px;font-weight:600;
  letter-spacing:3px;color:var(--acc);text-transform:uppercase}
.logo em{color:var(--mut);font-style:normal;font-weight:400;font-size:12px;letter-spacing:1px}
.conn{display:flex;align-items:center;gap:6px}
#sdot{width:9px;height:9px;border-radius:50%;background:var(--red);
  box-shadow:0 0 6px var(--red);transition:all .3s}
#sdot.live{background:var(--grn);box-shadow:0 0 6px var(--grn)}
.slbl{font-size:10px;color:var(--mut);letter-spacing:2px}

.card{
  background:var(--bg1);border:1px solid var(--border);border-radius:4px;
  padding:12px 16px;position:relative;overflow:hidden;
}
.card::before{content:'';position:absolute;top:0;left:0;right:0;height:2px;
  background:linear-gradient(90deg,transparent,var(--acc),transparent);opacity:.5}
.ct{font-family:var(--disp);font-size:9px;letter-spacing:4px;
  text-transform:uppercase;color:var(--mut);margin-bottom:8px}

/* ── TOP ROW: SWR + FWD/REF ── */
.top-row{display:flex;flex-direction:column;gap:8px}

/* SWR */
.big{font-family:var(--disp);font-size:52px;font-weight:600;
  line-height:1;letter-spacing:-1px;color:var(--acc);transition:color .3s}
.big.g{color:var(--grn)}.big.w{color:var(--yel)}.big.b{color:var(--red)}
.unit{font-family:var(--disp);font-size:18px;color:var(--mut)}

.vu{height:14px;background:var(--bg2);border:1px solid var(--border);
  border-radius:2px;overflow:hidden;display:flex;gap:2px;padding:3px;margin-top:8px}
.seg{flex:1;border-radius:1px;background:var(--bg3);transition:background 70ms}
.pwr-lbl{display:flex;justify-content:space-between;font-size:9px;color:var(--mut);margin:6px 0 4px}
.swr-scale{position:relative;height:14px;margin-top:4px}
.swr-tick{position:absolute;transform:translateX(-50%);font-size:9px;color:var(--mut);white-space:nowrap}

.badge{display:inline-block;font-family:var(--disp);font-size:9px;
  letter-spacing:2px;text-transform:uppercase;padding:3px 10px;
  border-radius:2px;border:1px solid var(--border);
  background:var(--bg3);color:var(--mut);transition:all .3s;margin-top:8px}
.badge.good{background:rgba(62,201,122,.12);border-color:var(--grn);color:var(--grn)}
.badge.warn{background:rgba(232,194,26,.1);border-color:var(--yel);color:var(--yel)}
.badge.bad{background:rgba(232,58,58,.12);border-color:var(--red);color:var(--red)}

/* FWD / REF */
.fwd-big{font-family:var(--disp);font-size:48px;font-weight:600;
  line-height:1;letter-spacing:-1px;color:var(--acc)}
.ref-big{font-family:var(--disp);font-size:30px;font-weight:600;
  line-height:1;letter-spacing:-0.5px;color:var(--mut);margin-top:10px}
.fvu{height:12px;background:var(--bg2);border:1px solid var(--border);
  border-radius:2px;overflow:hidden;display:flex;gap:2px;padding:3px;margin-top:8px}

/* ── AUTO TUNE ── */
.tune-row{display:flex;gap:8px;margin-bottom:8px}
.btn-tune{
  font-family:var(--disp);font-size:13px;font-weight:600;letter-spacing:3px;
  text-transform:uppercase;padding:13px 0;border-radius:3px;
  border:1px solid var(--border);background:var(--bg2);color:var(--mut);
  cursor:pointer;flex:1;position:relative;overflow:hidden;transition:all .15s;
  user-select:none;
}
.btn-tune.active{border-color:var(--acc);background:rgba(232,133,26,.12);
  color:var(--acc);box-shadow:0 0 12px rgba(232,133,26,.2)}
.btn-tune.active::before{content:'';position:absolute;top:0;left:0;right:0;height:2px;
  background:var(--acc);animation:pl .6s ease-in-out infinite alternate}
@keyframes pl{from{opacity:.3}to{opacity:1}}
.btn-tune:disabled{opacity:.3;cursor:not-allowed}
.btn-stop{
  font-family:var(--disp);font-size:12px;font-weight:600;letter-spacing:2px;
  text-transform:uppercase;padding:13px 22px;border-radius:3px;
  border:1px solid rgba(232,58,58,.5);background:rgba(232,58,58,.07);
  color:var(--red);cursor:pointer;transition:all .15s;user-select:none;
}
.prog-row{display:flex;align-items:center;gap:8px;margin-bottom:5px}
.prog-track{flex:1;height:4px;background:var(--bg3);border-radius:2px;overflow:hidden}
.prog-bar{height:100%;width:0%;background:var(--acc);border-radius:2px;transition:width .4s}
.prog-bar.done{background:var(--grn)}
.prog-pct{font-size:10px;color:var(--acc);min-width:28px;text-align:right}
.smsg{font-size:10px;color:var(--mut);min-height:13px}

/* ── MANUAL ── */
.sl-row{display:flex;align-items:center;gap:8px;margin-bottom:8px}
.sl-row label{font-size:9px;color:var(--mut);letter-spacing:2px;white-space:nowrap}
.sl-row input[type=range]{flex:1;accent-color:var(--acc)}
.sl-row .sv{font-size:10px;color:var(--acc);min-width:30px;text-align:right}
.jog-row{display:flex;gap:8px}
.jog-btn{
  flex:1;font-family:var(--disp);font-size:13px;font-weight:600;
  letter-spacing:3px;text-transform:uppercase;padding:14px 0;
  border-radius:3px;border:1px solid var(--border);background:var(--bg2);
  color:var(--mut);cursor:pointer;transition:all .12s;user-select:none;
  -webkit-tap-highlight-color:transparent;
}
.jog-btn.fon{border-color:var(--grn);background:rgba(62,201,122,.12);
  color:var(--grn);box-shadow:0 0 10px rgba(62,201,122,.2)}
.jog-btn.ron{border-color:var(--red);background:rgba(232,58,58,.12);
  color:var(--red);box-shadow:0 0 10px rgba(232,58,58,.2)}

/* ── INFO ── */
.info-row{display:flex;gap:8px}
.ic{flex:1;background:var(--bg2);border:1px solid var(--border);
  border-radius:3px;padding:6px 10px}
.ik{font-size:9px;color:var(--mut);letter-spacing:2px;margin-bottom:3px}
.iv{font-family:var(--disp);font-size:16px;color:var(--txt)}
.iv.a{color:var(--acc)}.iv.g{color:var(--grn)}
</style>
</head>
<body>

<header>
  <div class="logo">ANTENNA TUNER <em>· SWR / MOTOR</em></div>
  <div class="conn">
    <span id="sdot"></span>
    <span class="slbl" id="stxt">OFFLINE</span>
  </div>
</header>

<!-- FORWARD POWER -->
<div class="card">
  <div class="ct">Forward Power</div>
  <div><span class="big" id="fwd-num" style="color:var(--acc)">--</span><span class="unit"> W</span></div>
  <div class="pwr-lbl"><span>0</span><span>10</span><span>25</span><span>50</span><span>75</span><span>100</span></div>
  <div class="vu" id="fwd-track"></div>
</div>

<!-- SWR -->
<div class="card">
  <div class="ct">SWR</div>
  <div><span class="big" id="swr-num">--</span><span class="unit"> :1</span></div>
  <div class="vu" id="swr-track"></div>
  <div class="swr-scale" id="swr-scale"></div>
  <div class="badge" id="badge">--</div>
</div>

<!-- AUTO TUNE -->
<div class="card">
  <div class="ct">Auto Tune</div>
  <div class="tune-row">
    <button class="btn-tune" id="btn-tune" onclick="startTune()">AUTO TUNE</button>
    <button class="btn-stop" onclick="stopAll()">■ STOP</button>
  </div>
  <div class="prog-row">
    <div class="prog-track"><div class="prog-bar" id="prog-bar"></div></div>
    <span class="prog-pct" id="prog-pct">0%</span>
  </div>
  <div class="smsg" id="smsg">Gata</div>
</div>

<!-- MANUAL -->
<div class="card">
  <div class="ct">Control Manual</div>
  <div class="sl-row">
    <label>VITEZA</label>
    <input type="range" min="1" max="100" value="40" id="vel-slider"
      oninput="document.getElementById('vel-val').textContent=this.value+'%'">
    <span class="sv" id="vel-val">40%</span>
  </div>
  <div class="jog-row">
    <button class="jog-btn" id="btn-rev">◀ REV</button>
    <button class="jog-btn" id="btn-fwd">FWD ▶</button>
  </div>
</div>

<!-- INFO -->
<div class="card" style="padding:10px 16px">
  <div class="info-row">
    <div class="ic">
      <div class="ik">POZITIE (pasi)</div>
      <div class="iv a" id="pos-val">--</div>
    </div>
    <div class="ic">
      <div class="ik">BEST SWR</div>
      <div class="iv g" id="best-swr">--</div>
    </div>
    <div class="ic">
      <div class="ik">BEST POZITIE</div>
      <div class="iv" id="best-pos">--</div>
    </div>
  </div>
</div>

<script>
const SWR_SEGS=40,SWR_MAX=3,FWD_SEGS=28,MAX_VEL=200000;
const swrTicks=[1,1.5,2.0,2.5,3.0];
function swrF(s){return(Math.max(1,Math.min(s,SWR_MAX))-1)/(SWR_MAX-1);}
(()=>{
  const st=document.getElementById('swr-track');
  const sc=document.getElementById('swr-scale');
  for(let i=0;i<SWR_SEGS;i++){const s=document.createElement('div');s.className='seg';s.id='ss'+i;st.appendChild(s);}
  swrTicks.forEach(v=>{
    const pct=swrF(v)*100;
    const el=document.createElement('span');el.className='swr-tick';
    el.style.left=pct+'%';el.textContent=v%1===0?v+'':v.toFixed(1);
    if(pct<1||pct>99)el.style.display='none';
    sc.appendChild(el);
  });
  const ft=document.getElementById('fwd-track');
  for(let i=0;i<FWD_SEGS;i++){const s=document.createElement('div');s.className='seg';s.id='fs'+i;ft.appendChild(s);}
})();
function updSWR(v){
  const l=Math.round(swrF(v)*SWR_SEGS),l15=Math.round(swrF(1.5)*SWR_SEGS),l20=Math.round(swrF(2)*SWR_SEGS);
  for(let i=0;i<SWR_SEGS;i++)document.getElementById('ss'+i).style.background=
    i<l?(i<l15?'#3ec97a':i<l20?'#e8c21a':'#e83a3a'):'var(--bg3)';
}
function updFwd(w){
  // w = watts, scala 0-100W
  const frac=Math.min(w/100,1);
  const l=Math.round(frac*FWD_SEGS);
  for(let i=0;i<FWD_SEGS;i++){const p=i/FWD_SEGS;
    document.getElementById('fs'+i).style.background=i<l?(p<0.6?'#3ec97a':p<0.85?'#e8c21a':'#e83a3a'):'var(--bg3)';}
}
let tuning=false;
async function poll(){
  try{
    const d=await(await fetch('/status')).json();
    const swr=parseFloat(d.swr),fwd=parseFloat(d.fwd),ref=parseFloat(d.ref);
    tuning=d.tuning==='true'||d.tuning===true;
    document.getElementById('sdot').className='live';
    document.getElementById('stxt').textContent='LIVE';
    const sn=document.getElementById('swr-num');
    sn.textContent=swr>=90?'--':swr.toFixed(2);
    sn.className='big '+(swr<1.5?'g':swr<2.5?'w':swr<90?'b':'');
    if(swr<90)updSWR(swr);
    const fw=parseFloat(d.fwd_w)||0;
    document.getElementById('fwd-num').textContent=fw.toFixed(1);
    updFwd(fw);
    document.getElementById('pos-val').textContent=parseInt(d.position);
    const bg=document.getElementById('badge');
    if(swr>=90){bg.className='badge';bg.textContent='NO SIGNAL';}
    else if(swr<1.5){bg.className='badge good';bg.textContent='EXCELLENT';}
    else if(swr<2.0){bg.className='badge good';bg.textContent='GOOD';}
    else if(swr<2.5){bg.className='badge warn';bg.textContent='FAIR';}
    else{bg.className='badge bad';bg.textContent='POOR';}
    const prog=parseInt(d.progress);
    document.getElementById('prog-bar').style.width=prog+'%';
    document.getElementById('prog-bar').className='prog-bar'+(prog>=100?' done':'');
    document.getElementById('prog-pct').textContent=prog+'%';
    document.getElementById('smsg').textContent=d.status||'';
    const bs=parseFloat(d.best_swr);
    if(bs<90){
      document.getElementById('best-swr').textContent=bs.toFixed(2);
      document.getElementById('best-pos').textContent=d.best_pos+' pasi';
    }
    const bt=document.getElementById('btn-tune');
    bt.disabled=tuning;bt.className='btn-tune'+(tuning?' active':'');
  }catch(e){
    document.getElementById('sdot').className='';
    document.getElementById('stxt').textContent='OFFLINE';
  }
  setTimeout(poll,tuning?400:900);
}
async function startTune(){await fetch('/tune');}
async function stopAll(){await fetch('/stop');}
function vel(dir){return Math.round(parseInt(document.getElementById('vel-slider').value)/100*MAX_VEL)*dir;}
async function jog(dir){if(!tuning)await fetch('/manual?vel='+vel(dir)).catch(()=>{});}
const F=document.getElementById('btn-fwd'),R=document.getElementById('btn-rev');
F.addEventListener('mousedown', ()=>{F.classList.add('fon');jog(1);});
F.addEventListener('mouseup',   ()=>{F.classList.remove('fon');jog(0);});
F.addEventListener('mouseleave',()=>{F.classList.remove('fon');jog(0);});
R.addEventListener('mousedown', ()=>{R.classList.add('ron');jog(-1);});
R.addEventListener('mouseup',   ()=>{R.classList.remove('ron');jog(0);});
R.addEventListener('mouseleave',()=>{R.classList.remove('ron');jog(0);});
F.addEventListener('touchstart',e=>{e.preventDefault();F.classList.add('fon');jog(1);});
F.addEventListener('touchend',  e=>{e.preventDefault();F.classList.remove('fon');jog(0);});
R.addEventListener('touchstart',e=>{e.preventDefault();R.classList.add('ron');jog(-1);});
R.addEventListener('touchend',  e=>{e.preventDefault();R.classList.remove('ron');jog(0);});
poll();
</script>
</body>
</html>

)rawhtml";

// ─── Setup ────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== Antenna Tuner v6 Boot ===");

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  i2c_mutex = xSemaphoreCreateMutex();
  tic_init();

  delay(2000);  // FIX: lasa router-ul sa fie disponibil la pornire fara PC
  WiFiManager wm;
  wm.setConnectRetries(5);
  wm.setConnectTimeout(15);
  wm.setConfigPortalTimeout(180);
  // IP static optional — decomenteaza si ajusteaza:
  // wm.setSTAStaticIPConfig(IPAddress(192,168,1,200),
  //                         IPAddress(192,168,1,1),
  //                         IPAddress(255,255,255,0));

  if (!wm.autoConnect("AntennaTuner-Setup")) {
    Serial.println("WiFi esuat — restart");
    delay(3000);
    ESP.restart();
  }
  Serial.println("WiFi: " + WiFi.localIP().toString());

  // mDNS — accesat ca http://esp32.local
  if (MDNS.begin("esp32")) {
    MDNS.addService("http", "tcp", 80);
    Serial.println("mDNS: http://esp32.local");
  } else {
    Serial.println("WARN: mDNS esuat");
  }

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* r){
    r->send_P(200, "text/html", HTML);
  });
  server.on("/status", HTTP_GET, [](AsyncWebServerRequest* r){
    r->send(200, "application/json", build_json());
  });
  server.on("/tune", HTTP_GET, [](AsyncWebServerRequest* r){
    r->send(200, "text/plain", "ok");
    if (!g.tuning)
      xTaskCreatePinnedToCore(auto_tune_task, "tune", 8192, NULL, 1, NULL, 1);
  });
  server.on("/stop", HTTP_GET, [](AsyncWebServerRequest* r){
    g.tuning  = false;
    g_jog_vel = 0;
    tic_set_velocity(0);
    g.status_msg = "Oprit";
    r->send(200, "text/plain", "ok");
  });
  server.on("/manual", HTTP_GET, [](AsyncWebServerRequest* r){
    if (r->hasParam("vel")) {
      g_jog_vel = (int32_t)r->getParam("vel")->value().toInt();
      // Seteaza max_speed = viteza ceruta ca Tic sa nu o clampeze
      uint32_t spd = (uint32_t)abs(g_jog_vel);
      if (spd > 0) tic_set_max_speed(spd);
    }
    r->send(200, "text/plain", "ok");
  });

  server.begin();
  Serial.println("Server: http://" + WiFi.localIP().toString());
}

// ─── Loop — 50ms ─────────────────────────────────────────────────────
// exit_safe_start trimis la fiecare 50ms (fereastra 200ms conform doc Pololu)
// set_velocity(g_jog_vel) trimis mereu — 0 inseamna stop explicit
void loop() {
  if (!g.tuning) {
    tic_reset_timeout();
    tic_exit_safe_start();
    tic_set_velocity(g_jog_vel);

    static uint8_t tick = 0;
    if (++tick >= 4) {
      tick = 0;
      g.fwd_v = read_adc(PIN_ADC_FWD);
      g.ref_v = read_adc(PIN_ADC_REF);
      g.swr   = calc_swr(g.fwd_v, g.ref_v);
    }
  }

  // Interfata Serial USB — acelasi protocol JSON ca HTTP
  // Comenzi: STATUS / TUNE / STOP / VEL:<val>
  // Raspuns: JSON pe o singura linie terminata cu \n
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "STATUS") {
      // Raspuns JSON identic cu /status HTTP
      Serial.println(build_json());
    }
    else if (cmd == "TUNE") {
      if (!g.tuning) {
        xTaskCreatePinnedToCore(auto_tune_task, "tune_usb", 8192, NULL, 1, NULL, 1);
        Serial.println("{\"ok\":true,\"msg\":\"tune started\"}");
      } else {
        Serial.println("{\"ok\":false,\"msg\":\"already tuning\"}");
      }
    }
    else if (cmd == "STOP") {
      g.tuning  = false;
      g_jog_vel = 0;
      tic_set_velocity(0);
      g.status_msg = "Oprit";
      Serial.println("{\"ok\":true,\"msg\":\"stopped\"}");
    }
    else if (cmd.startsWith("VEL:")) {
      g_jog_vel = (int32_t)cmd.substring(4).toInt();
      Serial.println("{\"ok\":true,\"vel\":" + String(g_jog_vel) + "}");
    }
    else if (cmd == "PING") {
      Serial.println("{\"ok\":true,\"msg\":\"pong\"}");
    }
    // Comenzi debug
    else if (cmd == "POS") {
      Serial.println("{\"position\":" + String(tic_get_position()) + "}");
    }
  }

  // WiFi reconnect la 30s
  static unsigned long last_wifi = 0;
  if (millis() - last_wifi > 30000) {
    last_wifi = millis();
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi lost — reconnect...");
      WiFi.reconnect();
    }
  }

  delay(50);
  yield();
}
