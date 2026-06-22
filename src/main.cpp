/*
 * LumiGlyph / LumiGraph
 *
 * env:m5stick-c  … 元のM5StickC Plus版 (LumiGlyph)
 * env:Drawing    … AtomS3R-CAM: 5-bar linkage描画 + カメラ映像送信 (ROLE_DRAWING)
 * env:Display    … AtomS3R    : 映像受信表示 + ボタンでグリフ指示 (ROLE_DISPLAY)
 *
 * [ペアリング手順]
 *   1. Drawing (AtomS3R-CAM) と Display (AtomS3R) を両方起動
 *   2. Display 側で BtnA 長押し → ENQ 送信
 *   3. Drawing が ACK 返信 → Display が MAC 保存して自動再起動
 *
 * [Display ボタン操作]
 *   OK  (G38) → イ を白で描画
 *   NG  (G39) → チ を白で描画
 *   TRG (G5)  → d-i-g-i-k-e-y を赤で順次描画
 *   BtnA 長押し → ペアリング
 */

// ════════════════════════════════════════════════════════════════
//  ROLE_DRAWING : AtomS3R-CAM — カメラ映像送信 + 5-bar描画
// ════════════════════════════════════════════════════════════════
#ifdef ROLE_DRAWING

#include <FastLED.h>
#include <ESPNowCam.h>
#include <esp_camera.h>
#include <esp_now.h>
#include <WiFi.h>
#include "FS.h"
#include "SPIFFS.h"
#include "driver/ledc.h"
#include "ik.h"
#include "glyphs.h"

// ── カメラ GPIO (AtomS3R-CAM / M5StampS3 + OV2640) ──────────
static constexpr int CAM_POWER_PIN = 18;
static constexpr int CAM_PIN_XCLK  = 21;
static constexpr int CAM_PIN_SIOD  = 12;
static constexpr int CAM_PIN_SIOC  =  9;
static constexpr int CAM_PIN_VSYNC = 10;
static constexpr int CAM_PIN_HREF  = 14;
static constexpr int CAM_PIN_PCLK  = 40;
static constexpr int CAM_PIN_Y9    = 13;
static constexpr int CAM_PIN_Y8    = 11;
static constexpr int CAM_PIN_Y7    = 17;
static constexpr int CAM_PIN_Y6    =  4;
static constexpr int CAM_PIN_Y5    = 48;
static constexpr int CAM_PIN_Y4    = 46;
static constexpr int CAM_PIN_Y3    = 42;
static constexpr int CAM_PIN_Y2    =  3;

// ── サーボ LEDC ─────────────────────────────────────────────
// Camera XCLK が LEDC_TIMER_0 (CH0/CH1) を占有するため CH4/CH5 (Timer2) を使用
static constexpr int SERVO_PIN1  =  7;
static constexpr int SERVO_PIN2  =  6;
static constexpr int LEDC_CH_SV1 =  4;
static constexpr int LEDC_CH_SV2 =  5;
static constexpr int SERVO_FREQ  = 50;
static constexpr int SERVO_BITS  = 10;
static constexpr int SERVO_MIN_W = 26;    // ≈500µs
static constexpr int SERVO_MAX_W = 125;   // ≈2400µs
static int SERVO1_TRIM = 0;              // キャリブレーション補正値 (SPIFFS /trim.dat)
static int SERVO2_TRIM = 0;

// ── WS2812B (ペン LED) ──────────────────────────────────────
static constexpr int LED_PIN  = 8;
static constexpr int NUM_LEDS = 1;

// ── 描画パラメータ ───────────────────────────────────────────
static constexpr float HOME_X         = 18.0f;
static constexpr float HOME_Y         = 75.0f;  // 元LumiGlyph準拠 (GLYPH_CYと同一)
static constexpr int   STEP_MS        = 20;
static constexpr int   INTERP_STEPS   = 5;
static constexpr int   TARGET_DRAW_MS = 500;
static constexpr int   MIN_STEP_MS    = 8;
static constexpr int   PENUP_STEP_MS  = 15;
// IK検証済み上限スケール: 最遠点(12,32)でd1≈118mm < 120mm
static constexpr float DRAW_SCALE     = 0.75f;

// ── 通信パラメータ ───────────────────────────────────────────
static constexpr int     WIFI_CHANNEL       = 6;
static constexpr int     JPEG_QUALITY       = 24;
static constexpr int     PKT_MAC_LEN        = 10;
static constexpr int     PKT_CTRL_LEN       = 7;
static constexpr int     PKT_SGLY_LEN       = 8;
static constexpr int     SERVO_OFFSET       = 90;
static constexpr int     MAX_SPEED          = 60;
static constexpr unsigned long CANDIDATE_TIMEOUT_MS = 8000;
static constexpr float   JOY_SPEED          = 1.5f;   // mm per loop (IKジョイスティック移動量)

static constexpr uint8_t ID_CTRL[4] = {'s','i','n','1'};
static constexpr uint8_t ID_ENQ[4]  = {'s','e','n','q'};
static constexpr uint8_t ID_ACK[4]  = {'s','a','c','k'};
static constexpr uint8_t ID_PING[4] = {'s','p','i','n'};
static constexpr uint8_t ID_PONG[4] = {'s','p','o','n'};
static constexpr uint8_t ID_SGLY[4] = {'s','g','l','y'};
static constexpr uint8_t ID_CALB[4] = {'c','a','l','b'};
static constexpr uint8_t ID_HOME[4]  = {'h','o','m','e'};
static constexpr uint8_t ID_TORCH[4] = {'t','r','c','h'};
static constexpr uint8_t ID_CLRB[4]  = {'c','l','r','b'};
static constexpr const char* TRIM_FILE = "/trim.dat";
static constexpr const char* HOME_FILE = "/home.dat";
static constexpr float       CAL_SPEED = 0.8f;  // degrees per JOY_SPEED unit in calib

// ── グリフ選択テーブル (Display の OK でサイクル、NG で確定描画) ─
static const uint32_t COMBINED_CPS[] = {
    0x30A4u, // 0: イ
    0x30EDu, // 1: ロ
    0x30CFu, // 2: ハ
    0x30A2u, // 3: ア
    0x2665u, // 4: ♥  Heart
    0x2605u, // 5: ★  Star
    0x25CBu, // 6: ○  Circle
    0x25B3u, // 7: △  Tri
    0x25A1u, // 8: □  Rect
    0x0064u, // 9: d
    0x0069u, // 10: i
    0x0067u, // 11: g
    0x006Bu, // 12: k
    0x0065u, // 13: e
    0x0079u, // 14: y
};
static constexpr uint8_t COMBINED_COUNT = 15;
static constexpr uint8_t SGLY_IROHA    = 0xFD;
static constexpr uint8_t SGLY_ICHI     = 0xFE;
static constexpr uint8_t SGLY_DIGIKEY  = 0xFF;

// ── グローバル変数 ───────────────────────────────────────────
ESPNowCam radio;
static uint8_t recvBuf[64 * 1024];
CRGB leds[NUM_LEDS];

static CRGB pen_color = CRGB::White;
static float cur_x = HOME_X, cur_y = HOME_Y;

// ペアリング
static uint8_t ctrl_mac[6]       = {};
static bool    ctrl_mac_set      = false;
static uint8_t recv_src_mac[6]   = {};
static constexpr const char* CTRL_MAC_FILE = "/ctrl.mac";

// 候補(3ウェイハンドシェイク用)
static uint8_t       candidate_mac[6]   = {};
static bool          candidate_pending  = false;
static unsigned long candidate_ms       = 0;

// 遅延返信(recvコールバックからesp_now_send禁止のため)
static volatile bool   pending_reply = false;
static uint8_t         pending_reply_pkt[PKT_MAC_LEN];
static uint8_t         pending_reply_dest[6] = {};

// プリセット描画キュー
static volatile bool    pending_preset = false;
static volatile uint8_t preset_id     = 0;
static volatile uint8_t preset_r      = 255;
static volatile uint8_t preset_g      = 255;
static volatile uint8_t preset_b      = 255;
static bool             drawing_busy  = false;

// ジョイスティックIK
static volatile float joy_dx = 0.0f, joy_dy = 0.0f;

// キャリブレーション
static bool  in_calib = false;
static float cal_a1   = 45.0f;
static float cal_a2   = 45.0f;

// ホームポジション (ダブルクリックで現在位置を上書き保存)
static float g_home_x = 18.0f;
static float g_home_y = 75.0f;

// トーチ状態 (描画終了後に復元するため保持)
static bool g_torch_on = false;

// ── ユーティリティ ───────────────────────────────────────────
static bool matchId(const uint8_t* data, const uint8_t* id) {
    return data[0]==id[0] && data[1]==id[1] && data[2]==id[2] && data[3]==id[3];
}
static bool isJpeg(const uint8_t* data) {
    return data[0]==0xFF && data[1]==0xD8;
}

// ── カメラ初期化 ─────────────────────────────────────────────
static bool initCamera() {
    pinMode(CAM_POWER_PIN, OUTPUT);
    digitalWrite(CAM_POWER_PIN, LOW);
    delay(1500);
    camera_config_t cfg = {};
    cfg.pin_pwdn      = -1;
    cfg.pin_reset     = -1;
    cfg.pin_xclk      = CAM_PIN_XCLK;
    cfg.pin_sscb_sda  = CAM_PIN_SIOD;
    cfg.pin_sscb_scl  = CAM_PIN_SIOC;
    cfg.pin_d7=CAM_PIN_Y9; cfg.pin_d6=CAM_PIN_Y8;
    cfg.pin_d5=CAM_PIN_Y7; cfg.pin_d4=CAM_PIN_Y6;
    cfg.pin_d3=CAM_PIN_Y5; cfg.pin_d2=CAM_PIN_Y4;
    cfg.pin_d1=CAM_PIN_Y3; cfg.pin_d0=CAM_PIN_Y2;
    cfg.pin_vsync     = CAM_PIN_VSYNC;
    cfg.pin_href      = CAM_PIN_HREF;
    cfg.pin_pclk      = CAM_PIN_PCLK;
    cfg.xclk_freq_hz  = 20000000;
    cfg.ledc_timer    = LEDC_TIMER_0;
    cfg.ledc_channel  = LEDC_CHANNEL_0;
    cfg.pixel_format  = PIXFORMAT_RGB565;
    cfg.frame_size    = FRAMESIZE_QQVGA;
    cfg.jpeg_quality  = 0;
    cfg.fb_count      = 2;
    cfg.fb_location   = CAMERA_FB_IN_PSRAM;
    cfg.grab_mode     = CAMERA_GRAB_LATEST;
    if (esp_camera_init(&cfg) != ESP_OK) {
        Serial.println("[CAM] init failed");
        return false;
    }
    sensor_t* s = esp_camera_sensor_get();
    if (s) { s->set_hmirror(s, 1); s->set_vflip(s, 1); }
    Serial.println("[CAM] init OK");
    return true;
}

// ── サーボ ───────────────────────────────────────────────────
static void initServo() {
    // Camera が LEDC_TIMER_0 を占有するため LEDC_TIMER_2 を直接指定
    ledc_timer_config_t tmr = {};
    tmr.speed_mode      = LEDC_LOW_SPEED_MODE;
    tmr.duty_resolution = (ledc_timer_bit_t)SERVO_BITS;
    tmr.timer_num       = LEDC_TIMER_2;
    tmr.freq_hz         = (uint32_t)SERVO_FREQ;
    tmr.clk_cfg         = LEDC_AUTO_CLK;
    ledc_timer_config(&tmr);

    ledc_channel_config_t ch1 = {};
    ch1.gpio_num   = SERVO_PIN1;
    ch1.speed_mode = LEDC_LOW_SPEED_MODE;
    ch1.channel    = (ledc_channel_t)LEDC_CH_SV1;
    ch1.timer_sel  = LEDC_TIMER_2;
    ch1.duty       = 0;
    ch1.hpoint     = 0;
    ledc_channel_config(&ch1);

    ledc_channel_config_t ch2 = {};
    ch2.gpio_num   = SERVO_PIN2;
    ch2.speed_mode = LEDC_LOW_SPEED_MODE;
    ch2.channel    = (ledc_channel_t)LEDC_CH_SV2;
    ch2.timer_sel  = LEDC_TIMER_2;
    ch2.duty       = 0;
    ch2.hpoint     = 0;
    ledc_channel_config(&ch2);

    Serial.println("[SERVO] init OK  G5/CH4  G6/CH5  LEDC_TIMER_2");
}

static void setServoDeg(int a1, int a2) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)LEDC_CH_SV1, map(a1, 0, 180, SERVO_MIN_W, SERVO_MAX_W));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)LEDC_CH_SV1);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)LEDC_CH_SV2, map(a2, 0, 180, SERVO_MIN_W, SERVO_MAX_W));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)LEDC_CH_SV2);
}

// ── ペン LED ─────────────────────────────────────────────────
static void setPen(bool down) {
    leds[0] = down ? pen_color : CRGB::Black;
    FastLED.show();
}

// ── delayWithCam: delay中もカメラフレームをストリーム送信 ────
static void delayWithCam(int ms) {
    unsigned long end = millis() + (unsigned long)ms;
    while (millis() < end) {
        camera_fb_t* fb = esp_camera_fb_get();
        if (fb) {
            uint8_t* jpg = nullptr; size_t len = 0;
            if (frame2jpg(fb, JPEG_QUALITY, &jpg, &len)) {
                radio.sendData(jpg, len);
                free(jpg);
            }
            esp_camera_fb_return(fb);
        }
    }
}

// ── 描画プリミティブ ─────────────────────────────────────────
static void moveTo(float x, float y, bool penDown, int waitMs = STEP_MS) {
    float t1, t2;
    if (!calcIK(x, y, t1, t2)) return;
    int a1 = constrain((int)(t1 * RAD_TO_DEG) + SERVO1_TRIM, 0, 180);
    int a2 = constrain((int)(t2 * RAD_TO_DEG) + SERVO2_TRIM, 0, 180);
    setServoDeg(a1, a2);
    setPen(penDown);
    cur_x = x; cur_y = y;
    delayWithCam(waitMs);
}

static void drawLine(float x0, float y0, float x1, float y1,
                     bool pen, int steps = INTERP_STEPS, int waitMs = STEP_MS) {
    for (int i = 0; i <= steps; i++) {
        float t = (float)i / steps;
        moveTo(x0 + t*(x1-x0), y0 + t*(y1-y0), pen, waitMs);
    }
}

static void goHome(int ms = 600) {
    moveTo(g_home_x, g_home_y, false, ms);
}

// ── グリフ描画 ───────────────────────────────────────────────
static int countDrawSegs(const GlyphDef& g) {
    int n = 0;
    for (uint16_t i = 0; i < g.len; ) {
        int8_t bx = (int8_t)pgm_read_byte(&g.data[i]);
        if (bx == GLYPH_PENUP) { i += 3; } else { n++; i += 2; }
    }
    return max(1, n);
}

static void drawGlyph(const GlyphDef& g) {
    int segs  = countDrawSegs(g);
    int stepMs = max(MIN_STEP_MS, TARGET_DRAW_MS / (segs * INTERP_STEPS));

    float px = g_home_x, py = g_home_y;
    uint16_t i = 0;
    while (i < g.len) {
        int8_t bx = (int8_t)pgm_read_byte(&g.data[i]);
        if (bx == GLYPH_PENUP) {
            i++;
            if (i+1 >= g.len) break;
            int8_t gx = (int8_t)pgm_read_byte(&g.data[i++]);
            int8_t gy = (int8_t)pgm_read_byte(&g.data[i++]);
            float wx = g_home_x + gx * DRAW_SCALE;
            float wy = g_home_y + gy * DRAW_SCALE;
            drawLine(px, py, wx, wy, false, INTERP_STEPS, PENUP_STEP_MS);
            px = wx; py = wy;
        } else {
            i++;
            if (i >= g.len) break;
            int8_t gy = (int8_t)pgm_read_byte(&g.data[i++]);
            float wx = g_home_x + bx * DRAW_SCALE;
            float wy = g_home_y + gy * DRAW_SCALE;
            drawLine(px, py, wx, wy, true, INTERP_STEPS, stepMs);
            px = wx; py = wy;
        }
    }
    setPen(false);
}

static const GlyphDef* findGlyph(const GlyphDef* table, int sz, uint32_t cp) {
    for (int i = 0; i < sz; i++)
        if (pgm_read_dword(&table[i].codepoint) == cp) return &table[i];
    return nullptr;
}

static const GlyphDef* findAnyGlyph(uint32_t cp) {
    const GlyphDef* g = findGlyph(lumigraph_glyph_table, LUMIGRAPH_GLYPH_COUNT, cp);
    if (!g) g = findGlyph(sample_glyph_table, SAMPLE_GLYPH_COUNT, cp);
    return g;
}

// ── プリセット実行 ───────────────────────────────────────────
static void execPreset(uint8_t idx, uint8_t r, uint8_t g, uint8_t b) {
    pen_color = CRGB(r, g, b);
    if (idx == SGLY_DIGIKEY) {
        // "digikey" の7文字を順に描画
        static const uint32_t DIGIKEY_SEQ[] = {
            0x0044u, 0x0069u, 0x0067u, 0x0069u, 0x004Bu, 0x0065u, 0x0079u
        };
        static constexpr uint8_t DIGIKEY_CNT = 7;
        for (uint8_t j = 0; j < DIGIKEY_CNT; j++) {
            const GlyphDef* gd = findAnyGlyph(DIGIKEY_SEQ[j]);
            if (!gd) continue;
            // 前の文字を表示したまま待機してからバッファクリア
            goHome(150);
            delay(600);
            if (ctrl_mac_set) {
                uint8_t clrb[4]; memcpy(clrb, ID_CLRB, 4);
                radio.sendData(clrb, 4);
                delay(200);  // Display側のクリア + 転送中フレーム排出待ち
            }
            drawGlyph(*gd);
        }
        goHome(150);
    } else if (idx == SGLY_IROHA || idx == SGLY_ICHI) {
        // イロハ or イチ のシーケンス描画
        static const uint32_t IROHA_SEQ[] = {0x30A4u, 0x30EDu, 0x30CFu};
        static const uint32_t ICHI_SEQ[]  = {0x30A4u, 0x30C1u};
        const uint32_t* seq = (idx == SGLY_IROHA) ? IROHA_SEQ : ICHI_SEQ;
        const uint8_t   cnt = (idx == SGLY_IROHA) ? 3 : 2;
        for (uint8_t j = 0; j < cnt; j++) {
            const GlyphDef* gd = findAnyGlyph(seq[j]);
            if (!gd) continue;
            goHome(150);
            delay(600);
            if (ctrl_mac_set) {
                uint8_t clrb[4]; memcpy(clrb, ID_CLRB, 4);
                radio.sendData(clrb, 4);
                delay(200);
            }
            drawGlyph(*gd);
        }
        goHome(150);
    } else if (idx < COMBINED_COUNT) {
        const GlyphDef* gd = findAnyGlyph(COMBINED_CPS[idx]);
        if (gd) { goHome(150); drawGlyph(*gd); goHome(150); }
    }
    setPen(false);
}

// ── SPIFFS: ペアリングMAC永続化 ──────────────────────────────
static void loadCtrlMac() {
    if (!SPIFFS.exists(CTRL_MAC_FILE)) return;
    File f = SPIFFS.open(CTRL_MAC_FILE, FILE_READ);
    if (!f || f.size() < 6) { if (f) f.close(); return; }
    for (int i = 0; i < 6; i++) ctrl_mac[i] = (uint8_t)f.read();
    f.close();
    ctrl_mac_set = true;
    radio.setTarget(ctrl_mac);
    Serial.printf("[CTRL_MAC] %02X:%02X:%02X:%02X:%02X:%02X\n",
        ctrl_mac[0],ctrl_mac[1],ctrl_mac[2],ctrl_mac[3],ctrl_mac[4],ctrl_mac[5]);
}

static void saveCtrlMac(const uint8_t* mac) {
    memcpy(ctrl_mac, mac, 6);
    ctrl_mac_set = true;
    radio.setTarget(ctrl_mac);
    File f = SPIFFS.open(CTRL_MAC_FILE, FILE_WRITE);
    if (!f) return;
    for (int i = 0; i < 6; i++) f.write(ctrl_mac[i]);
    f.close();
    Serial.println("[CTRL_MAC] saved");
}

static void loadTrim() {
    if (!SPIFFS.exists(TRIM_FILE)) return;
    File f = SPIFFS.open(TRIM_FILE, FILE_READ);
    if (!f || f.size() < 2) { if (f) f.close(); return; }
    SERVO1_TRIM = (int)(int8_t)f.read();
    SERVO2_TRIM = (int)(int8_t)f.read();
    f.close();
    Serial.printf("[TRIM] loaded %d %d\n", SERVO1_TRIM, SERVO2_TRIM);
}

static void saveTrim() {
    File f = SPIFFS.open(TRIM_FILE, FILE_WRITE);
    if (!f) return;
    f.write((uint8_t)(int8_t)SERVO1_TRIM);
    f.write((uint8_t)(int8_t)SERVO2_TRIM);
    f.close();
    Serial.printf("[TRIM] saved %d %d\n", SERVO1_TRIM, SERVO2_TRIM);
}

static void loadHome() {
    if (!SPIFFS.exists(HOME_FILE)) return;
    File f = SPIFFS.open(HOME_FILE, FILE_READ);
    if (!f || f.size() < 8) { if (f) f.close(); return; }
    f.read((uint8_t*)&g_home_x, 4);
    f.read((uint8_t*)&g_home_y, 4);
    f.close();
    Serial.printf("[HOME] loaded (%.1f, %.1f)\n", g_home_x, g_home_y);
}
static void saveHome() {
    File f = SPIFFS.open(HOME_FILE, FILE_WRITE);
    if (!f) return;
    f.write((uint8_t*)&g_home_x, 4);
    f.write((uint8_t*)&g_home_y, 4);
    f.close();
    Serial.printf("[HOME] saved (%.1f, %.1f)\n", g_home_x, g_home_y);
}

// ── 受信コールバック ─────────────────────────────────────────
static void onDataRecv(uint32_t length) {
    if (length < 4) return;
    if (isJpeg(recvBuf)) return;

    uint8_t myMac[6];
    esp_read_mac(myMac, ESP_MAC_WIFI_STA);

    // ENQ: ペアリング状態に関係なく常に応答 (再ペアリング対応)
    if (matchId(recvBuf, ID_ENQ) && length >= (uint32_t)PKT_MAC_LEN) {
        if (!ctrl_mac_set) {
            memcpy(candidate_mac, recv_src_mac, 6);
            candidate_pending = true;
            candidate_ms = millis();
            Serial.printf("[PAIR] ENQ candidate %02X:%02X:%02X:%02X:%02X:%02X\n",
                recv_src_mac[0],recv_src_mac[1],recv_src_mac[2],
                recv_src_mac[3],recv_src_mac[4],recv_src_mac[5]);
        }
        memcpy(pending_reply_dest,    recv_src_mac, 6);  // 返信先 = 送信元
        memcpy(pending_reply_pkt,     ID_ACK, 4);
        memcpy(pending_reply_pkt + 4, myMac,  6);
        pending_reply = true;
        return;
    }

    // ペアリング済みなら登録外の送信元を全拒否
    if (ctrl_mac_set && memcmp(recv_src_mac, ctrl_mac, 6) != 0) return;

    // PING → PONG deferred送信
    if (matchId(recvBuf, ID_PING) && length >= (uint32_t)PKT_MAC_LEN) {
        memcpy(pending_reply_dest,    recv_src_mac, 6);  // 返信先 = 送信元
        memcpy(pending_reply_pkt,     ID_PONG, 4);
        memcpy(pending_reply_pkt + 4, myMac,   6);
        pending_reply = true;
        return;
    }

    // 制御パケット → ctrl_mac確定 + ジョイスティック値更新
    if (matchId(recvBuf, ID_CTRL) && length >= (uint32_t)PKT_CTRL_LEN) {
        if (!ctrl_mac_set) {
            if (!candidate_pending || memcmp(recv_src_mac, candidate_mac, 6) != 0) return;
            saveCtrlMac(candidate_mac);
            candidate_pending = false;
            Serial.println("[PAIR] ctrl_mac confirmed");
        }
        int lv = (int)recvBuf[4] - SERVO_OFFSET;
        int rv = (int)recvBuf[5] - SERVO_OFFSET;
        joy_dx = rv / (float)MAX_SPEED * JOY_SPEED;
        joy_dy = lv / (float)MAX_SPEED * JOY_SPEED;
        return;
    }

    // グリフプリセット "sgly"[preset][r][g][b]
    if (matchId(recvBuf, ID_SGLY) && length >= (uint32_t)PKT_SGLY_LEN) {
        preset_id = recvBuf[4];
        preset_r  = recvBuf[5];
        preset_g  = recvBuf[6];
        preset_b  = recvBuf[7];
        pending_preset = true;
        Serial.printf("[SGLY] preset=%d rgb(%d,%d,%d)\n",
            preset_id, preset_r, preset_g, preset_b);
        return;
    }

    // キャリブレーション "calb": 1回目=開始(45°/45°), 2回目=保存して終了
    if (matchId(recvBuf, ID_CALB) && !drawing_busy) {
        if (!in_calib) {
            in_calib = true;
            cal_a1 = 45.0f; cal_a2 = 45.0f;
            setServoDeg(45, 45);
            Serial.println("[CALIB] enter  a1=45 a2=45");
        } else {
            SERVO1_TRIM = (int)roundf(cal_a1) - 45;
            SERVO2_TRIM = (int)roundf(cal_a2) - 45;
            saveTrim();
            in_calib = false;
            Serial.printf("[CALIB] done  trim1=%d trim2=%d\n", SERVO1_TRIM, SERVO2_TRIM);
        }
        return;
    }

    // ホームポジション設定 "home": 現在位置を描画中心として保存
    if (matchId(recvBuf, ID_HOME) && !drawing_busy) {
        g_home_x = cur_x;
        g_home_y = cur_y;
        saveHome();
        Serial.printf("[HOME] set (%.1f, %.1f)\n", g_home_x, g_home_y);
        return;
    }

    // トーチ制御 "trch": LED白点灯/消灯
    if (matchId(recvBuf, ID_TORCH)) {
        g_torch_on = (length > 4) && (recvBuf[4] != 0);
        if (!drawing_busy) {
            if (g_torch_on) {
                leds[0] = CRGB::White;
                FastLED.setBrightness(16);
            } else {
                leds[0] = CRGB::Black;
                FastLED.setBrightness(4);
            }
            FastLED.show();
        }
        return;
    }
}

// ════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("=== LumiGraph Drawing ===");
    SPIFFS.begin(true);

    if (!initCamera()) { while (true) delay(1000); }

    // ESPNowCam: radio.setRecvCallback は ROLE_ROBOT と同様、
    // 明示的な esp_now_register_recv_cb で上書きして recv_src_mac を捕捉する
    radio.setRecvBuffer(recvBuf);
    radio.setRecvCallback(onDataRecv);
    radio.setChannel(WIFI_CHANNEL);
    radio.init();

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 1)
    esp_now_register_recv_cb(
        [](const esp_now_recv_info_t* info, const uint8_t* data, int len) {
            if (info) memcpy(recv_src_mac, info->src_addr, 6);
#else
    esp_now_register_recv_cb(
        [](const uint8_t* src, const uint8_t* data, int len) {
            if (src) memcpy(recv_src_mac, src, 6);
#endif
            // ESPNowCam protobufヘッダ [0x08][n][0x12][n][payload] を除去して転送
            if (len >= 5 && data[0]==0x08 && data[2]==0x12 && data[1]==data[3]) {
                uint8_t n = data[3];
                if (len == (int)(4+n) && (size_t)n <= sizeof(recvBuf)) {
                    memcpy(recvBuf, data+4, n);
                    onDataRecv((uint32_t)n);
                }
            }
        }
    );
    Serial.println("[RADIO] init OK");

    loadCtrlMac();
    loadTrim();
    loadHome();
    // FastLED を initServo より先に初期化: RMT の GPIO マトリクス設定を
    // LEDC が後から上書きするため競合しない
    FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(4);
    initServo();
    setPen(false);

    goHome(2500);
    Serial.println("Ready");
}

void loop() {
    // deferred ACK/PONG送信: 返信先MACに一時的にターゲットを向けて送信
    if (pending_reply) {
        pending_reply = false;
        radio.setTarget(pending_reply_dest);
        radio.sendData(pending_reply_pkt, PKT_MAC_LEN);
        if (ctrl_mac_set) radio.setTarget(ctrl_mac);
    }

    // 候補タイムアウト
    if (candidate_pending && millis() - candidate_ms > CANDIDATE_TIMEOUT_MS) {
        Serial.println("[PAIR] candidate expired");
        candidate_pending = false;
    }

    // プリセット描画 (drawing_busy 中は次のプリセットを受け付けない)
    if (pending_preset && !drawing_busy) {
        pending_preset = false;
        drawing_busy = true;
        execPreset(preset_id, preset_r, preset_g, preset_b);
        drawing_busy = false;
        // 描画後にトーチ状態を復元
        if (g_torch_on) {
            leds[0] = CRGB::White;
            FastLED.setBrightness(16);
            FastLED.show();
        } else {
            FastLED.setBrightness(4);
        }
    }

    unsigned long now = millis();

    // ジョイスティック / キャリブレーション: 20ms毎に更新 (制御パケット周期と同期)
    static unsigned long last_joy_ms  = 0;
    static unsigned long last_print_ms = 0;
    if (now - last_joy_ms >= 20) {
        last_joy_ms = now;

        if (in_calib) {
            if (fabsf(joy_dx) > 0.01f || fabsf(joy_dy) > 0.01f) {
                cal_a2 = constrain(cal_a2 + joy_dx / JOY_SPEED * CAL_SPEED, 0.0f, 180.0f);
                cal_a1 = constrain(cal_a1 + joy_dy / JOY_SPEED * CAL_SPEED, 0.0f, 180.0f);
                setServoDeg((int)cal_a1, (int)cal_a2);
                if (now - last_print_ms >= 250) {
                    last_print_ms = now;
                    Serial.printf("[CALIB] a1=%.0f a2=%.0f\n", cal_a1, cal_a2);
                }
            }
        }
        // 通常ジョイスティックIK (描画中は無効)
        else if (!drawing_busy && (fabsf(joy_dx) > 0.01f || fabsf(joy_dy) > 0.01f)) {
            float nx = constrain(cur_x + joy_dx,  2.0f, 34.0f);
            float ny = constrain(cur_y + joy_dy, 40.0f, 110.0f);
            float t1, t2;
            if (calcIK(nx, ny, t1, t2)) {
                int a1 = constrain((int)(t1*RAD_TO_DEG)+SERVO1_TRIM, 0, 180);
                int a2 = constrain((int)(t2*RAD_TO_DEG)+SERVO2_TRIM, 0, 180);
                setServoDeg(a1, a2);
                cur_x = nx; cur_y = ny;
            }
        }
    }

    // カメラフレーム送信: 100ms毎 (描画中は delayWithCam が担当)
    static unsigned long last_cam_ms = 0;
    if (!drawing_busy && now - last_cam_ms >= 100) {
        camera_fb_t* fb = esp_camera_fb_get();
        if (fb) {
            uint8_t* jpg = nullptr; size_t len = 0;
            if (frame2jpg(fb, JPEG_QUALITY, &jpg, &len)) {
                radio.sendData(jpg, len);
                free(jpg);
            }
            esp_camera_fb_return(fb);
            last_cam_ms = millis();
        }
    }
}

// ════════════════════════════════════════════════════════════════
//  ROLE_DISPLAY : AtomS3R — 映像受信表示 + ボタンでグリフ指示
// ════════════════════════════════════════════════════════════════
#elif defined(ROLE_DISPLAY)

#include <M5Unified.h>
#include <ESPNowCam.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_timer.h>
#include "FS.h"
#include "SPIFFS.h"

// ── 定数 ──────────────────────────────────────────────────────
static constexpr int     WIFI_CHANNEL       = 6;
static constexpr int     LCD_W              = 128;
static constexpr int     LCD_H              = 128;
static constexpr float   ADC_MAX_VALUE      = 4095.0f;
static constexpr float   DEADBAND           = 0.1f;
static constexpr int     LOG_SIZE           = 5;
static constexpr int     CTRL_INTERVAL_US   = 20 * 1000;
static constexpr unsigned long FRAME_TIMEOUT_MS = 2000;
static constexpr int     MAC_LIST_MAX       = 5;
static constexpr int     PING_TIMEOUT_MS    = 800;
static constexpr int     MAX_SPEED          = 60;
static constexpr int     SERVO_OFFSET       = 90;
static constexpr int     PKT_MAC_LEN        = 10;
static constexpr int     PKT_CTRL_LEN       = 7;
static constexpr int     PKT_SGLY_LEN       = 8;

static constexpr uint8_t ID_CTRL[4] = {'s','i','n','1'};
static constexpr uint8_t ID_ENQ[4]  = {'s','e','n','q'};
static constexpr uint8_t ID_ACK[4]  = {'s','a','c','k'};
static constexpr uint8_t ID_PING[4] = {'s','p','i','n'};
static constexpr uint8_t ID_PONG[4] = {'s','p','o','n'};
static constexpr uint8_t ID_SGLY[4] = {'s','g','l','y'};
static constexpr uint8_t ID_CALB[4] = {'c','a','l','b'};
static constexpr uint8_t ID_HOME[4]  = {'h','o','m','e'};
static constexpr uint8_t ID_TORCH[4] = {'t','r','c','h'};
static constexpr uint8_t ID_CLRB[4]  = {'c','l','r','b'};

static const char* MAC_FILE = "/mac.txt";

// ── グリフ選択テーブル (OK でサイクル、NG で確定描画) ────────
static const char* COMBINED_LABELS[] = {
    "I","Ro","Ha","IRoHa","IChi","A",
    "Heart","Star","Circle","Tri","Rect",
    "d","i","g","k","e","y",
    "DigiKey"
};
// 各グリフの固有色 (順序は COMBINED_LABELS と対応)
static const uint8_t COMBINED_COLORS[][3] = {
    {255,255,255},  // I      白
    {  0,220,220},  // Ro     シアン
    {255,  0,200},  // Ha     マゼンタ
    {255,255,255},  // IRoHa  白
    {255,220,  0},  // IChi   ゴールド
    {150,  0,255},  // A      インディゴ
    {255, 60,150},  // Heart  ピンク
    {255,210,  0},  // Star   イエロー
    {  0,150,255},  // Circle ブルー
    {  0,200,  0},  // Tri    グリーン
    {200,200,200},  // Rect   シルバー
    {255,  0,  0},  // d      レッド
    {  0, 80,255},  // i      ディープブルー
    {150,255,  0},  // g      ライムグリーン
    { 80,  0,255},  // k      バイオレット
    {255,180,  0},  // e      アンバー
    {  0,200,150},  // y      ティール
    {255, 80,  0},  // DigiKey オレンジ
};
// メニュー選択 → Drawing 側 idx マッピング
static const uint8_t COMBINED_SEND_IDX[] = {
    0, 1, 2,          // イ, ロ, ハ
    0xFD, 0xFE,       // IRoHa, IChi
    3, 4, 5, 6, 7, 8, // ア,♥,★,○,△,□
    9,10,11,12,13,14, // d,i,g,k,e,y
    0xFF,             // DigiKey
};
static constexpr uint8_t COMBINED_COUNT = 18;
static constexpr uint8_t SGLY_IROHA    = 0xFD;
static constexpr uint8_t SGLY_ICHI     = 0xFE;
static constexpr uint8_t SGLY_DIGIKEY  = 0xFF;
static uint8_t glyph_sel = 0;

// ── ピン定義 ──────────────────────────────────────────────────
static constexpr int ADC_H_PIN  =  8;
static constexpr int ADC_V_PIN  =  7;
static constexpr int TRG_SW_PIN =  5;
static constexpr int OK_SW_PIN  = 38;
static constexpr int NG_SW_PIN  = 39;

// ── グローバル変数 ───────────────────────────────────────────
ESPNowCam radio;
static uint8_t recvBuf[64 * 1024];

static volatile bool          frameReady    = false;
static volatile uint32_t      frameLen      = 0;
static volatile unsigned long last_frame_ms = 0;
static bool                   was_live      = false;

// 長時間露光バッファ (各ピクセルの最大輝度を保持)
static LGFX_Sprite* g_longExpSprite = nullptr;
static uint16_t     g_longExpBuf[LCD_W * LCD_H] = {};
static bool         g_longExpMode   = false;

// Drawing→Displayのバッファクリア要求フラグ
static volatile bool g_clrb_pending = false;

// ── 録画/再生 ─────────────────────────────────────────────────
struct DispRecFrame { int8_t jh; int8_t jv; uint8_t torch; };
static constexpr int REC_MAX_FRAMES = 250;   // 5秒 @ 20ms
static DispRecFrame  g_rec[REC_MAX_FRAMES];
static int           g_rec_len   = 0;
static bool          g_recording = false;
static uint8_t       g_rec_torch = 0;
static int           g_play_idx  = 0;
static bool          g_playing   = false;

// ── メニュー ──────────────────────────────────────────────────
static bool g_in_menu  = false;
static int  g_menu_sel = 0;
static constexpr int MENU_COLS      = 4;
static constexpr int MENU_GLYPH_CNT = 18;  // 0..17 = glyphs + DigiKey
static constexpr int MENU_REC_IDX   = 18;
static constexpr int MENU_PLAY_IDX  = 19;
static constexpr int MENU_TOTAL     = 20;

// ADC移動平均
static float h_log[LOG_SIZE] = {}, v_log[LOG_SIZE] = {};
static int   log_cnt = 0;
static float joy_h = 0.0f, joy_v = 0.0f;

// MAC履歴
static uint8_t macList[MAC_LIST_MAX][6];
static int     macCount  = 0;
static bool    is_paired = false;
static uint8_t target_addr[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

static volatile bool pong_received = false;
static volatile bool ack_received  = false;
static uint8_t response_mac[6] = {};

// ステータス表示
static char last_status[32] = "";

// ── ユーティリティ ───────────────────────────────────────────
static bool matchId(const uint8_t* data, const uint8_t* id) {
    return data[0]==id[0] && data[1]==id[1] && data[2]==id[2] && data[3]==id[3];
}
static bool isJpeg(const uint8_t* data) {
    return data[0]==0xFF && data[1]==0xD8;
}
static float deadbanded(float v, float b) {
    if (v > -b && v < b) return 0.0f;
    return v > 0 ? v-b : v+b;
}

static void setStatus(const char* msg) {
    strncpy(last_status, msg, sizeof(last_status)-1);
    last_status[sizeof(last_status)-1] = '\0';
}

static void drawStatusBar() {
    M5.Display.fillRect(0, LCD_H-10, LCD_W, 10, TFT_BLACK);
    M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Display.setTextSize(1);
    M5.Display.setCursor(2, LCD_H-9);
    M5.Display.print(last_status);
}

// ── 受信コールバック (ESPNowCam内部が protobuf ヘッダを除去して呼ぶ) ──
static void onDataRecv(uint32_t length) {
    if (length < 2) return;
    if (isJpeg(recvBuf)) {
        frameLen = length; frameReady = true; last_frame_ms = millis(); return;
    }
    if (length < 4) return;
    if (matchId(recvBuf, ID_ACK) && length >= (uint32_t)PKT_MAC_LEN) {
        memcpy(response_mac, recvBuf+4, 6); ack_received = true; return;
    }
    if (matchId(recvBuf, ID_PONG) && length >= (uint32_t)PKT_MAC_LEN) {
        memcpy(response_mac, recvBuf+4, 6); pong_received = true; return;
    }
    if (matchId(recvBuf, ID_CLRB)) {
        g_clrb_pending = true; return;
    }
}

// ── SPIFFS: MAC履歴 ──────────────────────────────────────────
static void loadMacList() {
    macCount = 0;
    if (!SPIFFS.exists(MAC_FILE)) return;
    File f = SPIFFS.open(MAC_FILE, FILE_READ);
    if (!f) return;
    int cnt = min((int)(f.size()/6), MAC_LIST_MAX);
    for (int i = 0; i < cnt; i++)
        for (int j = 0; j < 6; j++) macList[i][j] = (uint8_t)f.read();
    f.close(); macCount = cnt;
    Serial.printf("[MAC] loaded %d entries\n", macCount);
}

static void saveMacList() {
    File f = SPIFFS.open(MAC_FILE, FILE_WRITE);
    if (!f) return;
    for (int i = 0; i < macCount; i++)
        for (int j = 0; j < 6; j++) f.write(macList[i][j]);
    f.close();
}

static void prependMac(const uint8_t* mac) {
    uint8_t tmp[MAC_LIST_MAX][6]; int nc = 0;
    for (int i = 0; i < macCount; i++)
        if (memcmp(macList[i], mac, 6) != 0 && nc < MAC_LIST_MAX-1)
            memcpy(tmp[nc++], macList[i], 6);
    memcpy(macList[0], mac, 6);
    for (int i = 0; i < nc; i++) memcpy(macList[i+1], tmp[i], 6);
    macCount = nc+1;
    saveMacList();
}

// ── 疎通確認 ─────────────────────────────────────────────────
static bool pingMac(const uint8_t* mac) {
    uint8_t myMac[6]; esp_read_mac(myMac, ESP_MAC_WIFI_STA);
    uint8_t pkt[PKT_MAC_LEN];
    memcpy(pkt, ID_PING, 4); memcpy(pkt+4, myMac, 6);
    pong_received = false;
    radio.sendData(pkt, PKT_MAC_LEN);
    unsigned long t = millis();
    while (millis()-t < (unsigned long)PING_TIMEOUT_MS) {
        if (pong_received && memcmp(response_mac, mac, 6)==0) return true;
        if (pong_received) pong_received = false;
        delay(5);
    }
    return false;
}

static bool resolveMac() {
    for (int i = 0; i < macCount; i++) {
        Serial.printf("[MAC] ping %02X:%02X:%02X:%02X:%02X:%02X\n",
            macList[i][0],macList[i][1],macList[i][2],
            macList[i][3],macList[i][4],macList[i][5]);
        radio.setTarget(macList[i]);  // PING前にターゲット設定
        if (pingMac(macList[i])) {
            Serial.println("[MAC] pong OK");
            prependMac(macList[i]);
            memcpy(target_addr, macList[i], 6);
            return true;
        }
    }
    Serial.println("[MAC] all failed -> delete");
    SPIFFS.remove(MAC_FILE); macCount = 0; return false;
}

// ── ペアリング ───────────────────────────────────────────────
static void doPairing() {
    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setTextColor(TFT_YELLOW, TFT_BLACK);
    M5.Display.setTextSize(1);
    M5.Display.setCursor(2, 2);
    M5.Display.println("Pairing...");

    uint8_t myMac[6]; esp_read_mac(myMac, ESP_MAC_WIFI_STA);
    uint8_t pkt[PKT_MAC_LEN];
    memcpy(pkt, ID_ENQ, 4); memcpy(pkt+4, myMac, 6);
    ack_received = false;
    uint8_t bcast[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    radio.setTarget(bcast);  // ENQはブロードキャスト
    radio.sendData(pkt, PKT_MAC_LEN);

    unsigned long t = millis();
    while (millis()-t < 3000) {
        if (ack_received) {
            prependMac(response_mac);
            memcpy(target_addr, response_mac, 6);
            radio.setTarget(target_addr);
            is_paired = true;
            M5.Display.fillScreen(TFT_GREEN);
            M5.Display.setTextColor(TFT_BLACK, TFT_GREEN);
            M5.Display.setTextSize(2);
            M5.Display.setCursor(8, 50);
            M5.Display.println("PAIRED!");
            delay(1000);
            esp_restart();
            return;
        }
        delay(10);
    }
    setStatus("PAIR FAIL");
    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setTextColor(TFT_RED, TFT_BLACK);
    M5.Display.setCursor(2, 2);
    M5.Display.println("PAIR FAIL");
    delay(1000);
    M5.Display.fillScreen(TFT_BLACK);
    drawStatusBar();
}

// ── home パケット送信 ────────────────────────────────────────
static void sendSetHome() {
    uint8_t pkt[4];
    memcpy(pkt, ID_HOME, 4);
    radio.sendData(pkt, 4);
    setStatus("Home set!"); drawStatusBar();
    Serial.println("[HOME] sent");
}

// ── torch パケット送信 ───────────────────────────────────────
static void sendTorch(bool on) {
    uint8_t pkt[5];
    memcpy(pkt, ID_TORCH, 4);
    pkt[4] = on ? 1 : 0;
    radio.sendData(pkt, 5);
    g_rec_torch = on ? 1 : 0;  // 録画中のトーチ状態を更新
    Serial.printf("[TORCH] %s\n", on ? "ON" : "OFF");
}

// ── sgly パケット送信 ────────────────────────────────────────
static void sendSgly(uint8_t idx, uint8_t r, uint8_t g, uint8_t b, const char* label = nullptr) {
    uint8_t pkt[PKT_SGLY_LEN] = {
        ID_SGLY[0],ID_SGLY[1],ID_SGLY[2],ID_SGLY[3], idx, r, g, b
    };
    radio.sendData(pkt, PKT_SGLY_LEN);
    char buf[32];
    snprintf(buf, sizeof(buf), ">>> %s", label ? label : "?");
    setStatus(buf); drawStatusBar();
    Serial.printf("[SGLY] sent idx=%d\n", idx);
}

// ── メニュー描画 ─────────────────────────────────────────────
static void drawMenu() {
    M5.Display.fillScreen(TFT_BLACK);
    const int cw = LCD_W / MENU_COLS;  // 32px
    const int ch = 14;                  // セル高 (文字8px + 余白)
    for (int i = 0; i < MENU_TOTAL; i++) {
        int col = i % MENU_COLS, row = i / MENU_COLS;
        int x = col * cw, y = row * ch;
        bool sel = (i == g_menu_sel);
        const char* label;
        uint8_t fr, fg2, fb;
        if (i < MENU_GLYPH_CNT) {
            label = COMBINED_LABELS[i];
            fr = COMBINED_COLORS[i][0]; fg2 = COMBINED_COLORS[i][1]; fb = COMBINED_COLORS[i][2];
        } else if (i == MENU_REC_IDX) {
            label = g_recording ? "REC*" : "REC";
            fr = 255; fg2 = 40; fb = 40;
        } else {
            label = (g_rec_len > 0) ? "PLAY" : "(no)";
            fr = (g_rec_len > 0) ? 0 : 64; fg2 = (g_rec_len > 0) ? 255 : 64; fb = (g_rec_len > 0) ? 80 : 64;
        }
        if (sel) {
            M5.Display.fillRect(x, y, cw, ch-1, TFT_WHITE);
            M5.Display.setTextColor(TFT_BLACK, TFT_WHITE);
        } else {
            M5.Display.fillRect(x, y, cw, ch-1, TFT_BLACK);
            M5.Display.setTextColor(M5.Display.color888(fr, fg2, fb), TFT_BLACK);
        }
        M5.Display.setTextSize(1);
        M5.Display.setCursor(x+1, y+3);
        M5.Display.print(label);
    }
    char buf[32];
    snprintf(buf, sizeof(buf), "Red:Exit  Black:Draw%s", g_recording?" REC*":"");
    setStatus(buf); drawStatusBar();
}

// ── ADC + 制御パケット (esp_timer で20ms毎に呼ばれる) ────────
static void readADC() {
    h_log[log_cnt] = 2.0f * analogRead(ADC_H_PIN) / ADC_MAX_VALUE - 1.0f;
    v_log[log_cnt] = 2.0f * analogRead(ADC_V_PIN) / ADC_MAX_VALUE - 1.0f;
    if (++log_cnt >= LOG_SIZE) log_cnt = 0;
    float sh=0, sv=0;
    for (int i=0; i<LOG_SIZE; i++) { sh+=h_log[i]; sv+=v_log[i]; }
    joy_h = deadbanded(sh/LOG_SIZE, DEADBAND);
    joy_v = deadbanded(sv/LOG_SIZE, DEADBAND);
}

static void sendCtrlPacket() {
    if (g_in_menu || g_playing) return;  // メニュー/再生中はタイマー送信を抑制
    int xv = constrain((int)(MAX_SPEED * joy_h), -90, 90);
    int yv = constrain((int)(MAX_SPEED * joy_v), -90, 90);
    uint8_t pkt[PKT_CTRL_LEN] = {
        ID_CTRL[0],ID_CTRL[1],ID_CTRL[2],ID_CTRL[3],
        (uint8_t)(yv + SERVO_OFFSET),
        (uint8_t)(xv + SERVO_OFFSET),
        0
    };
    radio.sendData(pkt, PKT_CTRL_LEN);
    // 録画中: ジョイスティック状態を記録
    if (g_recording && g_rec_len < REC_MAX_FRAMES) {
        g_rec[g_rec_len++] = {
            (int8_t)constrain((int)(joy_h * 127), -127, 127),
            (int8_t)constrain((int)(joy_v * 127), -127, 127),
            g_rec_torch
        };
    }
}

static void ctrlTimerCb(void*) { readADC(); sendCtrlPacket(); }

// ── ボタンエッジ検出 (50ms デバウンス付き) ──────────────────
static bool         ok_prev=false,  ng_prev=false,  trg_prev=false;
static unsigned long ok_ms=0,        ng_ms=0,         trg_ms=0;
static constexpr unsigned long DEBOUNCE_MS = 50;

static bool readEdge(int pin, bool& prev, unsigned long& lastMs) {
    bool cur = !digitalRead(pin);
    unsigned long now = millis();
    bool edge = cur && !prev && (now - lastMs >= DEBOUNCE_MS);
    if (cur && !prev) lastMs = now;
    prev = cur;
    return edge;
}

// ════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    auto cfg = M5.config();
    M5.begin(cfg);
    M5.Display.setRotation(2);  // コントローラ上下反転取り付け補正
    Serial.println("=== LumiGraph Display ===");
    SPIFFS.begin(true);

    radio.setRecvBuffer(recvBuf);
    radio.setRecvCallback(onDataRecv);
    radio.setChannel(WIFI_CHANNEL);
    radio.init();
    Serial.println("[RADIO] init OK");

    pinMode(ADC_H_PIN,  ANALOG);
    pinMode(ADC_V_PIN,  ANALOG);
    pinMode(TRG_SW_PIN, INPUT_PULLUP);
    pinMode(OK_SW_PIN,  INPUT_PULLUP);
    pinMode(NG_SW_PIN,  INPUT_PULLUP);

    loadMacList();
    if (macCount > 0) is_paired = resolveMac();

    if (is_paired) {
        radio.setTarget(target_addr);
        setStatus("PAIRED");
    } else {
        setStatus("NO PAIR  HoldA:pair");
    }

    // 制御パケット送信タイマー (20ms周期)
    esp_timer_handle_t t;
    esp_timer_create_args_t ta = {};
    ta.callback = ctrlTimerCb;
    ta.name     = "ctrl";
    esp_timer_create(&ta, &t);
    esp_timer_start_periodic(t, CTRL_INTERVAL_US);

    // 長時間露光スプライト初期化
    g_longExpSprite = new LGFX_Sprite(&M5.Display);
    g_longExpSprite->setColorDepth(16);
    g_longExpSprite->createSprite(LCD_W, LCD_H);

    M5.Display.fillScreen(TFT_BLACK);
    drawStatusBar();
    Serial.println("Ready");
}

void loop() {
    M5.update();

    unsigned long now = millis();

    // BtnA 長押し → ペアリング
    if (M5.BtnA.wasHold()) { doPairing(); }
    // BtnA ダブルタップ → ホームポジション設定
    if (M5.BtnA.wasDoubleClicked()) { sendSetHome(); }

    // Red(OK) → メニュー Open/Close、録画・再生停止
    if (readEdge(OK_SW_PIN, ok_prev, ok_ms)) {
        if (g_playing) { g_playing = false; sendTorch(false); }
        if (g_recording) { g_recording = false; }
        g_in_menu = !g_in_menu;
        if (g_in_menu) {
            drawMenu();
        } else {
            setStatus("Red:Sel  Black:Draw");
            M5.Display.fillScreen(TFT_BLACK);
            drawStatusBar();
        }
    }

    if (g_in_menu) {
        // ── メニューモード ───────────────────────────────────────
        static unsigned long last_nav_ms = 0;
        if (now - last_nav_ms >= 180) {
            bool moved = false;
            if      (joy_h < -0.3f && g_menu_sel % MENU_COLS < MENU_COLS-1)
                { g_menu_sel++; moved = true; }
            else if (joy_h >  0.3f && g_menu_sel % MENU_COLS > 0)
                { g_menu_sel--; moved = true; }
            else if (joy_v >  0.3f && g_menu_sel + MENU_COLS < MENU_TOTAL)
                { g_menu_sel += MENU_COLS; moved = true; }
            else if (joy_v < -0.3f && g_menu_sel - MENU_COLS >= 0)
                { g_menu_sel -= MENU_COLS; moved = true; }
            if (moved) { drawMenu(); last_nav_ms = now; }
        }
        if (readEdge(NG_SW_PIN, ng_prev, ng_ms)) {
            if (g_menu_sel < MENU_GLYPH_CNT) {
                // グリフ描画
                glyph_sel = g_menu_sel;
                g_in_menu = false;
                memset(g_longExpBuf, 0, sizeof(g_longExpBuf));
                M5.Display.fillRect(0, 0, LCD_W, LCD_H-10, TFT_BLACK);
                g_longExpMode = true;
                sendSgly(COMBINED_SEND_IDX[glyph_sel],
                         COMBINED_COLORS[glyph_sel][0],
                         COMBINED_COLORS[glyph_sel][1],
                         COMBINED_COLORS[glyph_sel][2],
                         COMBINED_LABELS[glyph_sel]);
                setStatus("Red:Sel  Black:Draw"); drawStatusBar();
            } else if (g_menu_sel == MENU_REC_IDX) {
                if (!g_recording) {
                    g_rec_len = 0; g_rec_torch = 0;
                    g_recording = true;
                    g_in_menu = false;
                    memset(g_longExpBuf, 0, sizeof(g_longExpBuf));
                    M5.Display.fillRect(0, 0, LCD_W, LCD_H-10, TFT_BLACK);
                    g_longExpMode = true;
                    setStatus("REC... Red:stop"); drawStatusBar();
                } else {
                    g_recording = false;
                    drawMenu();
                }
            } else if (g_menu_sel == MENU_PLAY_IDX && g_rec_len > 0) {
                g_play_idx = 0; g_playing = true;
                g_in_menu = false;
                memset(g_longExpBuf, 0, sizeof(g_longExpBuf));
                M5.Display.fillRect(0, 0, LCD_W, LCD_H-10, TFT_BLACK);
                g_longExpMode = true;
                setStatus("PLAY..."); drawStatusBar();
            }
        }
    } else {
        // ── 通常モード ───────────────────────────────────────────
        if (!g_playing && !g_recording && readEdge(NG_SW_PIN, ng_prev, ng_ms)) {
            memset(g_longExpBuf, 0, sizeof(g_longExpBuf));
            M5.Display.fillRect(0, 0, LCD_W, LCD_H-10, TFT_BLACK);
            g_longExpMode = true;
            sendSgly(COMBINED_SEND_IDX[glyph_sel],
                     COMBINED_COLORS[glyph_sel][0],
                     COMBINED_COLORS[glyph_sel][1],
                     COMBINED_COLORS[glyph_sel][2],
                     COMBINED_LABELS[glyph_sel]);
            setStatus("Red:Sel  Black:Draw"); drawStatusBar();
        }
        // TRG → トーチ (短押し=トグル, 長押し1秒=モーメンタリ)
        {
            static bool torch_persistent = false;
            static bool trg_raw_prev     = false;
            static unsigned long trg_press_start = 0;
            static bool last_torch       = false;
            static constexpr unsigned long TORCH_LONG_MS = 1000;
            bool trg_cur = !digitalRead(TRG_SW_PIN);
            if (trg_cur && !trg_raw_prev) trg_press_start = now;
            if (!trg_cur && trg_raw_prev) {
                unsigned long held = now - trg_press_start;
                if (held >= 50 && held < TORCH_LONG_MS) torch_persistent = !torch_persistent;
            }
            trg_raw_prev = trg_cur;
            bool torch_now = torch_persistent || (trg_cur && (now - trg_press_start >= TORCH_LONG_MS));
            if (torch_now != last_torch) { sendTorch(torch_now); last_torch = torch_now; }
        }
        // 録画状態表示 & 最大録画チェック
        if (g_recording) {
            static int last_disp_len = -1;
            if (g_rec_len >= REC_MAX_FRAMES) {
                g_recording = false;
                setStatus("REC MAX"); drawStatusBar();
            } else if (g_rec_len / 25 != last_disp_len / 25) {
                char buf[32]; snprintf(buf, sizeof(buf), "REC %ds", g_rec_len/50);
                setStatus(buf); drawStatusBar();
                last_disp_len = g_rec_len;
            }
        }
    }

    // ── 再生処理 (20ms毎にctrlパケットを自前送信) ─────────────
    if (g_playing) {
        static unsigned long last_play_ms = 0;
        if (now - last_play_ms >= 20) {
            last_play_ms = now;
            if (g_play_idx < g_rec_len) {
                float ph = g_rec[g_play_idx].jh / 127.0f;
                float pv = g_rec[g_play_idx].jv / 127.0f;
                int xv = constrain((int)(MAX_SPEED * ph), -90, 90);
                int yv = constrain((int)(MAX_SPEED * pv), -90, 90);
                uint8_t pkt[PKT_CTRL_LEN] = {
                    ID_CTRL[0],ID_CTRL[1],ID_CTRL[2],ID_CTRL[3],
                    (uint8_t)(yv + SERVO_OFFSET), (uint8_t)(xv + SERVO_OFFSET), 0
                };
                radio.sendData(pkt, PKT_CTRL_LEN);
                static uint8_t last_play_torch = 0xFF;
                if (g_rec[g_play_idx].torch != last_play_torch) {
                    sendTorch(g_rec[g_play_idx].torch != 0);
                    last_play_torch = g_rec[g_play_idx].torch;
                }
                g_play_idx++;
            } else {
                g_playing = false;
                sendTorch(false);
                setStatus("PLAY done"); drawStatusBar();
            }
        }
    }

    // ── DrawingからのCLRB (DigiKey文字間クリア) ─────────────────
    if (g_clrb_pending && !g_in_menu) {
        g_clrb_pending = false;
        memset(g_longExpBuf, 0, sizeof(g_longExpBuf));
        M5.Display.fillRect(0, 0, LCD_W, LCD_H-10, TFT_BLACK);
        drawStatusBar();
    }

    // 映像タイムアウト監視
    bool is_live = (last_frame_ms != 0) &&
                   ((millis() - last_frame_ms) < FRAME_TIMEOUT_MS);
    if (is_live != was_live) {
        setStatus(is_live ? (is_paired ? "ON AIR" : "ON AIR BCAST") : "NO SIGNAL");
        if (!is_live) drawStatusBar();
        was_live = is_live;
    }

    // 映像表示 (メニュー中は更新しない)
    if (frameReady && !g_in_menu) {
        frameReady = false;
        if (g_longExpMode) {
            // 軌跡モード: 180°回転 + 最大輝度保持
            g_longExpSprite->drawJpg(recvBuf, frameLen, 0, 0, LCD_W, LCD_H);
            const uint16_t* src = (const uint16_t*)g_longExpSprite->getBuffer();
            if (src) {
                const int total = LCD_W * LCD_H;
                for (int i = 0; i < total; i++) {
                    uint16_t px = src[total - 1 - i];
                    if (px > g_longExpBuf[i]) g_longExpBuf[i] = px;
                }
            }
            M5.Display.pushImage(0, 0, LCD_W, LCD_H, g_longExpBuf);
        } else {
            // 通常モード: 180°回転してライブ映像を表示
            g_longExpSprite->drawJpg(recvBuf, frameLen, 0, 0, LCD_W, LCD_H);
            const uint16_t* src = (const uint16_t*)g_longExpSprite->getBuffer();
            if (src) {
                const int total = LCD_W * LCD_H;
                for (int i = 0; i < total; i++) g_longExpBuf[i] = src[total - 1 - i];
                M5.Display.pushImage(0, 0, LCD_W, LCD_H, g_longExpBuf);
            }
        }
        drawStatusBar();
    }
}

// ════════════════════════════════════════════════════════════════
//  env:m5stick-c : 元の LumiGlyph (M5StickC Plus)
// ════════════════════════════════════════════════════════════════
#else

/*
 * LumiGlyph — 5節リンク機構 × LED光軌道文字描画
 *
 * 操作:
 *   ボタンA: 次のグリフを描画
 *   ボタンB: ホーム復帰 / 停止
 *
 * グリフ追加:
 *   python tools/convert_kanjivg.py kanjivg/kanji "あいうえお..." src/glyphs_data.h
 *   でビルドすると glyph_table[] が自動的に使われる。
 */

// FastLED を先に include — TFT_eSPI の #define RED/BLACK と enum 衝突を避ける
#include <FastLED.h>
#include <M5StickCPlus.h>
#include <ESP32Servo.h>
#include <math.h>

// グリフデータ (convert_kanjivg.py が生成したファイルがあれば優先)
#include "glyphs.h"
#if __has_include("glyphs_data.h")
#   include "glyphs_data.h"
#   define USE_GENERATED_TABLE
#endif

// ───────── ピン定義 ─────────
#define SERVO1_PIN  25
#define SERVO2_PIN  26
#define LED_PIN      0
#define NUM_LEDS     1

// ───────── 機構パラメータ (mm) ─────────
static constexpr float L1 = 60.0f;
static constexpr float L2 = 60.0f;
static constexpr float L3 = 60.0f;
static constexpr float L4 = 60.0f;
static constexpr float D  = 36.0f;

// ───────── キャリブレーション ─────────
static constexpr int SERVO1_TRIM =  0;
static constexpr int SERVO2_TRIM =  0;

// ───────── 動作設定 ─────────
static constexpr float HOME_X         = 18.0f;
static constexpr float HOME_Y         = 75.0f;
static constexpr int   STEP_MS        = 30;    // goHome 等のデフォルト
static constexpr int   INTERP_STEPS   = 5;     // 補間ステップ数
static constexpr int   TARGET_DRAW_MS  = 1000; // 目標描画時間 (ms/文字)
static constexpr int   MIN_STEP_MS     = 8;    // サーボ追従の下限
static constexpr int   PENUP_STEP_MS   = 20;   // ペンアップ移動の 1 ステップ時間

// ───────── 表示マッピング ─────────
static constexpr float DISP_SCALE = 2.0f;
static constexpr int   DISP_CX    = 120;
static constexpr int   DISP_CY    =  67;

// ───────── グローバル ─────────
Servo servo1, servo2;
CRGB  leds[NUM_LEDS];

// 使用するグリフテーブルへのポインタ
#ifdef USE_GENERATED_TABLE
static const GlyphDef* const s_table     = glyph_table;
static const int              s_tableSize = GLYPH_TABLE_SIZE;
#else
static const GlyphDef* const s_table     = sample_glyph_table;
static const int              s_tableSize = SAMPLE_GLYPH_COUNT;
#endif

static int     s_glyphIndex = 0;
static bool    s_busy       = false;
static uint8_t s_hue        = 0;   // 虹色サイクル用

// ═══════════════════════════════════════════════
// 逆運動学  elbow-out 構成
// ═══════════════════════════════════════════════
bool calcIK(float x, float y, float &t1, float &t2) {
    float d1sq = x*x + y*y;
    float d1   = sqrtf(d1sq);
    if (d1 > L1+L3 || d1 < fabsf(L1-L3)+0.01f) return false;
    float cosB1 = constrain((L1*L1+d1sq-L3*L3)/(2.0f*L1*d1), -1.0f, 1.0f);
    t1 = atan2f(y,x) + acosf(cosB1);

    float dx2=x-D, dy2=y;
    float d2sq=dx2*dx2+dy2*dy2, d2=sqrtf(d2sq);
    if (d2 > L2+L4 || d2 < fabsf(L2-L4)+0.01f) return false;
    float cosB2 = constrain((L2*L2+d2sq-L4*L4)/(2.0f*L2*d2), -1.0f, 1.0f);
    t2 = atan2f(dy2,dx2) - acosf(cosB2);
    return true;
}

// ═══════════════════════════════════════════════
// 基本描画プリミティブ
// ═══════════════════════════════════════════════
void setPen(bool down) {
    if (down) {
        leds[0] = CHSV(s_hue, 255, 255);
        s_hue += 3;  // 約85ステップで一周
    } else {
        leds[0] = CRGB::Black;
    }
    FastLED.show();
}

void moveTo(float x, float y, bool penDown, int waitMs = STEP_MS) {
    float t1, t2;
    if (!calcIK(x, y, t1, t2)) return;
    int a1 = constrain((int)(t1*RAD_TO_DEG)+SERVO1_TRIM, 0, 180);
    int a2 = constrain((int)(t2*RAD_TO_DEG)+SERVO2_TRIM, 0, 180);
    servo1.write(a1);
    servo2.write(a2);
    setPen(penDown);

    // LCD にリアルタイム描画
    if (penDown) {
        int px = (int)(DISP_CX + (x-HOME_X)*DISP_SCALE);
        int py = (int)(DISP_CY - (y-HOME_Y)*DISP_SCALE);
        if (px>=0 && px<240 && py>=0 && py<135)
            M5.Lcd.drawPixel(px, py, TFT_WHITE);
    }
    delay(waitMs);
}

// steps ステップ線形補間、各ステップ waitMs ms 待機
void drawLine(float x0,float y0,float x1,float y1,bool pen,int steps=INTERP_STEPS,int waitMs=STEP_MS){
    for(int i=0;i<=steps;i++){
        float t=(float)i/steps;
        moveTo(x0+t*(x1-x0), y0+t*(y1-y0), pen, waitMs);
    }
}

void goHome(int ms=600) { moveTo(HOME_X, HOME_Y, false, ms); }

// ═══════════════════════════════════════════════
// グリフ描画
//   ペンダウン区間数から stepMs を逆算し、1文字≈1秒になるよう調整
// ═══════════════════════════════════════════════

// ペンダウン区間数（ウェイポイントのペア数）を数える
static int countDrawSegs(const GlyphDef& g) {
    int n = 0;
    for (uint16_t i = 0; i < g.len; ) {
        int8_t bx = (int8_t)pgm_read_byte(&g.data[i]);
        if (bx == GLYPH_PENUP) { i += 3; }
        else                   { n++;  i += 2; }
    }
    return max(1, n);
}

void drawGlyph(const GlyphDef& g) {
    // 1秒 ÷ (区間数 × 補間ステップ数) = 1ステップあたりの時間
    int drawSegs  = countDrawSegs(g);
    int stepMs    = max(MIN_STEP_MS, TARGET_DRAW_MS / (drawSegs * INTERP_STEPS));

    float px = HOME_X, py = HOME_Y;
    uint16_t i = 0;
    while (i < g.len) {
        int8_t bx = (int8_t)pgm_read_byte(&g.data[i]);

        if (bx == GLYPH_PENUP) {
            i++;
            if (i+1 >= g.len) break;
            int8_t gx = (int8_t)pgm_read_byte(&g.data[i++]);
            int8_t gy = (int8_t)pgm_read_byte(&g.data[i++]);
            float wx = GLYPH_CX + gx * DRAW_SCALE;
            float wy = GLYPH_CY + gy * DRAW_SCALE;
            drawLine(px, py, wx, wy, false, INTERP_STEPS, PENUP_STEP_MS);
            px = wx; py = wy;
        } else {
            i++;
            if (i >= g.len) break;
            int8_t gy = (int8_t)pgm_read_byte(&g.data[i++]);
            float wx = GLYPH_CX + bx * GLYPH_SCALE;
            float wy = GLYPH_CY + gy * GLYPH_SCALE;
            drawLine(px, py, wx, wy, true, INTERP_STEPS, stepMs);
            px = wx; py = wy;
        }
    }
    setPen(false);
}

// グリフテーブルから codepoint を検索
const GlyphDef* findGlyph(uint32_t cp) {
    for (int i = 0; i < s_tableSize; i++) {
        if (pgm_read_dword(&s_table[i].codepoint) == cp)
            return &s_table[i];
    }
    return nullptr;
}

// ═══════════════════════════════════════════════
// リンク機構の可到達領域境界を LCD に描画
// ═══════════════════════════════════════════════
void drawWorkspaceBoundary() {
    const float R = L1 + L3;  // 120mm = 最大リーチ
    const uint16_t col = TFT_DARKGREY;
    int ppx = -1, ppy = -1;

    // 各スクリーン列ごとに上限 y を計算して折れ線描画
    for (int px = 0; px < 240; px++) {
        float wx  = HOME_X + (float)(px - DISP_CX) / DISP_SCALE;
        float y1sq = R*R - wx*wx;
        float y2sq = R*R - (wx - D)*(wx - D);
        float wy;
        if      (y1sq >= 0 && y2sq >= 0) wy = fminf(sqrtf(y1sq), sqrtf(y2sq));
        else if (y1sq >= 0)               wy = sqrtf(y1sq);
        else if (y2sq >= 0)               wy = sqrtf(y2sq);
        else { ppx = -1; continue; }

        int py = (int)(DISP_CY - (wy - HOME_Y) * DISP_SCALE);
        if (py >= 0 && py < 135) {
            if (ppx >= 0) M5.Lcd.drawLine(ppx, ppy, px, py, col);
            ppx = px; ppy = py;
        } else { ppx = -1; }
    }
}

// ═══════════════════════════════════════════════
// 表示ヘルパー
// ═══════════════════════════════════════════════
void drawGlyphPreview(const GlyphDef& g) {
    // 描画エリアのみクリア
    M5.Lcd.fillRect(0, 50, 240, 85, TFT_BLACK);

    // グリフ名を表示
    const char* label = (const char*)pgm_read_ptr(&g.label);
    uint32_t cp = pgm_read_dword(&g.codepoint);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextColor(TFT_CYAN, TFT_BLACK);
    M5.Lcd.setCursor(4, 52);
    M5.Lcd.printf("U+%04X  %s", cp, label);

    drawWorkspaceBoundary();
}

void showHeader() {
    M5.Lcd.fillScreen(TFT_BLACK);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(TFT_CYAN, TFT_BLACK);
    M5.Lcd.setCursor(4, 4);
    M5.Lcd.print("LumiGlyph");
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Lcd.setCursor(4, 26);
    M5.Lcd.printf("[B] Select  [A] Draw  (%d glyphs)", s_tableSize);

    // サーボ位置マーカー
    int sx0=(int)(DISP_CX+(-HOME_X)*DISP_SCALE), sy0=(int)(DISP_CY+HOME_Y*DISP_SCALE);
    int sx1=(int)(DISP_CX+(D-HOME_X)*DISP_SCALE);
    M5.Lcd.fillCircle(sx0, sy0, 3, TFT_YELLOW);
    M5.Lcd.fillCircle(sx1, sy0, 3, TFT_YELLOW);
}

// ═══════════════════════════════════════════════
// setup
// ═══════════════════════════════════════════════
void setup() {
    M5.begin(true, true, true);
    M5.Lcd.setRotation(3);

    // Beep.begin() は M5.begin() 内で無条件実行、ledc ch0 で G2 を駆動する。
    // end() でデタッチしサーボ信号の漏れを防ぐ。
    M5.Beep.end();
    pinMode(SPEAKER_PIN, INPUT);

    // サーボ — timer 2/3 を使用 (Speaker の ledc ch0/timer0 を避ける)
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    servo1.setPeriodHertz(50);
    servo2.setPeriodHertz(50);
    servo1.attach(SERVO1_PIN, 500, 2400);
    servo2.attach(SERVO2_PIN, 500, 2400);

    // WS2812B
    FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(20);
    setPen(false);

    goHome(1000);
    delay(300);

    showHeader();
    drawGlyphPreview(s_table[s_glyphIndex]);

    // 起動フラッシュ
    for (int i=0;i<3;i++){ setPen(true); delay(100); setPen(false); delay(100); }

    M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
    M5.Lcd.setCursor(4, 120);
    M5.Lcd.print("[B] Select  [A] Draw");
}

// ═══════════════════════════════════════════════
// loop
// ═══════════════════════════════════════════════
void loop() {
    M5.update();

    // [B] 文字選択 (サイクル)
    if (M5.BtnB.wasPressed() && !s_busy) {
        s_glyphIndex = (s_glyphIndex + 1) % s_tableSize;
        drawGlyphPreview(s_table[s_glyphIndex]);
        M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
        M5.Lcd.setCursor(4, 120);
        M5.Lcd.print("[B] Select  [A] Draw");
    }

    // [A] 描画開始
    if (M5.BtnA.wasPressed() && !s_busy) {
        s_busy = true;

        const GlyphDef& g = s_table[s_glyphIndex];
        drawGlyphPreview(g);

        M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
        M5.Lcd.setCursor(4, 120);
        M5.Lcd.print("Drawing...            ");

        drawGlyph(g);
        goHome(400);

        // 描画完了 → 自動で次の文字をセット (B 押下相当)
        s_glyphIndex = (s_glyphIndex + 1) % s_tableSize;
        drawGlyphPreview(s_table[s_glyphIndex]);

        M5.Lcd.setCursor(4, 120);
        M5.Lcd.print("[B] Select  [A] Draw");

        s_busy = false;
    }

    delay(50);
}

#endif // ROLE_DRAWING / ROLE_DISPLAY / legacy
