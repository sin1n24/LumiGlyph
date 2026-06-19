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
            float wx = GLYPH_CX + gx * GLYPH_SCALE;
            float wy = GLYPH_CY + gy * GLYPH_SCALE;
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
