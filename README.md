# LumiGlyph

**5節リンク機構搭載 LED ライトペインティングマシン** — DigiKey Make ONE 2026 出展作品

2台のデバイスが連携し、5節平面リンク機構のクーラー点に取り付けた WS2812B LED でグリフ（文字・記号）の筆跡をトレース。長時間露光写真として光の書道を記録します。

- **動画 (YouTube)**: https://youtu.be/CDKWRTzFumU
- **ソースコード (GitHub)**: https://github.com/sin1n24/LumiGlyphProtoPedia
- **作品ページ (ProtoPedia)**: https://protopedia.net/prototype/8594
- **作者**: sin1's studio — https://sin1.studio/

---

## 部品リスト（BOM）

### 描画機
| 部品 | 備考 |
|------|------|
| M5Stack AtomS3R-CAM | メインコントローラ + カメラ |
| マイクロサーボ × 2 | G25, G26 に接続 |
| WS2812B LED × 1 | G0 に接続（ペン LED） |
| **4サーボ接続基板**（自作） | サーボ電源供給基板 — [スイッチサイエンス](https://ssci.to/11122) |
| 5節リンク機構フレーム | 自作、全リンク長 60mm |

### 表示機
| 部品 | 備考 |
|------|------|
| M5Stack AtomS3R | メインコントローラ + 表示 |
| **ミニコントローラ**（自作） | ジョイスティック + ボタン3個 ハンドヘルド型 — [スイッチサイエンス](https://ssci.to/9520) |

---

## システム概要

| 役割 | デバイス | 機能 |
|------|----------|------|
| **描画 (Drawing)** | M5Stack AtomS3R-CAM | 逆運動学計算・サーボ駆動・LED 制御 |
| **表示 (Display)** | M5Stack AtomS3R | カメラ映像受信・長時間露光合成・メニュー UI |

2台は ESP-NOW で無線通信します。描画機がカメラフレームを送信し、表示機が画素ごとの最大輝度合成（長時間露光ブレンド）を行ってライトトレール映像を生成します。

---

## ハードウェア仕様

### 描画機（AtomS3R-CAM）
- **ボード**: M5Stack AtomS3R-CAM（ESP32-S3）
- **サーボ**: マイクロサーボ × 2 — G25（サーボ1）、G26（サーボ2）
- **LED**: WS2812B × 1 — G0
- **リンク機構**: 5節平面リンク、全リンク長 L = 60mm、ピボット間隔 D = 36mm

### 表示機（AtomS3R）
- **ボード**: M5Stack AtomS3R（ESP32-S3）
- **ジョイスティック**: X → G8、Y → G7
- **ボタン**: 赤（メニュー）→ G38、黒（描画）→ G39、トリガー（トーチ）→ G5

---

## ビルド・書き込み

[PlatformIO](https://platformio.org/) が必要です。

```bash
# 描画機（AtomS3R-CAM）書き込み
pio run -e Drawing --target upload

# 表示機（AtomS3R）書き込み
pio run -e Display --target upload
```

アップロード速度は `platformio.ini` で環境ごとに設定されています。

### 初回ペアリング
両デバイスを起動し、表示機側で BtnA を長押しすると `PAIRED` が表示されます。

---

## 操作方法

### 表示機のボタン操作

| 入力 | 動作 |
|------|------|
| **赤ボタン** | メニューを開く / 閉じる |
| **黒ボタン** | 選択中のグリフを描画（メニュー内: 実行） |
| **トリガー** 短押し | LED トーチ ON/OFF |
| **トリガー** 長押し（1秒以上） | 押している間だけトーチ点灯 |
| **BtnA** ダブルタップ | 現在の LED 位置をホームに設定 |
| **ジョイスティック** | 手動描画 / メニュー移動 |

### メニュー項目

| 項目 | 内容 |
|------|------|
| I / Ro / Ha | 片仮名 1 文字描画 |
| IRoHa | イ→ロ→ハ を順次描画（各文字で露光） |
| IChi | イ→チ を順次描画（各文字で露光） |
| A / Heart / Star / Circle / Tri / Rect | 各種記号グリフ |
| d / i / g / k / e / y | 英小文字 |
| **DigiKey** | `D-i-g-i-K-e-y` を 7 文字順次描画 |
| **REC** | ジョイスティック + トーチ操作を最大 5 秒録画 |
| **PLAY** | 最後の録画を再生 |

---

## グリフシステム

グリフは `src/glyphs.h` に `int8_t` 座標ストリームとして定義されています。

```
GLYPH_PENUP (-128)  x  y   → ペンを上げて絶対座標 (x, y) へ移動
x  y                        → 絶対座標 (x, y) へ描画
```

座標の物理換算: `physical_mm = center + value × 0.5`  
デフォルト中心: (18, 75) mm — ダブルタップでランタイム変更可能

KanjiVG SVG データからグリフを自動生成する場合:

```bash
python tools/convert_kanjivg.py kanjivg/kanji "あいうえお" src/glyphs_data.h
```

`src/glyphs_data.h`（自動生成）が存在する場合は `src/glyphs.h` より優先されます。

---

## サーボキャリブレーション

`src/main.cpp` の先頭にあるトリム定数を編集してください:

```cpp
static int SERVO1_TRIM = 0;   // サーボ1の角度オフセット（度）
static int SERVO2_TRIM = 0;   // サーボ2の角度オフセット（度）
static int SERVO1_DIR  = 1;   // 1 または -1
static int SERVO2_DIR  = 1;
```

これらの値は SPIFFS に保存され、キャリブレーションモードでランタイム調整も可能です。

---

## プロジェクト構成

```
src/
  main.cpp        # ROLE_DRAWING + ROLE_DISPLAY（ビルドフラグで切替）
  glyphs.h        # 手書きグリフストロークデータ
  ik.h            # 5節リンク機構 逆運動学
tools/
  convert_kanjivg.py   # KanjiVG → glyphs_data.h 変換スクリプト
platformio.ini    # ビルド環境: Drawing, Display, m5stick-c
docs/
  index.html      # GitHub Pages 紹介ページ
```

---

## ライセンス

MIT
