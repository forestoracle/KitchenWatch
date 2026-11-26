#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <FastLED.h>
#include <Wire.h>
#include <EEPROM.h>
#include <RTClib.h>
#include <time.h>
#include <RDA5807.h>
#include <GyverEncoder.h>

/* ============================================================
 *  KitchenWatch — ESP8266 (NodeMCU v2)
 *  Главный файл прошивки.
 *
 *  Инициализирует подключённые библиотеки:
 *    - ESP8266WiFi       — подключение к Wi-Fi сети
 *    - FastLED           — управление адресной LED-матрицей
 *    - WebSockets        — сервер для управления с браузера/телефона
 *    - ArduinoJson       — разбор/генерация JSON-сообщений
 *    - Wire + RTClib     — часы реального времени PCF8563 (I2C)
 *    - EEPROM            — эмуляция энергонезависимой памяти во flash
 *
 *  Индикатор: 4 панели 8x8, выстроенные в один горизонтальный ряд
 *  (общее поле 32x8). На старте показывается анимация подключения
 *  к Wi-Fi. После подключения время PCF8563 синхронизируется с
 *  NTP-сервером, и на панелях отображаются текущие часы (ЧЧ:ММ).
 *  Зелёная точка в правом верхнем углу сигнализирует о том, что
 *  Wi-Fi подключён.
 *
 *  Если сеть недоступна (или подключиться не удалось за несколько
 *  попыток) и синхронизация невозможна, устройство переходит в
 *  автономный режим: часы читают и показывают время прямо из PCF8563
 *  без синхронизации. Сеть при этом продолжает периодически
 *  пробоваться в фоне, и при её появлении время тихо синхронизируется.
 *
 *  Настройки (яркость матрицы, станция, громкость и цвета цифр
 *  часов/частоты) хранятся в энергонезависимой памяти — эмуляции EEPROM
 *  во flash ESP8266 (библиотека EEPROM.h). Запись выполняется через
 *  10 секунд после последнего изменения: один цикл стирания сектора на
 *  пачку изменений экономит ресурс flash.
 * ============================================================ */

// ---------- Настройки Wi-Fi ----------
const char* WIFI_SSID     = "rainforest";      // <-- укажите имя сети
const char* WIFI_PASSWORD = "m@rtlet1Bush";  // <-- укажите пароль

// ---------- Панели 8x8 ----------
#define PANEL_COUNT   4                          // количество панелей
#define PANEL_SIZE    8                          // размер одной панели 8x8
#define MATRIX_WIDTH  (PANEL_COUNT * PANEL_SIZE) // общая ширина = 32
#define MATRIX_HEIGHT PANEL_SIZE                 // общая высота = 8
#define NUM_LEDS      (MATRIX_WIDTH * MATRIX_HEIGHT) // всего 256 светодиодов

// ---------- Настройки LED-матрицы ----------
#define LED_PIN     2       // GPIO2 (D4) — линия данных
#define COLOR_ORDER GRB     // порядок каналов цвета (WS2812B -> GRB)
#define LED_DEFAULT_BRIGHTNESS 64 // стартовая яркость 0..255 (если во flash нет сохранённой)

// true  — «змейка» внутри каждой панели (стандарт для матриц 8x8 WS2812);
// false — построчная разводка (строки идут слева направо).
const bool kMatrixSerpentineLayout = false;

// ---------- Диагностика разводки матрицы ----------
// 0 = обычная работа. 1 = «прогон индексов» — по очереди зажигает каждый
// светодиод FastLED и печатает его индекс в Serial, чтобы сопоставить индекс
// с физическим положением на матрице (см. runMatrixTest()).
#define MATRIX_TEST_MODE 0

// ---------- Ориентация дисплея ----------
// Если изображение зеркальное/перевёрнутое, меняйте нужные флаги (0/1),
// не трогая логику XY().
#define FLIP_X              0   // 1 = зеркально по горизонтали (лево <-> право)
#define FLIP_Y              0   // 1 = зеркально по вертикали (верх <-> низ)
#define REVERSE_PANEL_ORDER 0   // 1 = первая панель цепочки физически справа

// ---------- Часы реального времени PCF8563 (I2C) ----------
// NodeMCU v2: SDA = GPIO4 (D2), SCL = GPIO5 (D1) — пины Wire по умолчанию.
// Микросхема PCF8563 работает по адресу 0x51 (DS1307 использовал 0x68).
RTC_PCF8563 rtc;

// ---------- FM-приёмник RDA5807 (I2C) ----------
// Модуль подключается к той же шине I2C, что и PCF8563 (SDA=D2, SCL=D1).
// Адреса чипа: 0x10 (полный доступ) и 0x11 (доступ к одному регистру).
RDA5807 rx;

// ---------- Настройки радио по умолчанию ----------
#define RADIO_DEFAULT_FREQ   10390   // стартовая станция 103.9 МГц (единицы 10 кГц)
#define RADIO_DEFAULT_VOLUME 8       // стартовая громкость (0..15)
#define VOLUME_SHOW_MS       2000    // показ шкалы громкости на матрице после изменения, мс

// ---------- Энергонезависимая память: эмуляция EEPROM во flash ----------
// Отдельной EEPROM у ESP8266 нет: Arduino-ядро отводит под неё один сектор
// flash 4 КБ (у карты 4 МБ — 0x405FB000, сразу за файловой системой).
// EEPROM.begin(EE_SIZE) читает EE_SIZE байт сектора в RAM, а commit() стирает
// сектор и записывает данные обратно. Настройки (яркость, станция, громкость и
// цвета цифр) записываются через SAVE_DELAY_MS после последнего изменения — одно
// стирание сектора на пачку изменений экономит ресурс flash.
#define EE_SIZE           16     // сколько байт нужно (максимум библиотеки — 4096);
                                 // 16 — уже кратно 4 байтам

#define EE_ADDR_MAGIC       0x00 // сигнатура «настройки радио записаны»
#define EE_ADDR_FREQ        0x01 // 2 байта: частота в единицах 10 кГц
#define EE_ADDR_VOLUME      0x03 // 1 байт: громкость 0..15
#define EE_ADDR_CLOCK_R     0x04 // 3 байта: цвет цифр часов (R, G, B)
#define EE_ADDR_CLOCK_G     0x05
#define EE_ADDR_CLOCK_B     0x06
#define EE_ADDR_FREQ_R      0x07 // 3 байта: цвет цифр частоты (R, G, B)
#define EE_ADDR_FREQ_G      0x08
#define EE_ADDR_FREQ_B      0x09
#define EE_ADDR_COLOR_MAGIC 0x0A // сигнатура «цвета сохранены»
#define EE_ADDR_BRIGHTNESS  0x0B // 1 байт: яркость матрицы 0..255
#define EE_ADDR_BRIGHT_MAGIC 0x0C // сигнатура «яркость сохранена»
#define EE_MAGIC            0x4B // 'K' — KitchenWatch
#define EE_COLOR_MAGIC      0xC1 // сигнатура блока цветов цифр
#define EE_BRIGHT_MAGIC     0xB7 // сигнатура блока яркости

#define SAVE_DELAY_MS     10000  // пауза после изменения до записи во flash, мс

// Минимальный интервал авторассылки состояния клиентам WebSocket после
// изменения настроек (громкость при повороте энкодера «трещит» по щелчкам).
#define STATE_PUSH_INTERVAL_MS 150

// ---------- Энкодер ----------
// Поворот — поиск следующей/предыдущей станции; кнопка — переключение
// индикации «часы / частота станции».
#define ENC_CLK_PIN 12   // D6 — выход A энкодера
#define ENC_DT_PIN  13   // D7 — выход B энкодера
#define ENC_SW_PIN  14   // D5 — кнопка энкодера
Encoder enc(ENC_CLK_PIN, ENC_DT_PIN, ENC_SW_PIN); // тип задаётся в setup() (TYPE2)

// ---------- Синхронизация времени по NTP ----------
#define NTP_SERVER_1      "pool.ntp.org"
#define NTP_SERVER_2      "time.nist.gov"
#define GMT_OFFSET_SEC    (3 * 3600)   // часовой пояс: UTC+3 (Москва); без учёта DST
#define NTP_VALID_EPOCH   1000000000L  // время валидно начиная с 2001-09-09
#define SYNC_TIMEOUT_MS   15000        // сколько ждать NTP до перехода на показания PCF8563

// ---------- Повторные попытки подключения к Wi-Fi ----------
#define WIFI_MAX_ATTEMPTS        3      // число попыток подключения при старте
#define WIFI_ATTEMPT_TIMEOUT_MS  10000  // таймаут одной попытки подключения, мс
#define WIFI_RECONNECT_MS        60000  // период фоновых попыток восстановления связи, мс

// Цвета индикации по умолчанию и период обновления кадра.
// Текущие цвета хранятся в gClockColor / gFreqColor: их можно менять
// командами WebSocket и они сохраняются в EEPROM.
#define CLOCK_COLOR       CRGB::Green  // цифры часов
#define FREQ_COLOR        CRGB::Blue   // цифры частоты: горит только синий канал — меньше ток
#define CLOCK_REFRESH_MS  500          // перерисовываем часы 2 раза в секунду

CRGB leds[NUM_LEDS];

// ---------- WebSocket-сервер ----------
#define WEBSOCKET_PORT 81
WebSocketsServer webSocket(WEBSOCKET_PORT);

// ---------- Текущее состояние ----------
uint8_t gBrightness = LED_DEFAULT_BRIGHTNESS; // яркость 0..255, сохраняется в EEPROM
CRGB    gColor      = CRGB::Green;
bool    gManualMode = false; // true = дисплеем управляет пользователь (JSON)

// Текущие цвета цифр (меняются командами clock_color / freq_color,
// сохраняются в EEPROM вместе с остальными настройками).
CRGB    gClockColor = CLOCK_COLOR;
CRGB    gFreqColor  = FREQ_COLOR;

enum AppState {
    STATE_CONNECTING, // Wi-Fi ещё не подключён — показываем анимацию
    STATE_SYNCING,    // Wi-Fi подключён — ждём синхронизации PCF8563 с NTP
    STATE_RUNNING     // часы работают (читаем время из PCF8563)
};
AppState      gState       = STATE_CONNECTING;
unsigned long gSyncStartMs = 0;   // момент начала синхронизации с NTP
unsigned long gLastDrawMs  = 0;   // время последней перерисовки часов
bool          gRtcFound    = false; // true = PCF8563 найден на шине I2C

// Состояние подключения к Wi-Fi и синхронизации времени.
unsigned long gWifiAttemptMs   = 0;     // момент старта текущей попытки подключения
unsigned long gLastReconnectMs = 0;     // время последней фоновой попытки подключения
uint8_t       gWifiAttempts    = 0;     // число сделанных попыток подключения
bool          gTimeSynced      = false; // true = время RTC синхронизировано с NTP
bool          gSntpArmed       = false; // true = configTime() вызван для текущего соединения

enum DisplayMode {
    DISPLAY_TIME,  // часы
    DISPLAY_FREQ   // частота FM-станции
};
DisplayMode gDisplayMode = DISPLAY_TIME;

bool          gRadioFound     = false; // true = RDA5807 найден на шине I2C

// ---------- Настройки радио и энергонезависимая память ----------
uint16_t gRadioFreq   = RADIO_DEFAULT_FREQ;   // текущая станция (единицы 10 кГц)
uint8_t  gRadioVolume = RADIO_DEFAULT_VOLUME; // текущая громкость (0..15)

// Временный показ шкалы громкости на матрице (после изменения энкодером).
bool          gVolumeOverlay   = false; // true = показываем шкалу громкости
unsigned long gVolumeOverlayMs = 0;     // момент последнего изменения громкости

bool          gEepromReady       = false; // true = EEPROM.begin() выполнен
bool          gSettingsDirty     = false; // true = настройки изменены, ждём запись
unsigned long gSettingsChangedMs = 0;     // момент последнего изменения настроек

// Авторассылка состояния клиентам WebSocket после изменений. Сама рассылка
// выполняется из loop() не чаще STATE_PUSH_INTERVAL_MS.
bool          gStateDirty        = false; // true = состояние изменилось
unsigned long gStateDirtyMs      = 0;     // момент последнего изменения состояния

/* ------------------------------------------------------------
 *  Энергонезависимая память: эмуляция EEPROM во flash ESP8266.
 *  begin() отводит EE_SIZE байт кучи и читает их из сектора
 *  flash; write() меняет только RAM-копию, commit() стирает
 *  сектор и записывает копию обратно.
 * ------------------------------------------------------------ */
bool eepromInit() {
    EEPROM.begin(EE_SIZE);   // в ядре ESP8266 begin() возвращает void
    gEepromReady = ((size_t)EEPROM.length() == (size_t)EE_SIZE);
    return gEepromReady;
}

// Чтение одного байта из RAM-копии (flash при этом не читается).
bool eepromReadByte(uint8_t addr, uint8_t& value) {
    if (!gEepromReady || addr >= EE_SIZE) {
        return false;
    }
    value = EEPROM.read(addr);
    return true;
}

// Запись одного байта в RAM-копию; во flash — только по eepromCommit().
// Если байт не изменился, признак «данные изменены» не выставляется.
void eepromWriteByte(uint8_t addr, uint8_t value) {
    if (!gEepromReady || addr >= EE_SIZE) {
        return;
    }
    EEPROM.write(addr, value);
}

// Сброс RAM-копии во flash: стирание сектора + запись EE_SIZE байт.
bool eepromCommit() {
    if (!gEepromReady) {
        return false;
    }
    return EEPROM.commit();
}

/* ------------------------------------------------------------
 *  Прочитать сохранённые яркость, станцию, громкость и цвета цифр
 *  из flash. При отсутствии/некорректности данных остаются значения
 *  по умолчанию (LED_DEFAULT_BRIGHTNESS / RADIO_DEFAULT_FREQ /
 *  RADIO_DEFAULT_VOLUME / CLOCK_COLOR / FREQ_COLOR).
 * ------------------------------------------------------------ */
void loadSettings() {
    uint8_t magic = 0;
    if (!eepromReadByte(EE_ADDR_MAGIC, magic) || magic != EE_MAGIC) {
        Serial.println("[EEPROM] no saved settings — using defaults");
        return;
    }

    uint8_t hi = 0, lo = 0, vol = 0;
    if (!eepromReadByte(EE_ADDR_FREQ, hi) ||
        !eepromReadByte(EE_ADDR_FREQ + 1, lo) ||
        !eepromReadByte(EE_ADDR_VOLUME, vol)) {
        Serial.println("[EEPROM] read error — using defaults");
        return;
    }

    uint16_t freq = ((uint16_t)hi << 8) | lo;
    if (freq >= 8750 && freq <= 10800) { // FM-диапазон 87.5..108.0 МГц
        gRadioFreq = freq;
    }
    if (vol <= 15) {
        gRadioVolume = vol;
    }

    // Цвета цифр хранятся с отдельной сигнатурой: настройки, записанные
    // прежней прошивкой, не должны дать случайный цвет (0xFF во flash).
    uint8_t colorMagic = 0;
    if (eepromReadByte(EE_ADDR_COLOR_MAGIC, colorMagic) &&
        colorMagic == EE_COLOR_MAGIC) {
        uint8_t cr = 0, cg = 0, cb = 0, fr = 0, fg = 0, fb = 0;
        if (eepromReadByte(EE_ADDR_CLOCK_R, cr) &&
            eepromReadByte(EE_ADDR_CLOCK_G, cg) &&
            eepromReadByte(EE_ADDR_CLOCK_B, cb) &&
            eepromReadByte(EE_ADDR_FREQ_R, fr) &&
            eepromReadByte(EE_ADDR_FREQ_G, fg) &&
            eepromReadByte(EE_ADDR_FREQ_B, fb)) {
            gClockColor = CRGB(cr, cg, cb);
            gFreqColor  = CRGB(fr, fg, fb);
        }
    }

    // Яркость также хранится с отдельной сигнатурой: байт, оставшийся
    // от прежней прошивки (0xFF во flash), не должен дать яркость 255.
    uint8_t brightMagic = 0;
    if (eepromReadByte(EE_ADDR_BRIGHT_MAGIC, brightMagic) &&
        brightMagic == EE_BRIGHT_MAGIC) {
        uint8_t bright = 0;
        if (eepromReadByte(EE_ADDR_BRIGHTNESS, bright)) {
            gBrightness = bright;
        }
    }

    Serial.printf("[EEPROM] loaded: freq=%u vol=%u bright=%u clock=(%u,%u,%u) freq_col=(%u,%u,%u)\n",
                  gRadioFreq, gRadioVolume, gBrightness,
                  gClockColor.r, gClockColor.g, gClockColor.b,
                  gFreqColor.r, gFreqColor.g, gFreqColor.b);
}

/* ------------------------------------------------------------
 *  Записать текущие яркость, станцию, громкость и цвета цифр в
 *  энергонезависимую память. Сначала обновляется RAM-копия,
 *  затем одним commit() перезаписывается сектор flash — одно
 *  стирание на пачку изменений. При сбое питания посреди записи
 *  сигнатуры EE_MAGIC / EE_COLOR_MAGIC / EE_BRIGHT_MAGIC могут не
 *  записаться, и соответствующие данные будут отвергнуты.
 *  true — данные записаны во flash; false — ошибка записи.
 * ------------------------------------------------------------ */
bool saveSettings() {
    if (!gEepromReady) {
        return false;
    }
    uint16_t freq = gRadioFound ? rx.getFrequency() : gRadioFreq;
    uint8_t  vol  = gRadioVolume;

    eepromWriteByte(EE_ADDR_FREQ,     (uint8_t)(freq >> 8));
    eepromWriteByte(EE_ADDR_FREQ + 1, (uint8_t)(freq & 0xFF));
    eepromWriteByte(EE_ADDR_VOLUME,   vol);
    eepromWriteByte(EE_ADDR_MAGIC,    EE_MAGIC);

    eepromWriteByte(EE_ADDR_CLOCK_R,  gClockColor.r);
    eepromWriteByte(EE_ADDR_CLOCK_G,  gClockColor.g);
    eepromWriteByte(EE_ADDR_CLOCK_B,  gClockColor.b);
    eepromWriteByte(EE_ADDR_FREQ_R,   gFreqColor.r);
    eepromWriteByte(EE_ADDR_FREQ_G,   gFreqColor.g);
    eepromWriteByte(EE_ADDR_FREQ_B,   gFreqColor.b);
    eepromWriteByte(EE_ADDR_COLOR_MAGIC, EE_COLOR_MAGIC);

    // Яркость пишется последней вместе со своей сигнатурой.
    eepromWriteByte(EE_ADDR_BRIGHTNESS,   gBrightness);
    eepromWriteByte(EE_ADDR_BRIGHT_MAGIC, EE_BRIGHT_MAGIC);

    if (!eepromCommit()) {
        Serial.println("[EEPROM] commit failed — will retry later");
        return false;
    }

    gRadioFreq = freq;
    Serial.printf("[EEPROM] saved: freq=%u vol=%u bright=%u\n",
                  freq, vol, gBrightness);
    return true;
}

/* ------------------------------------------------------------
 *  Отметить, что настройки изменились: запись во flash будет
 *  выполнена через SAVE_DELAY_MS после последнего изменения.
 * ------------------------------------------------------------ */
void markSettingsChanged() {
    gSettingsDirty     = true;
    gSettingsChangedMs = millis();
}

/* ------------------------------------------------------------
 *  Отложенная (через SAVE_DELAY_MS) запись настроек во flash.
 *  Вызывается из loop(); ничего не делает, пока пауза не истекла.
 *  При ошибке записи настройки остаются «грязными» и попытка
 *  повторяется ещё через SAVE_DELAY_MS.
 * ------------------------------------------------------------ */
void serviceSettingsSave() {
    if (!gSettingsDirty || !gEepromReady) {
        return;
    }
    if (millis() - gSettingsChangedMs >= SAVE_DELAY_MS) {
        if (saveSettings()) {
            gSettingsDirty = false;
        } else {
            gSettingsChangedMs = millis(); // повторим попытку позже
        }
    }
}

/* ------------------------------------------------------------
 *  XY-отображение: переводит координаты (x, y) в индекс светодиода.
 *  x — по горизонтали (0..31), y — по вертикали (0..7, 0 = верх).
 *  Панели идут слева направо; данные передаются через все панели
 *  последовательно по одному пину (первая в цепочке — левая панель).
 *  Ориентация настраивается флагами FLIP_X / FLIP_Y / REVERSE_PANEL_ORDER.
 * ------------------------------------------------------------ */
uint16_t XY(uint8_t x, uint8_t y) {
    // 1. Ориентация по горизонтали (лево <-> право).
    if (FLIP_X) {
        x = MATRIX_WIDTH - 1 - x;
    }
    // 2. Ориентация по вертикали (верх <-> низ).
    if (FLIP_Y) {
        y = MATRIX_HEIGHT - 1 - y;
    }

    uint8_t  panel = x / PANEL_SIZE;   // номер панели 0..3
    uint8_t  px    = x % PANEL_SIZE;   // координата внутри панели

    // 3. Когда первая панель цепочки физически находится справа.
    if (REVERSE_PANEL_ORDER) {
        panel = PANEL_COUNT - 1 - panel;
    }

    uint16_t local;
    if (kMatrixSerpentineLayout && (y & 0x01)) {
        // Нечётные строки в матрице разведены в обратную сторону.
        local = (uint16_t)(y + 1) * PANEL_SIZE - 1 - px;
    } else {
        local = (uint16_t)y * PANEL_SIZE + px;
    }

    return (uint16_t)panel * (PANEL_SIZE * PANEL_SIZE) + local;
}

/* ------------------------------------------------------------
 *  Установка пикселя по координатам (безопасная).
 * ------------------------------------------------------------ */
void setPixel(int16_t x, int16_t y, const CRGB& color) {
    if (x < 0 || x >= MATRIX_WIDTH || y < 0 || y >= MATRIX_HEIGHT) {
        return;
    }
    leds[XY((uint8_t)x, (uint8_t)y)] = color;
}

/* ------------------------------------------------------------
 *  Диагностика разводки матрицы (MATRIX_TEST_MODE = 1).
 *  По очереди зажигает светодиоды по индексу FastLED (0..255)
 *  и выводит в Serial его координаты. Наблюдая за «бегущим»
 *  красным пикселем, легко определить реальную разводку:
 *    - пиксель идёт строго слева направо по каждой строке без
 *      разворотов  -> матрица прогрессивная, нужен
 *                     kMatrixSerpentineLayout = false;
 *    - на нечётных строках пиксель идёт в обратную сторону
 *      (змейкой)    -> kMatrixSerpentineLayout = true;
 *    - пиксель прыгает между панелями -> неверен порядок/направление
 *      панелей, попробуйте REVERSE_PANEL_ORDER.
 * ------------------------------------------------------------ */
void runMatrixTest() {
    static unsigned long last = 0;
    static uint16_t idx = 0;

    if (millis() - last < 300) {
        return;
    }
    last = millis();

    FastLED.clear();
    leds[idx] = CRGB::Red;
    FastLED.show();

    Serial.printf("[TEST] idx=%u  panel=%u  px=%u  py=%u\n",
                  (unsigned)idx,
                  (unsigned)(idx / 64),
                  (unsigned)((idx % 64) % 8),
                  (unsigned)((idx % 64) / 8));
    idx = (idx + 1) % NUM_LEDS;
}

/* ------------------------------------------------------------
 *  Анимация процесса подключения к Wi-Fi: «бегущая» точка
 *  с затухающим следом (не блокирует работу цикла).
 * ------------------------------------------------------------ */
void drawConnectingAnimation() {
    fadeToBlackBy(leds, NUM_LEDS, 48);

    uint16_t t   = millis() / 25;
    uint8_t  pos = t % (2 * MATRIX_WIDTH);           // ping-pong
    if (pos >= MATRIX_WIDTH) {
        pos = 2 * MATRIX_WIDTH - 1 - pos;
    }

    setPixel(pos, 3, CRGB::Cyan);
    setPixel(pos, 4, CRGB::Cyan);
    FastLED.show();
}

/* ------------------------------------------------------------
 *  Шрифт 5x7 для цифр 0..9. Каждая цифра — 7 строк по 5 бит
 *  (бит 4 = левая колонка, бит 0 = правая).
 * ------------------------------------------------------------ */
static const uint8_t DIGIT_FONT[10][7] = {
    {0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E}, // 0
    {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E}, // 1
    {0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F}, // 2
    {0x1F, 0x02, 0x04, 0x02, 0x01, 0x11, 0x0E}, // 3
    {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02}, // 4
    {0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E}, // 5
    {0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E}, // 6
    {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08}, // 7
    {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E}, // 8
    {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C}  // 9
};

/* ------------------------------------------------------------
 *  Рисует цифру 5x7 в пределах одной панели 8x8.
 *  panelX — номер панели по горизонтали (0..3).
 * ------------------------------------------------------------ */
void drawDigit(uint8_t digit, uint8_t panelX, const CRGB& color) {
    digit %= 10;
    const uint8_t colOffset = 1; // (8 - 5) / 2 ≈ по центру
    for (uint8_t row = 0; row < 7; row++) {
        uint8_t bits = DIGIT_FONT[digit][row];
        for (uint8_t col = 0; col < 5; col++) {
            if (bits & (0x10 >> col)) {
                setPixel(panelX * PANEL_SIZE + colOffset + col, row, color);
            }
        }
    }
}

/* ------------------------------------------------------------
 *  Отображение текущего времени ЧЧ:ММ из PCF8563.
 *  Двоеточие мигает раз в секунду; в правом верхнем углу —
 *  зелёная точка, пока Wi-Fi подключён.
 * ------------------------------------------------------------ */
void drawClock() {
    DateTime now = rtc.now();

    FastLED.clear();

    drawDigit(now.hour() / 10, 0, gClockColor);
    drawDigit(now.hour() % 10, 1, gClockColor);
    drawDigit(now.minute() / 10, 2, gClockColor);
    drawDigit(now.minute() % 10, 3, gClockColor);

    // Мигающее двоеточие между часами и минутами (гаснет каждую секунду).
    if ((now.second() & 0x01) == 0) {
        setPixel(15, 2, gClockColor);
        setPixel(15, 4, gClockColor);
    }

    // Зелёная точка — индикатор успешного Wi-Fi подключения.
    if (WiFi.status() == WL_CONNECTED) {
        setPixel(MATRIX_WIDTH - 1, 0, CRGB::Green); // (31, 0)
    }

    FastLED.show();
}

/* ------------------------------------------------------------
 *  Индикатор ошибки: PCF8563 не найден на шине I2C.
 *  Красная точка в левом верхнем углу + зелёная точка Wi-Fi.
 * ------------------------------------------------------------ */
void drawErrorIndicator() {
    FastLED.clear();
    setPixel(0, 0, CRGB::Red); // RTC не найден
    if (WiFi.status() == WL_CONNECTED) {
        setPixel(MATRIX_WIDTH - 1, 0, CRGB::Green);
    }
    FastLED.show();
}

/* ------------------------------------------------------------
 *  Отображение частоты текущей FM-станции.
 *  Формат: [сотни]десятки-единицы.десятые, например 103.9 или 87.5.
 *  Десятичная точка рисуется в правом нижнем углу панели единиц.
 * ------------------------------------------------------------ */
void drawFrequency() {
    // getFrequency() возвращает частоту в единицах 10 кГц (10390 = 103.9 МГц).
    uint16_t f       = rx.getFrequency();
    uint16_t mhz10   = f / 10;      // 1039 -> «103.9»
    uint8_t  intPart = mhz10 / 10;  // целая часть (87..108)
    uint8_t  frac    = mhz10 % 10;  // десятые доли

    FastLED.clear();

    if (intPart >= 100) {
        drawDigit((intPart / 100) % 10, 0, gFreqColor);
        drawDigit((intPart / 10)  % 10, 1, gFreqColor);
    } else {
        // Частоты ниже 100 МГц — первая панель пустая, цифры сдвинуты вправо.
        drawDigit((intPart / 10) % 10, 1, gFreqColor);
    }
    drawDigit(intPart % 10, 2, gFreqColor);
    drawDigit(frac, 3, gFreqColor);

    // Десятичная точка в правом нижнем углу панели с единицами (панель 2).
    setPixel(2 * PANEL_SIZE + 6, 7, gFreqColor);
    setPixel(2 * PANEL_SIZE + 7, 7, gFreqColor);

    // Зелёная точка — индикатор Wi-Fi (как на экране часов).
    if (WiFi.status() == WL_CONNECTED) {
        setPixel(MATRIX_WIDTH - 1, 0, CRGB::Green);
    }

    FastLED.show();
}

/* ------------------------------------------------------------
 *  Индикатор ошибки: RDA5807 не найден на шине I2C.
 *  Красная точка в левом нижнем углу + зелёная точка Wi-Fi.
 * ------------------------------------------------------------ */
void drawRadioErrorIndicator() {
    FastLED.clear();
    setPixel(0, MATRIX_HEIGHT - 1, CRGB::Red); // приёмник не найден
    if (WiFi.status() == WL_CONNECTED) {
        setPixel(MATRIX_WIDTH - 1, 0, CRGB::Green);
    }
    FastLED.show();
}

/* ------------------------------------------------------------
 *  Шкала громкости FM-приёмника: 16 уровней (0..15), по 2
 *  светодиода на уровень (заполняет всю ширину 32 px).
 *  Показывается кратковременно после изменения громкости.
 * ------------------------------------------------------------ */
void drawVolume() {
    FastLED.clear();

    for (uint8_t level = 0; level < 16; level++) {
        if (level >= gRadioVolume) {
            break;
        }
        for (uint8_t x = 0; x < 2; x++) {
            for (uint8_t y = 2; y <= 5; y++) {
                setPixel(level * 2 + x, y, CRGB::Orange);
            }
        }
    }

    // Зелёная точка — индикатор Wi-Fi (как на экране часов).
    if (WiFi.status() == WL_CONNECTED) {
        setPixel(MATRIX_WIDTH - 1, 0, CRGB::Green);
    }

    FastLED.show();
}

/* ------------------------------------------------------------
 *  Отрисовка текущего режима индикации: шкала громкости
 *  (кратковременно), часы или частота станции.
 * ------------------------------------------------------------ */
void drawCurrentDisplay() {
    // Кратковременно показываем шкалу громкости после её изменения.
    if (gVolumeOverlay) {
        if (millis() - gVolumeOverlayMs < VOLUME_SHOW_MS) {
            drawVolume();
            return;
        }
        gVolumeOverlay = false; // время показа истекло — обычный режим
    }

    if (gDisplayMode == DISPLAY_FREQ) {
        if (gRadioFound) {
            drawFrequency();
        } else {
            drawRadioErrorIndicator();
        }
    } else {
        if (gRtcFound) {
            drawClock();
        } else {
            drawErrorIndicator();
        }
    }
}

/* ------------------------------------------------------------
 *  Применить системное время (полученное по NTP) к PCF8563.
 *  Возвращает false, пока достоверное сетевое время не получено.
 *  При успехе выставляет gTimeSynced = true.
 * ------------------------------------------------------------ */
bool applyNtpTime() {
    time_t utc = time(nullptr);

    if (utc < NTP_VALID_EPOCH) {
        return false;
    }

    // NTP даёт UTC; прибавляем часовой пояс и раскладываем на компоненты.
    time_t    local = utc + GMT_OFFSET_SEC;
    struct tm t;
    gmtime_r(&local, &t);

    DateTime dt(t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
                t.tm_hour, t.tm_min, t.tm_sec);
    rtc.adjust(dt);
    gTimeSynced = true;

    Serial.printf("[RTC] set from NTP: %04u-%02u-%02u %02u:%02u:%02u\n",
                  dt.year(), dt.month(), dt.day(),
                  dt.hour(), dt.minute(), dt.second());
    return true;
}

/* ------------------------------------------------------------
 *  Синхронизация PCF8563 с NTP (не блокирует loop()).
 *  Возвращает true, когда синхронизация завершена (или истёк
 *  таймаут — тогда оставляем текущее время PCF8563 как есть).
 * ------------------------------------------------------------ */
bool syncRtcFromNtp() {
    if (applyNtpTime()) {
        return true;
    }

    if (millis() - gSyncStartMs > SYNC_TIMEOUT_MS) {
        Serial.println("[NTP] sync timeout — using current PCF8563 time");
        return true;
    }

    return false;
}

/* ------------------------------------------------------------
 *  Запустить очередную попытку подключения к Wi-Fi.
 *  Счётчик попыток и момент старта используются в loop().
 * ------------------------------------------------------------ */
void startWifiAttempt() {
    WiFi.disconnect();
    delay(10);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    gWifiAttempts++;
    gWifiAttemptMs = millis();
    Serial.printf("[WiFi] attempt %u/%u to \"%s\"...\n",
                  gWifiAttempts, WIFI_MAX_ATTEMPTS, WIFI_SSID);
}

/* ------------------------------------------------------------
 *  Wi-Fi подключён: перейти к синхронизации PCF8563 с NTP.
 * ------------------------------------------------------------ */
void beginTimeSync() {
    gState       = STATE_SYNCING;
    gSyncStartMs = millis();
    gSntpArmed   = true;
    configTime(0, 0, NTP_SERVER_1, NTP_SERVER_2);
    Serial.printf("[WiFi] connected, IP: %s\n",
                  WiFi.localIP().toString().c_str());
    Serial.println("[NTP] syncing PCF8563...");
}

/* ------------------------------------------------------------
 *  Фоновая поддержка сети (вызывается в STATE_RUNNING, не мешая
 *  показу времени):
 *    - если связь есть, но RTC ещё не синхронизирован — тихо
 *      догоняем время по NTP;
 *    - если связи нет — периодически пробуем переподключиться.
 * ------------------------------------------------------------ */
void serviceNetwork() {
    if (WiFi.status() == WL_CONNECTED) {
        if (!gTimeSynced) {
            // Связь есть, но время ещё не синхронизировано: при первом
            // появлении сети в фоне заново запускаем NTP.
            if (!gSntpArmed) {
                gSyncStartMs = millis();
                gSntpArmed   = true;
                configTime(0, 0, NTP_SERVER_1, NTP_SERVER_2);
                Serial.println("[NTP] background sync started");
            }
            if (applyNtpTime()) {
                gLastDrawMs = 0; // обновить часы после коррекции времени
            }
        }
        return;
    }

    // Связи нет: при следующем подключении запустим NTP заново.
    gSntpArmed = false;

    if (millis() - gLastReconnectMs >= WIFI_RECONNECT_MS) {
        gLastReconnectMs = millis();
        gWifiAttempts    = 0;
        startWifiAttempt();
        Serial.println("[WiFi] background reconnect attempt...");
    }
}

// Прототип: определение ниже (используется при изменении состояния).
void notifyStateChanged();

/* ------------------------------------------------------------
 *  Ручной режим: заливка всей матрицы текущим цветом/яркостью.
 * ------------------------------------------------------------ */
void applyManualColor() {
    fill_solid(leds, NUM_LEDS, gColor);
    FastLED.setBrightness(gBrightness);
    FastLED.show();
}

/* ------------------------------------------------------------
 *  Переключить индикацию «часы ⇄ частота».
 *  Используется и кнопкой энкодера, и командой WebSocket.
 * ------------------------------------------------------------ */
void toggleDisplayMode() {
    gDisplayMode   = (gDisplayMode == DISPLAY_TIME) ? DISPLAY_FREQ : DISPLAY_TIME;
    gVolumeOverlay = false; // переключение отменяет показ шкалы громкости
    gLastDrawMs    = 0;     // перерисовать немедленно
    Serial.printf("[UI] display -> %s\n",
                  gDisplayMode == DISPLAY_TIME ? "TIME" : "FREQ");
    notifyStateChanged(); // сообщить клиентам WebSocket
}

// Прототип: определение ниже (используется командами next / prev).
void seekStation(uint8_t direction);

/* ------------------------------------------------------------
 *  Сформировать JSON с текущими настройками и состоянием.
 *  Используется как ответ на команду {"cmd":"get"} (алиас "status")
 *  и как приветствие при подключении клиента.
 * ------------------------------------------------------------ */
void buildStateJson(String& out) {
    JsonDocument doc;
    doc["cmd"] = "state";

    // Индикатор
    doc["brightness"] = gBrightness;
    doc["manual"]     = gManualMode;
    doc["display"]    = (gDisplayMode == DISPLAY_TIME) ? "time" : "freq";

    JsonObject color = doc["color"].to<JsonObject>();
    color["r"] = gColor.r;
    color["g"] = gColor.g;
    color["b"] = gColor.b;

    JsonObject clockColor = doc["clock_color"].to<JsonObject>();
    clockColor["r"] = gClockColor.r;
    clockColor["g"] = gClockColor.g;
    clockColor["b"] = gClockColor.b;

    JsonObject freqColor = doc["freq_color"].to<JsonObject>();
    freqColor["r"] = gFreqColor.r;
    freqColor["g"] = gFreqColor.g;
    freqColor["b"] = gFreqColor.b;

    // Радио: frequency — МГц для UI, frequency_raw — единицы 10 кГц.
    uint16_t freq = gRadioFound ? rx.getFrequency() : gRadioFreq;
    doc["frequency"]     = freq / 100.0;
    doc["frequency_raw"] = freq;
    doc["volume"]        = gRadioVolume;
    doc["rssi"]          = gRadioFound ? rx.getRssi() : 0;

    // Время и состояние подсистем.
    if (gRtcFound) {
        DateTime now = rtc.now();
        char buf[12];
        snprintf(buf, sizeof(buf), "%02u:%02u:%02u",
                 (unsigned)now.hour(), (unsigned)now.minute(),
                 (unsigned)now.second());
        doc["time"] = buf; // ArduinoJson копирует значение при сериализации
    }
    doc["wifi"]        = (WiFi.status() == WL_CONNECTED);
    doc["time_synced"] = gTimeSynced;
    doc["rtc"]         = gRtcFound;
    doc["radio"]       = gRadioFound;
    doc["eeprom"]      = gEepromReady;
    doc["dirty"]       = gSettingsDirty; // настройки ещё не записаны во flash

    out = "";
    out.reserve(384);   // без лишней фрагментации кучи
    serializeJson(doc, out);
}

/* ------------------------------------------------------------
 *  Отправить состояние всем подключённым клиентам.
 * ------------------------------------------------------------ */
void broadcastState() {
    String out;
    buildStateJson(out);
    webSocket.broadcastTXT(out);
}

/* ------------------------------------------------------------
 *  Отметить изменение состояния: рассылка будет выполнена из loop()
 *  не чаще STATE_PUSH_INTERVAL_MS. Ответ на команду get уходит сразу
 *  и от этой задержки не зависит.
 * ------------------------------------------------------------ */
void notifyStateChanged() {
    gStateDirty   = true;
    gStateDirtyMs = millis();
}

/* ------------------------------------------------------------
 *  Отложенная авторассылка состояния клиентам (вызывается из loop()).
 * ------------------------------------------------------------ */
void serviceStateBroadcast() {
    if (!gStateDirty) {
        return;
    }
    if (millis() - gStateDirtyMs < STATE_PUSH_INTERVAL_MS) {
        return;
    }
    gStateDirty = false;
    broadcastState();
}

/* ------------------------------------------------------------
 *  Настроить приёмник на заданную частоту (единицы 10 кГц).
 *  Частота выравнивается по сетке 0.1 МГц и ограничивается
 *  FM-диапазоном 87.5..108.0 МГц. Новая станция сохраняется
 *  в EEPROM через SAVE_DELAY_MS (см. markSettingsChanged()).
 * ------------------------------------------------------------ */
void setRadioFrequency(uint16_t freq) {
    if (!gRadioFound) {
        Serial.println("[RADIO] not found — frequency ignored");
        return;
    }

    if (freq < 8750)  freq = 8750;   // FM-диапазон 87.5..108.0 МГц
    if (freq > 10800) freq = 10800;
    freq = (uint16_t)(((freq + 5) / 10) * 10); // шаг сетки 0.1 МГц

    rx.setFrequency(freq);
    gRadioFreq     = rx.getFrequency();
    gDisplayMode   = DISPLAY_FREQ; // показать частоту на матрице
    gVolumeOverlay = false;
    gLastDrawMs    = 0;

    // В ручном режиме матрицу не перерисовываем: смена индикации
    // станет видна после команды "auto".
    if (!gManualMode) {
        drawFrequency();
        gLastDrawMs = millis();
    }

    markSettingsChanged(); // сохранится через SAVE_DELAY_MS
    notifyStateChanged();  // и разошлём новое состояние клиентам WebSocket

    Serial.printf("[RADIO] frequency set to %u.%u MHz\n",
                  (unsigned)(gRadioFreq / 100),
                  (unsigned)((gRadioFreq / 10) % 10));
}

/* ------------------------------------------------------------
 *  Обработчик событий WebSocket.
 *  Команды (JSON):
 *    {"cmd":"color","r":..,"g":..,"b":..}       — залить матрицу цветом;
 *    {"cmd":"brightness","value":..}            — яркость матрицы;
 *    {"cmd":"volume","value":..}                — громкость приёмника;
 *    {"cmd":"frequency","value":103.9}          — частота приёмника
 *                                                 (МГц или единицы 10 кГц);
 *    {"cmd":"next"} / {"cmd":"prev"}             — следующая/предыдущая станция;
 *    {"cmd":"toggle"}                            — индикация «часы ⇄ частота»;
 *    {"cmd":"clock_color","r":..,"g":..,"b":..} — цвет цифр часов;
 *    {"cmd":"freq_color","r":..,"g":..,"b":..}  — цвет цифр частоты;
 *    {"cmd":"get"} / {"cmd":"status"}           — прислать текущие настройки;
 *    {"cmd":"on"}, {"cmd":"off"}, {"cmd":"auto"}.
 *  "auto" возвращает автоматический режим индикатора.
 *  Ответы: при подключении — {"status":"connected"} и снимок состояния
 *  {"cmd":"state",...}; на get/status — снимок состояния; после изменения
 *  настроек — авторассылка состояния всем подключённым клиентам.
 * ------------------------------------------------------------ */
void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
    switch (type) {
        case WStype_DISCONNECTED:
            Serial.printf("[WS] client #%u disconnected\n", num);
            break;

        case WStype_CONNECTED: {
            IPAddress ip = webSocket.remoteIP(num);
            Serial.printf("[WS] client #%u connected from %d.%d.%d.%d\n",
                          num, ip[0], ip[1], ip[2], ip[3]);
            webSocket.sendTXT(num, "{\"status\":\"connected\"}");

            // Сразу отдаём текущие настройки, чтобы UI не ждал запроса.
            String out;
            buildStateJson(out);
            webSocket.sendTXT(num, out);
            break;
        }

        case WStype_TEXT: {
            JsonDocument doc;
            DeserializationError error = deserializeJson(doc, payload, length);

            if (error) {
                Serial.printf("[WS] JSON parse error: %s\n", error.c_str());
                break;
            }

            const char* cmd = doc["cmd"] | "";

            if (strcmp(cmd, "color") == 0) {
                uint8_t r = doc["r"] | 0;
                uint8_t g = doc["g"] | 0;
                uint8_t b = doc["b"] | 0;
                gColor = CRGB(r, g, b);
                gManualMode = true;
                applyManualColor();
                notifyStateChanged();
                Serial.printf("[WS] color set to (%u, %u, %u)\n", r, g, b);
            }
            else if (strcmp(cmd, "brightness") == 0) {
                gBrightness = doc["value"] | gBrightness;
                FastLED.setBrightness(gBrightness);
                FastLED.show();
                markSettingsChanged(); // сохранится через SAVE_DELAY_MS
                notifyStateChanged();
                Serial.printf("[WS] brightness set to %u\n", gBrightness);
            }
            else if (strcmp(cmd, "volume") == 0) {
                uint8_t v = doc["value"] | gRadioVolume;
                if (v > 15) {
                    v = 15;
                }
                gRadioVolume = v;
                if (gRadioFound) {
                    rx.setVolume(gRadioVolume);
                }
                markSettingsChanged(); // сохранится через SAVE_DELAY_MS
                notifyStateChanged();
                Serial.printf("[WS] volume set to %u\n", gRadioVolume);
            }
            else if (strcmp(cmd, "frequency") == 0) {
                if (doc["value"].isNull()) {
                    Serial.println("[WS] frequency: missing \"value\"");
                } else {
                    double v = doc["value"].as<double>();
                    // 103.9 — МГц; 10390 — единицы 10 кГц (как RADIO_DEFAULT_FREQ).
                    uint16_t freq = (v > 200.0) ? (uint16_t)(v + 0.5)
                                                : (uint16_t)(v * 100.0 + 0.5);
                    setRadioFrequency(freq);
                }
            }
            else if (strcmp(cmd, "next") == 0) {
                seekStation(RDA_SEEK_UP);
            }
            else if (strcmp(cmd, "prev") == 0) {
                seekStation(RDA_SEEK_DOWN);
            }
            else if (strcmp(cmd, "toggle") == 0) {
                toggleDisplayMode();
            }
            else if (strcmp(cmd, "clock_color") == 0) {
                // Неуказанные компоненты сохраняют текущее значение.
                uint8_t r = doc["r"] | gClockColor.r;
                uint8_t g = doc["g"] | gClockColor.g;
                uint8_t b = doc["b"] | gClockColor.b;
                gClockColor = CRGB(r, g, b);
                gLastDrawMs = 0; // перерисовать часы немедленно (в авторежиме)
                markSettingsChanged();
                notifyStateChanged();
                Serial.printf("[WS] clock color set to (%u, %u, %u)\n", r, g, b);
            }
            else if (strcmp(cmd, "freq_color") == 0) {
                uint8_t r = doc["r"] | gFreqColor.r;
                uint8_t g = doc["g"] | gFreqColor.g;
                uint8_t b = doc["b"] | gFreqColor.b;
                gFreqColor = CRGB(r, g, b);
                gLastDrawMs = 0; // перерисовать частоту немедленно (в авторежиме)
                markSettingsChanged();
                notifyStateChanged();
                Serial.printf("[WS] freq color set to (%u, %u, %u)\n", r, g, b);
            }
            else if (strcmp(cmd, "get") == 0 || strcmp(cmd, "status") == 0) {
                // Ответ отправляется только запросившему клиенту и сразу,
                // без задержки авторассылки.
                String out;
                buildStateJson(out);
                webSocket.sendTXT(num, out);
                Serial.printf("[WS] state sent to client #%u (%u bytes)\n",
                              num, (unsigned)out.length());
            }
            else if (strcmp(cmd, "off") == 0) {
                gManualMode = true;
                FastLED.clear(true);
                notifyStateChanged();
                Serial.println("[WS] LEDs off");
            }
            else if (strcmp(cmd, "on") == 0) {
                gManualMode = true;
                applyManualColor();
                notifyStateChanged();
                Serial.println("[WS] LEDs on");
            }
            else if (strcmp(cmd, "auto") == 0) {
                gManualMode = false;
                gLastDrawMs = 0; // немедленно вернуться к отображению часов/индикатора
                notifyStateChanged();
                Serial.println("[WS] auto indicator mode");
            }
            else {
                Serial.printf("[WS] unknown command: %s\n", cmd);
            }
            break;
        }

        case WStype_BIN:
            Serial.printf("[WS] binary message, %u bytes\n", (unsigned)length);
            break;

        default:
            break;
    }
}

/* ------------------------------------------------------------
 *  Проверка наличия RDA5807 на шине I2C (адреса 0x10 / 0x11).
 * ------------------------------------------------------------ */
bool radioDetect() {
    Wire.beginTransmission(I2C_ADDR_FULL_ACCESS);
    if (Wire.endTransmission() == 0) {
        return true;
    }
    Wire.beginTransmission(I2C_ADDR_DIRECT_ACCESS);
    return (Wire.endTransmission() == 0);
}

/* ------------------------------------------------------------
 *  Поиск FM-станции в заданном направлении и вывод её частоты.
 *  Вызывается энкодером и командами WebSocket next / prev.
 * ------------------------------------------------------------ */
void seekStation(uint8_t direction) {
    if (!gRadioFound) {
        Serial.println("[RADIO] not found — seek ignored");
        return;
    }

    // Блокирующий поиск; по завершении библиотека фиксирует найденную
    // частоту, которую затем возвращает getFrequency().
    rx.seek(RDA_SEEK_WRAP, direction, NULL);

    gDisplayMode   = DISPLAY_FREQ; // сразу показать найденную частоту
    gVolumeOverlay = false;
    gLastDrawMs    = 0;

    // В ручном режиме матрицу не перерисовываем: смена индикации
    // станет видна после команды "auto".
    if (!gManualMode) {
        drawFrequency();
        gLastDrawMs = millis();
    }

    uint16_t f = rx.getFrequency();
    Serial.printf("[RADIO] seek %s -> %u.%u MHz (RSSI %d)\n",
                  direction == RDA_SEEK_UP ? "UP" : "DOWN",
                  (unsigned)(f / 100), (unsigned)((f / 10) % 10),
                  rx.getRssi());

    // Станция изменилась — планируем отложенную запись настроек в EEPROM.
    if (f != gRadioFreq) {
        gRadioFreq = f;
        markSettingsChanged();
    }
    notifyStateChanged(); // режим индикации / частота изменились
}

/* ------------------------------------------------------------
 *  Изменить громкость приёмника на delta (+1 / -1).
 *  Вызывается при удержании кнопки энкодера и повороте.
 *  Новое значение показывается на матрице и сохраняется в
 *  EEPROM через SAVE_DELAY_MS (см. markSettingsChanged()).
 * ------------------------------------------------------------ */
void adjustVolume(int8_t delta) {
    if (!gRadioFound) {
        return;
    }

    int v = (int)gRadioVolume + delta;
    if (v < 0)  v = 0;
    if (v > 15) v = 15;
    if (v == gRadioVolume) {
        return; // уже на пределе
    }

    gRadioVolume = (uint8_t)v;
    rx.setVolume(gRadioVolume);
    markSettingsChanged();
    notifyStateChanged(); // сообщить клиентам WebSocket

    gVolumeOverlay   = true; // показать шкалу громкости
    gVolumeOverlayMs = millis();
    gLastDrawMs      = 0;    // перерисовать немедленно

    Serial.printf("[ENC] volume -> %u\n", gRadioVolume);
}

/* ------------------------------------------------------------
 *  Обработка энкодера:
 *    - поворот                    — поиск станции;
 *    - удержание кнопки + поворот — громкость;
 *    - короткий клик кнопки       — индикация «часы / частота».
 * ------------------------------------------------------------ */
void handleEncoder() {
    enc.tick();

    if (enc.isRightH()) {
        adjustVolume(+1);
    } else if (enc.isLeftH()) {
        adjustVolume(-1);
    } else if (enc.isRight()) {
        seekStation(RDA_SEEK_UP);
    } else if (enc.isLeft()) {
        seekStation(RDA_SEEK_DOWN);
    }

    if (enc.isClick()) {
        toggleDisplayMode();
    }
}

/* ------------------------------------------------------------
 *  Инициализация всех подсистем.
 * ------------------------------------------------------------ */
void setup() {
    // 1. Последовательный порт для отладки.
    Serial.begin(115200);
    delay(100);
    Serial.println("\n\nKitchenWatch starting...");

    // 2. LED-матрица (FastLED).
    FastLED.addLeds<WS2812B, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS)
           .setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(gBrightness);
    FastLED.clear(true);
    Serial.printf("[LED] %d LEDs (%d panels of %dx%d) on GPIO%d\n",
                  NUM_LEDS, PANEL_COUNT, PANEL_SIZE, PANEL_SIZE, LED_PIN);

    // 3. Часы реального времени PCF8563 (I2C, адрес 0x51).
    gRtcFound = rtc.begin();
    if (!gRtcFound) {
        Serial.println("[RTC] PCF8563 not found! Check wiring (SDA=D2, SCL=D1).");
    } else {
        // STOP-бит (бит 5 регистра Control_1): после первого включения
        // генератор часов PCF8563 остановлен — его нужно запустить.
        if (!rtc.isrunning()) {
            rtc.start();
            Serial.println("[RTC] PCF8563 was stopped — clock started.");
        }

        // Флаг VL (Voltage Low) взводится при просадке питания и сбрасывается
        // только записью времени: если он установлен, время недостоверно.
        if (rtc.lostPower()) {
            Serial.println("[RTC] PCF8563 lost power (VL flag) — loading compile time, will correct via NTP.");
            rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        } else {
            DateTime now = rtc.now();
            Serial.printf("[RTC] PCF8563 OK, current: %04u-%02u-%02u %02u:%02u:%02u\n",
                          now.year(), now.month(), now.day(),
                          now.hour(), now.minute(), now.second());
        }
    }

    // 4. Энергонезависимая память, FM-приёмник RDA5807 и энкодер.
    Wire.begin(); // SDA=D2, SCL=D1 (уже инициализирована для PCF8563)

    // 4a. Энергонезависимая память — эмуляция EEPROM во flash ESP8266.
    if (eepromInit()) {
        Serial.printf("[EEPROM] flash emulation ready (%u bytes).\n", (unsigned)EE_SIZE);
        loadSettings(); // яркость, станция, громкость и цвета цифр
        FastLED.setBrightness(gBrightness); // применить яркость из flash
    } else {
        Serial.println("[EEPROM] init failed — settings will not be persisted.");
    }

    // 4b. FM-приёмник.
    gRadioFound = radioDetect();
    if (gRadioFound) {
        rx.setup();                  // включение и инициализация приёмника
        rx.setMute(false);           // снять мут
        rx.setVolume(gRadioVolume);  // громкость из EEPROM или по умолчанию
        rx.setFrequency(gRadioFreq); // станция из EEPROM или по умолчанию
        Serial.printf("[RADIO] RDA5807 ready (%u.%u MHz, vol %u)\n",
                      (unsigned)(gRadioFreq / 100),
                      (unsigned)((gRadioFreq / 10) % 10),
                      gRadioVolume);
    } else {
        Serial.println("[RADIO] RDA5807 not found on I2C! Check SDA=D2, SCL=D1.");
    }

    enc.setType(TYPE2); // один щелчок энкодера = один шаг
    Serial.println("[ENC] encoder ready (CLK=D6, DT=D7, SW=D5)");

    // 5. Wi-Fi: стартуем первую попытку подключения. Повторные попытки
    //    и анимацию обрабатывает loop(); при полном отказе сети часы
    //    работают автономно от PCF8563.
    WiFi.mode(WIFI_STA);
    startWifiAttempt();

    // 6. WebSocket-сервер.
    webSocket.begin();
    webSocket.onEvent(onWebSocketEvent);
    Serial.printf("[WS] server started on port %d\n", WEBSOCKET_PORT);

    Serial.println("KitchenWatch ready");
}

/* ------------------------------------------------------------
 *  Главный цикл.
 * ------------------------------------------------------------ */
void loop() {
    // Диагностика разводки матрицы: при MATRIX_TEST_MODE=1 показываем
    // «прогон индексов» вместо обычного индикатора.
    if (MATRIX_TEST_MODE) {
        runMatrixTest();
        return;
    }

    // Обработка WebSocket-событий (приём команд).
    webSocket.loop();

    // Отложенная запись настроек в EEPROM (через SAVE_DELAY_MS после изменения).
    serviceSettingsSave();

    // Авторассылка состояния клиентам WebSocket после изменений
    // (не чаще STATE_PUSH_INTERVAL_MS, см. notifyStateChanged()).
    serviceStateBroadcast();

    // В ручном режиме дисплеем управляет пользователь.
    if (gManualMode) {
        return;
    }

    switch (gState) {
        case STATE_CONNECTING:
            if (WiFi.status() == WL_CONNECTED) {
                // Подключились — запускаем синхронизацию PCF8563 с NTP.
                beginTimeSync();
            } else if (millis() - gWifiAttemptMs >= WIFI_ATTEMPT_TIMEOUT_MS) {
                // Текущая попытка не удалась: либо повторяем, либо,
                // если попытки исчерпаны, работаем без сети.
                if (gWifiAttempts < WIFI_MAX_ATTEMPTS) {
                    Serial.printf("[WiFi] attempt %u failed, retrying...\n",
                                  gWifiAttempts);
                    startWifiAttempt();
                } else {
                    Serial.println("[WiFi] network unavailable after all attempts — "
                                   "showing PCF8563 time without sync");
                    gState           = STATE_RUNNING;
                    gLastReconnectMs = millis(); // первую фоновую попытку — через период
                    gLastDrawMs      = 0;        // сразу показать время из PCF8563
                }
            } else {
                drawConnectingAnimation();
            }
            break;

        case STATE_SYNCING:
            if (syncRtcFromNtp()) {
                gState      = STATE_RUNNING;
                gLastDrawMs = 0; // немедленно нарисовать часы
            } else {
                drawConnectingAnimation(); // продолжаем «рабочую» анимацию
            }
            break;

        case STATE_RUNNING:
            handleEncoder();
            serviceNetwork(); // фоновая поддержка сети и синхронизации
            if (millis() - gLastDrawMs >= CLOCK_REFRESH_MS) {
                gLastDrawMs = millis();
                drawCurrentDisplay();
            }
            break;
    }
}
