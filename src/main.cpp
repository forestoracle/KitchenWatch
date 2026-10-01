#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <FastLED.h>
#include <Wire.h>
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
 *  Настройки радио (станция и громкость) хранятся в EEPROM AT24C02
 *  (если микросхема подключена к шине I2C) и записываются через 10
 *  секунд после последнего изменения — это экономит ресурс ячеек.
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

// ---------- Энергонезависимая память AT24C02 (I2C) ----------
// Микросхема 2 Кбит = 256 байт, адрес 0x50 (при A0..A2 = 0). Подключена к той
// же шине I2C, что и PCF8563 / RDA5807 (SDA=D2, SCL=D1). Настройки (станция и
// громкость) записываются через SAVE_DELAY_MS после последнего изменения,
// чтобы не изнашивать ресурс ячеек памяти.
#define AT24C02_ADDR      0x50
#define AT24C02_WRITE_MS  6      // время цикла записи tWR для AT24C02 (~5 мс)

#define EE_ADDR_MAGIC     0x00   // сигнатура «настройки записаны»
#define EE_ADDR_FREQ      0x01   // 2 байта: частота в единицах 10 кГц
#define EE_ADDR_VOLUME    0x03   // 1 байт: громкость 0..15
#define EE_MAGIC          0x4B   // 'K' — KitchenWatch

#define SAVE_DELAY_MS     10000  // пауза после изменения до записи в EEPROM, мс

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

// Цвет цифр часов и период обновления кадра.
#define CLOCK_COLOR       CRGB::Green
#define CLOCK_REFRESH_MS  500          // перерисовываем часы 2 раза в секунду

CRGB leds[NUM_LEDS];

// ---------- WebSocket-сервер ----------
#define WEBSOCKET_PORT 81
WebSocketsServer webSocket(WEBSOCKET_PORT);

// ---------- Текущее состояние ----------
uint8_t gBrightness = 64;
CRGB    gColor      = CRGB::Green;
bool    gManualMode = false; // true = дисплеем управляет пользователь (JSON)

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

bool          gEepromFound       = false; // true = AT24C02 найден на шине I2C
bool          gSettingsDirty     = false; // true = настройки изменены, ждём запись
unsigned long gSettingsChangedMs = 0;     // момент последнего изменения настроек

/* ------------------------------------------------------------
 *  AT24C02 (I2C EEPROM): определение, чтение и запись байт.
 *  Адрес микросхемы — AT24C02_ADDR (0x50 при A0..A2 = 0).
 * ------------------------------------------------------------ */
bool eepromDetect() {
    Wire.beginTransmission(AT24C02_ADDR);
    return (Wire.endTransmission() == 0);
}

// Чтение одного байта по адресу addr; false — ошибка шины.
bool eepromReadByte(uint8_t addr, uint8_t& value) {
    Wire.beginTransmission(AT24C02_ADDR);
    Wire.write(addr);
    if (Wire.endTransmission() != 0) {
        return false;
    }
    if (Wire.requestFrom(AT24C02_ADDR, 1) != 1) {
        return false;
    }
    value = (uint8_t)Wire.read();
    return true;
}

// Запись одного байта; после записи выдерживается цикл tWR.
void eepromWriteByte(uint8_t addr, uint8_t value) {
    Wire.beginTransmission(AT24C02_ADDR);
    Wire.write(addr);
    Wire.write(value);
    Wire.endTransmission();
    delay(AT24C02_WRITE_MS);
}

/* ------------------------------------------------------------
 *  Прочитать сохранённые станцию и громкость из AT24C02.
 *  При отсутствии/некорректности данных остаются значения по
 *  умолчанию (RADIO_DEFAULT_FREQ / RADIO_DEFAULT_VOLUME).
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

    Serial.printf("[EEPROM] loaded: freq=%u vol=%u\n", gRadioFreq, gRadioVolume);
}

/* ------------------------------------------------------------
 *  Записать текущие станцию и громкость в AT24C02.
 *  Сигнатура пишется последней: при сбое питания посреди записи
 *  она не окажется выставленной раньше самих данных.
 * ------------------------------------------------------------ */
void saveSettings() {
    if (!gEepromFound) {
        return;
    }
    uint16_t freq = gRadioFound ? rx.getFrequency() : gRadioFreq;
    uint8_t  vol  = gRadioVolume;

    eepromWriteByte(EE_ADDR_FREQ,     (uint8_t)(freq >> 8));
    eepromWriteByte(EE_ADDR_FREQ + 1, (uint8_t)(freq & 0xFF));
    eepromWriteByte(EE_ADDR_VOLUME,   vol);
    eepromWriteByte(EE_ADDR_MAGIC,    EE_MAGIC);

    gRadioFreq = freq;
    Serial.printf("[EEPROM] saved: freq=%u vol=%u\n", freq, vol);
}

/* ------------------------------------------------------------
 *  Отметить, что настройки изменились: запись в EEPROM будет
 *  выполнена через SAVE_DELAY_MS после последнего изменения.
 * ------------------------------------------------------------ */
void markSettingsChanged() {
    gSettingsDirty     = true;
    gSettingsChangedMs = millis();
}

/* ------------------------------------------------------------
 *  Отложенная (через SAVE_DELAY_MS) запись настроек в EEPROM.
 *  Вызывается из loop(); ничего не делает, пока пауза не истекла.
 * ------------------------------------------------------------ */
void serviceSettingsSave() {
    if (!gSettingsDirty || !gEepromFound) {
        return;
    }
    if (millis() - gSettingsChangedMs >= SAVE_DELAY_MS) {
        saveSettings();
        gSettingsDirty = false;
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

    drawDigit(now.hour() / 10, 0, CLOCK_COLOR);
    drawDigit(now.hour() % 10, 1, CLOCK_COLOR);
    drawDigit(now.minute() / 10, 2, CLOCK_COLOR);
    drawDigit(now.minute() % 10, 3, CLOCK_COLOR);

    // Мигающее двоеточие между часами и минутами (гаснет каждую секунду).
    if ((now.second() & 0x01) == 0) {
        setPixel(15, 2, CLOCK_COLOR);
        setPixel(15, 4, CLOCK_COLOR);
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
        drawDigit((intPart / 100) % 10, 0, CLOCK_COLOR);
        drawDigit((intPart / 10)  % 10, 1, CLOCK_COLOR);
    } else {
        // Частоты ниже 100 МГц — первая панель пустая, цифры сдвинуты вправо.
        drawDigit((intPart / 10) % 10, 1, CLOCK_COLOR);
    }
    drawDigit(intPart % 10, 2, CLOCK_COLOR);
    drawDigit(frac, 3, CLOCK_COLOR);

    // Десятичная точка в правом нижнем углу панели с единицами (панель 2).
    setPixel(2 * PANEL_SIZE + 6, 7, CLOCK_COLOR);
    setPixel(2 * PANEL_SIZE + 7, 7, CLOCK_COLOR);

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

/* ------------------------------------------------------------
 *  Ручной режим: заливка всей матрицы текущим цветом/яркостью.
 * ------------------------------------------------------------ */
void applyManualColor() {
    fill_solid(leds, NUM_LEDS, gColor);
    FastLED.setBrightness(gBrightness);
    FastLED.show();
}

/* ------------------------------------------------------------
 *  Обработчик событий WebSocket.
 *  Команды (JSON): {"cmd":"color","r":..,"g":..,"b":..},
 *                  {"cmd":"brightness","value":..},
 *                  {"cmd":"on"}, {"cmd":"off"}, {"cmd":"auto"}.
 *  "auto" возвращает автоматический режим индикатора.
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
                Serial.printf("[WS] color set to (%u, %u, %u)\n", r, g, b);
            }
            else if (strcmp(cmd, "brightness") == 0) {
                gBrightness = doc["value"] | gBrightness;
                FastLED.setBrightness(gBrightness);
                FastLED.show();
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
                Serial.printf("[WS] volume set to %u\n", gRadioVolume);
            }
            else if (strcmp(cmd, "off") == 0) {
                gManualMode = true;
                FastLED.clear(true);
                Serial.println("[WS] LEDs off");
            }
            else if (strcmp(cmd, "on") == 0) {
                gManualMode = true;
                applyManualColor();
                Serial.println("[WS] LEDs on");
            }
            else if (strcmp(cmd, "auto") == 0) {
                gManualMode = false;
                gLastDrawMs = 0; // немедленно вернуться к отображению часов/индикатора
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
 * ------------------------------------------------------------ */
void seekStation(uint8_t direction) {
    if (!gRadioFound) {
        return;
    }

    // Блокирующий поиск; по завершении библиотека фиксирует найденную
    // частоту, которую затем возвращает getFrequency().
    rx.seek(RDA_SEEK_WRAP, direction, NULL);

    gDisplayMode = DISPLAY_FREQ;  // сразу показать найденную частоту
    drawFrequency();
    gLastDrawMs = millis();

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
        gDisplayMode = (gDisplayMode == DISPLAY_TIME) ? DISPLAY_FREQ : DISPLAY_TIME;
        gVolumeOverlay = false; // клик отменяет показ шкалы громкости
        gLastDrawMs = 0;
        Serial.printf("[UI] display -> %s\n",
                      gDisplayMode == DISPLAY_TIME ? "TIME" : "FREQ");
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

    // 4. EEPROM AT24C02, FM-приёмник RDA5807 и энкодер.
    Wire.begin(); // SDA=D2, SCL=D1 (уже инициализирована для PCF8563)

    // 4a. Энергонезависимая память (если подключена) — читаем настройки.
    gEepromFound = eepromDetect();
    if (gEepromFound) {
        Serial.println("[EEPROM] AT24C02 found on I2C.");
        loadSettings(); // заполняет gRadioFreq / gRadioVolume
    } else {
        Serial.println("[EEPROM] AT24C02 not found — settings will not be persisted.");
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
