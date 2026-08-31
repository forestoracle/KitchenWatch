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
 *    - Wire + RTClib     — часы реального времени DS1307 (I2C)
 *
 *  Индикатор: 4 панели 8x8, выстроенные в один горизонтальный ряд
 *  (общее поле 32x8). На старте показывается анимация подключения
 *  к Wi-Fi. После подключения время DS1307 синхронизируется с
 *  NTP-сервером, и на панелях отображаются текущие часы (ЧЧ:ММ).
 *  Зелёная точка в правом верхнем углу сигнализирует о том, что
 *  Wi-Fi подключён.
 * ============================================================ */

// ---------- Настройки Wi-Fi ----------
const char* WIFI_SSID     = "YOUR_WIFI_SSID";      // <-- укажите имя сети
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";  // <-- укажите пароль

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
const bool kMatrixSerpentineLayout = true;

// ---------- Ориентация дисплея ----------
// Если изображение зеркальное/перевёрнутое, меняйте нужные флаги (0/1),
// не трогая логику XY().
#define FLIP_X              0   // 1 = зеркально по горизонтали (лево <-> право)
#define FLIP_Y              0   // 1 = зеркально по вертикали (верх <-> низ)
#define REVERSE_PANEL_ORDER 0   // 1 = первая панель цепочки физически справа

// ---------- Часы реального времени DS1307 (I2C) ----------
// NodeMCU v2: SDA = GPIO4 (D2), SCL = GPIO5 (D1) — пины Wire по умолчанию.
RTC_DS1307 rtc;

// ---------- FM-приёмник RDA5807 (I2C) ----------
// Модуль подключается к той же шине I2C, что и DS1307 (SDA=D2, SCL=D1).
// Адреса чипа: 0x10 (полный доступ) и 0x11 (доступ к одному регистру).
RDA5807 rx;

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
#define SYNC_TIMEOUT_MS   15000        // сколько ждать NTP до перехода на показания DS1307

// Цвет цифр часов и период обновления кадра.
#define CLOCK_COLOR       CRGB::White
#define CLOCK_REFRESH_MS  500          // перерисовываем часы 2 раза в секунду

CRGB leds[NUM_LEDS];

// ---------- WebSocket-сервер ----------
#define WEBSOCKET_PORT 81
WebSocketsServer webSocket(WEBSOCKET_PORT);

// ---------- Текущее состояние ----------
uint8_t gBrightness = 64;
CRGB    gColor      = CRGB::White;
bool    gManualMode = false; // true = дисплеем управляет пользователь (JSON)

enum AppState {
    STATE_CONNECTING, // Wi-Fi ещё не подключён — показываем анимацию
    STATE_SYNCING,    // Wi-Fi подключён — ждём синхронизации DS1307 с NTP
    STATE_RUNNING     // часы работают (читаем время из DS1307)
};
AppState      gState       = STATE_CONNECTING;
unsigned long gSyncStartMs = 0;   // момент начала синхронизации с NTP
unsigned long gLastDrawMs  = 0;   // время последней перерисовки часов
bool          gRtcFound    = false; // true = DS1307 найден на шине I2C

enum DisplayMode {
    DISPLAY_TIME,  // часы
    DISPLAY_FREQ   // частота FM-станции
};
DisplayMode gDisplayMode = DISPLAY_TIME;

bool          gRadioFound     = false; // true = RDA5807 найден на шине I2C

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
 *  Отображение текущего времени ЧЧ:ММ из DS1307.
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
 *  Индикатор ошибки: DS1307 не найден на шине I2C.
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
 *  Отрисовка текущего режима индикации (часы или частота станции).
 * ------------------------------------------------------------ */
void drawCurrentDisplay() {
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
 *  Синхронизация DS1307 с NTP (не блокирует loop()).
 *  Возвращает true, когда синхронизация завершена (или истёк
 *  таймаут — тогда оставляем текущее время DS1307 как есть).
 * ------------------------------------------------------------ */
bool syncRtcFromNtp() {
    time_t utc = time(nullptr);

    if (utc >= NTP_VALID_EPOCH) {
        // NTP даёт UTC; прибавляем часовой пояс и раскладываем на компоненты.
        time_t    local = utc + GMT_OFFSET_SEC;
        struct tm t;
        gmtime_r(&local, &t);

        DateTime dt(t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
                    t.tm_hour, t.tm_min, t.tm_sec);
        rtc.adjust(dt);

        Serial.printf("[RTC] set from NTP: %04u-%02u-%02u %02u:%02u:%02u\n",
                      dt.year(), dt.month(), dt.day(),
                      dt.hour(), dt.minute(), dt.second());
        return true;
    }

    if (millis() - gSyncStartMs > SYNC_TIMEOUT_MS) {
        Serial.println("[NTP] sync timeout — using current DS1307 time");
        return true;
    }

    return false;
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
}

/* ------------------------------------------------------------
 *  Обработка энкодера: поворот — поиск станции, кнопка —
 *  переключение индикации «часы / частота станции».
 * ------------------------------------------------------------ */
void handleEncoder() {
    enc.tick();

    if (enc.isRight()) {
        seekStation(RDA_SEEK_UP);
    } else if (enc.isLeft()) {
        seekStation(RDA_SEEK_DOWN);
    }

    if (enc.isClick()) {
        gDisplayMode = (gDisplayMode == DISPLAY_TIME) ? DISPLAY_FREQ : DISPLAY_TIME;
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

    // 3. Часы реального времени DS1307 (I2C).
    gRtcFound = rtc.begin();
    if (!gRtcFound) {
        Serial.println("[RTC] DS1307 not found! Check wiring (SDA=D2, SCL=D1).");
    } else if (!rtc.isrunning()) {
        Serial.println("[RTC] DS1307 not running (lost power) — loading compile time, will correct via NTP.");
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    } else {
        DateTime now = rtc.now();
        Serial.printf("[RTC] DS1307 OK, current: %04u-%02u-%02u %02u:%02u:%02u\n",
                      now.year(), now.month(), now.day(),
                      now.hour(), now.minute(), now.second());
    }

    // 4. FM-приёмник RDA5807 и энкодер.
    Wire.begin(); // SDA=D2, SCL=D1 (уже инициализирована для DS1307)
    gRadioFound = radioDetect();
    if (gRadioFound) {
        rx.setup();                 // включение и инициализация приёмника
        rx.setMute(false);          // снять мут
        rx.setVolume(8);            // громкость 0..15
        rx.setFrequency(10390);     // стартовая станция 103.9 МГц
        Serial.println("[RADIO] RDA5807 ready (103.9 MHz)");
    } else {
        Serial.println("[RADIO] RDA5807 not found on I2C! Check SDA=D2, SCL=D1.");
    }

    enc.setType(TYPE2); // один щелчок энкодера = один шаг
    Serial.println("[ENC] encoder ready (CLK=D6, DT=D7, SW=D5)");

    // 5. Wi-Fi: стартуем асинхронно, анимацию отрисовывает loop().
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.printf("[WiFi] connecting to %s...\n", WIFI_SSID);

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
    // Обработка WebSocket-событий (приём команд).
    webSocket.loop();

    // В ручном режиме дисплеем управляет пользователь.
    if (gManualMode) {
        return;
    }

    switch (gState) {
        case STATE_CONNECTING:
            if (WiFi.status() != WL_CONNECTED) {
                drawConnectingAnimation();
            } else {
                // Подключились — запускаем синхронизацию DS1307 с NTP.
                gState = STATE_SYNCING;
                gSyncStartMs = millis();
                configTime(0, 0, NTP_SERVER_1, NTP_SERVER_2);
                Serial.printf("[WiFi] connected, IP: %s\n",
                              WiFi.localIP().toString().c_str());
                Serial.println("[NTP] syncing DS1307...");
            }
            break;

        case STATE_SYNCING:
            if (syncRtcFromNtp()) {
                gState = STATE_RUNNING;
                gLastDrawMs = 0; // немедленно нарисовать часы
            } else {
                drawConnectingAnimation(); // продолжаем «рабочую» анимацию
            }
            break;

        case STATE_RUNNING:
            handleEncoder();
            if (millis() - gLastDrawMs >= CLOCK_REFRESH_MS) {
                gLastDrawMs = millis();
                drawCurrentDisplay();
            }
            break;
    }
}
