

#include <Arduino.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <mcp2515.h>
#include <EEPROM.h>

struct can_frame canMsg;
MCP2515 mcp2515(10);  // CS = D10

#define PERIOD_BUF_MAX 21  // Максимальный размер буфера для медианного фильтра

const uint8_t CALIB_SAMPLE_COUNT = 10;        // Количество измерений при калибровке
const uint16_t SPEED_TIMEOUT_TICKS = 2;       // Таймаут пропадания импульсов (в тиках TIMER1_COMPA)
const uint16_t SPEED_HARD_LIMIT = 140;        // Максимальная скорость (для расчета diffMin)
const uint32_t DEFAULT_SPEED_COEF = 11200UL;  // Дефолтный коэффициент скорости (если EEPROM пустая)
const uint32_t DEFAULT_OCR1A = 3124UL;        // Значение по умолчанию для OCR1A
const int EEPROM_ADDR_COEF = 0;               // Адрес EEPROM коэффициента
const int EEPROM_ADDR_SL = 14;                // Адрес EEPROM отсечки скорости
const int EEPROM_ADDR_WIN = 12;               // Адрес EEPROM для размера окна фильтра
const int EEPROM_ADDR_OCR1A = 4;              // Адрес EEPROM для значения OCR1A
const unsigned long SP_INTERVAL_MS = 250;     // Сервисный режим sp: вывод скорости (каждые 250 мс.)
const unsigned long CALIB_INTERVAL_MS = 500;  // Интервал времени при калибровке в мс.

volatile uint8_t lowerSpeedCutoff = 0;             // Нижняя отсечка скорости (при меньшей скорости отправляется 0)
volatile uint8_t periodIdx = 0;                    // Текущий индекс в буфере периодов
volatile uint8_t diffMinCnt = 0;                   // Счетчик периодов ниже diffMin (для фильтрации шума)
volatile uint16_t lastCaptureTicks = 0;            // Время последнего захвата таймера 1 (для вычисления периода)
volatile uint16_t lastSpeedTick = 0;               // Время последнего захвата в тиках COMPA (для таймаута)
volatile uint16_t compaTick = 0;                   // Счетчик прерываний COMPA (инкрементируется каждые saved_OCR1A тиков)
volatile uint16_t saved_OCR1A = 0;                 // Значение для OCR1A (определяет частоту прерывания COMPA)
volatile uint16_t diff = 0;                        // Последний измеренный период (разница между захватами)
volatile uint16_t periodBuf[PERIOD_BUF_MAX];       // Кольцевой буфер периодов для медианного фильтра
volatile uint16_t diffMin;                         // Минимальный валидный период (рассчитывается из SPEED_HARD_LIMIT)
volatile uint32_t canSpeedKmh = 0;                 // Скорость для отправки по CAN (после всех фильтров и отсечек)
volatile uint32_t speedCoef = DEFAULT_SPEED_COEF;  // Коэффициент преобразования периода в скорость км/ч
volatile bool speedSignalAlive = false;            // Флаг наличия сигнала скорости (не было таймаута)
volatile bool canTxPending = false;                // Флаг необходимости отправки CAN-сообщения
volatile bool periodBufFilled = false;             // Флаг заполненности буфера периодов (первый проход завершен)
volatile bool filterMedian = false;                // Флаг необходимости обработки нового периода в медианном фильтре
bool serviceMode = false;                          // Флаг нахождения в сервисном режиме (управление по UART)
bool showSpeedInService = false;                   // Флаг вывода скорости в сервисном режиме
bool calibrating = false;                          // Флаг активного процесса калибровки
uint8_t periodWinSize = 11;                        // Фактический размер окна медианного фильтра (нечетный)
uint8_t calib_samples_collected = 0;               // Счетчик собранных samples при калибровке
uint32_t calibTargetSpeed = 0;                     // Целевая скорость при калибровке (вводится пользователем)
uint32_t calibSpeedSum = 0;                        // Сумма скоростей samples при калибровке
unsigned long calib_last_ms = 0;                   // Время последнего sample при калибровке
unsigned long lastServicePrintMs = 0;              // Время последнего вывода скорости в сервисном режиме
uint32_t speedRaw = 0;                             // Рассчитанная скорость без фильтрации (для калибровки и отладки)

String uartRxBuffer = "";  // Буфер для приема команд по UART


uint32_t eepromReadUint32(int addr) {
  uint32_t v = 0;
  v = (uint32_t)EEPROM.read(addr);
  v |= (uint32_t)EEPROM.read(addr + 1) << 8;
  v |= (uint32_t)EEPROM.read(addr + 2) << 16;
  v |= (uint32_t)EEPROM.read(addr + 3) << 24;
  return v;
}

void eepromWriteUint32(int addr, uint32_t v) {
  EEPROM.update(addr, (uint8_t)(v & 0xFF));
  EEPROM.update(addr + 1, (uint8_t)((v >> 8) & 0xFF));
  EEPROM.update(addr + 2, (uint8_t)((v >> 16) & 0xFF));
  EEPROM.update(addr + 3, (uint8_t)((v >> 24) & 0xFF));
}


void setup() {
  Serial.begin(9600);
  uartRxBuffer.reserve(64);

  uint32_t saved = eepromReadUint32(EEPROM_ADDR_COEF);
  if (saved == 0xFFFFFFFFUL || saved == 0UL) {
    speedCoef = DEFAULT_SPEED_COEF;
    eepromWriteUint32(EEPROM_ADDR_COEF, speedCoef);
  } else {
    speedCoef = saved;
  }
  diffMin = speedCoef / SPEED_HARD_LIMIT;

  saved_OCR1A = (uint16_t)eepromReadUint32(EEPROM_ADDR_OCR1A);
  if (saved_OCR1A == 0xFFFFFFFFUL || saved_OCR1A == 0UL) {
    saved_OCR1A = DEFAULT_OCR1A;
    eepromWriteUint32(EEPROM_ADDR_OCR1A, saved_OCR1A);
  }

  uint8_t savedWin = EEPROM.read(EEPROM_ADDR_WIN);
  if (savedWin < 3 || savedWin > PERIOD_BUF_MAX || (savedWin % 2) == 0) {
    periodWinSize = 11;  // дефолт
    EEPROM.update(EEPROM_ADDR_WIN, periodWinSize);
  } else {
    periodWinSize = savedWin;
  }

  lowerSpeedCutoff = EEPROM.read(EEPROM_ADDR_SL);
  if (lowerSpeedCutoff == 0xFF || lowerSpeedCutoff == 0) {
    lowerSpeedCutoff = 20;  // дефолт
    EEPROM.update(EEPROM_ADDR_SL, lowerSpeedCutoff);
  }



  Serial.println(F("CAN Speed sensor — старт"));
  Serial.print(F("Текущий коэффициент (speedCoef): "));
  Serial.println(speedCoef);
  Serial.print(F("Текущая отсечка скорости: "));
  Serial.println(lowerSpeedCutoff);
  Serial.print(F("Текущее значение OCR1A: "));
  Serial.println(saved_OCR1A);
  Serial.print(F("Размер окна медианного фильтра: "));
  Serial.println(periodWinSize);
  Serial.println(F("Отправь service для входа в сервисный режим"));

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= (1 << ICES1);
  TCCR1B |= (1 << CS12) | (1 << CS10);
  TCNT1 = 0;
  OCR1A = TCNT1 + saved_OCR1A;
  TIMSK1 |= (1 << ICIE1);
  TIMSK1 |= (1 << OCIE1A);
  sei();
}


ISR(TIMER1_CAPT_vect) {
  uint16_t capture = ICR1;

  if (capture >= lastCaptureTicks) {
    diff = capture - lastCaptureTicks;
  } else {
    diff = (0xFFFF - lastCaptureTicks) + capture + 1;
  }
  lastCaptureTicks = capture;

  speedSignalAlive = true;
  filterMedian = true;
  lastSpeedTick = compaTick;
}

ISR(TIMER1_COMPA_vect) {
  OCR1A += saved_OCR1A;
  compaTick++;
  if ((uint16_t)(compaTick - lastSpeedTick) > SPEED_TIMEOUT_TICKS) {
    speedSignalAlive = false;
    diffMinCnt = 0;
    canSpeedKmh = 0;
  }

  canTxPending = true;
}

void loop() {

  if (filterMedian) {
    uint16_t diffCan;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      filterMedian = false;
      diffCan = diff;
    }

    if (diffCan < 50) return;

    if (diffCan < diffMin) {
      if (diffMinCnt < 10) {
        diffMinCnt++;
        return;
      }
      canSpeedKmh = 150;
      return;
    }
    diffMinCnt = 0;

    periodBuf[periodIdx] = diffCan;
    periodIdx++;

    if (periodIdx >= periodWinSize) {
      periodIdx = 0;
      periodBufFilled = true;
    }

    if (!periodBufFilled) return;

    uint16_t tmp[PERIOD_BUF_MAX];
    for (uint8_t i = 0; i < periodWinSize; i++) {
      tmp[i] = periodBuf[i];
    }

    for (uint8_t i = 1; i < periodWinSize; i++) {
      uint16_t key = tmp[i];
      int8_t j = i - 1;
      while (j >= 0 && tmp[j] > key) {
        tmp[j + 1] = tmp[j];
        j--;
      }
      tmp[j + 1] = key;
    }

    uint16_t medianDiff;
    medianDiff = tmp[periodWinSize - 3];  // Смещение точки мдианы
    if (medianDiff == 0) return;

    speedRaw = speedCoef / medianDiff;
    canSpeedKmh = (!speedSignalAlive || speedRaw <= lowerSpeedCutoff) ? 0 : speedRaw;
  }



  if (canTxPending) {
    uint32_t speedTx;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      speedTx = canSpeedKmh;
      canTxPending = false;
    }

    //Serial.print(F("[SP] Speed = "));
    //Serial.println(speedTx);
    //Serial.println(F(" km/h"));


    uint16_t Data1 = (uint16_t)((speedTx * 100UL) >> 8);
    uint16_t Data2 = (uint16_t)(((uint32_t)speedTx * 10000UL >> 8) % 100UL);

    canMsg.can_id = 0x355;            // устанавливаем CAN id в 0x355
    canMsg.can_dlc = 0x07;            // длина поля данных CAN = 7
    canMsg.data[0] = (uint8_t)Data1;  // первое значение скорости
    canMsg.data[1] = (uint8_t)Data2;  // второе значение скорости
    canMsg.data[2] = 0x00;
    canMsg.data[3] = 0x00;
    canMsg.data[4] = 0x00;
    canMsg.data[5] = 0x00;
    canMsg.data[6] = 0x40;  //команда 0x40 на включение ЭУР
    canMsg.data[7] = 0x00;
    mcp2515.sendMessage(&canMsg);
  }


  // 1) UART input (line-oriented)
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (uartRxBuffer.length() > 0) {
        handleCommand(uartRxBuffer);
        uartRxBuffer = "";
      }
    } else {
      uartRxBuffer += c;
      if (uartRxBuffer.length() > 200) uartRxBuffer = uartRxBuffer.substring(uartRxBuffer.length() - 200);
    }
  }

  unsigned long now = millis();

  // 2) SP режим
  if (serviceMode && showSpeedInService && (now - lastServicePrintMs >= SP_INTERVAL_MS)) {
    lastServicePrintMs = now;
    uint32_t speedTx;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      speedTx = canSpeedKmh;
    }

    Serial.print(F("[SP] Speed = "));
    Serial.print(speedTx);
    Serial.println(F(" km/h"));
  }

  // 3) Калибровка
  if (calibrating) {
    if (now - calib_last_ms >= CALIB_INTERVAL_MS) {
      calib_last_ms = now;
      uint32_t speedSample;

      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        speedSample = canSpeedKmh;
      }

      calibSpeedSum += speedSample;
      calib_samples_collected++;

      Serial.print(F("[CALIB] sample "));
      Serial.print(calib_samples_collected);
      Serial.print(F(" = "));
      Serial.print(speedSample);
      Serial.println(F(" km/h"));

      if (calib_samples_collected >= CALIB_SAMPLE_COUNT) {
        uint32_t speedAvg = calibSpeedSum / (uint32_t)CALIB_SAMPLE_COUNT;
        if (speedAvg == 0) {
          Serial.println(F("[CALIB] Ошибка: измеренная скорость = 0, калибровка не выполнена"));
        } else {
          uint64_t coefCalc = (uint64_t)speedCoef * (uint64_t)calibTargetSpeed;
          uint32_t newCoef = (uint32_t)(coefCalc / (uint64_t)speedAvg);
          if (newCoef == 0) {
            Serial.println(F("[CALIB] Вычислен нулевой коэффициент — отказ"));
          } else {
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
              speedCoef = newCoef;
            }
            eepromWriteUint32(EEPROM_ADDR_COEF, speedCoef);
            Serial.print(F("[CALIB] Успешно. Новый speedCoef = "));
            Serial.println(speedCoef);
            diffMin = speedCoef / SPEED_HARD_LIMIT;
          }
        }
        calibrating = false;
        calib_samples_collected = 0;
        calibSpeedSum = 0;
        printServiceHints();
      }
    }
  }
}

void printlnSlow(const __FlashStringHelper* s) {
  Serial.println(s);
  delay(90);
}

void printServiceHints() {
  printlnSlow(F("=== СЕРВИСНЫЙ РЕЖИМ ==="));
  printlnSlow(F("stop - Выйти из сервисного режима."));
  printlnSlow(F("sp - Показать/скрыть текущую скорость."));
  printlnSlow(F("start <speed> - Начать калибровку. Пример: start 50"));
  printlnSlow(F("sl <speed> - Значение нижней отсечки скорости."));
  printlnSlow(F("win <odd> - Размер медианного фильтра (с перезагрузкой)."));
  printlnSlow(F("ocr <value> - Значение OCR1A (с перезагрузкой)."));
  printlnSlow(F(""));

  printlnSlow(F("-- ↓ текущие значения ↓ --"));
  Serial.println(String(F("Коэффициент = ")) + speedCoef);
  delay(90);
  Serial.println(String(F("Отсечка [sl] = ")) + lowerSpeedCutoff);
  delay(90);
  Serial.println(String(F("Размер фильтра [win] = ")) + periodWinSize);
  delay(90);
  Serial.println(String(F("Размер OCR1A [ocr] = ")) + saved_OCR1A);
  delay(90);
  printlnSlow(F("========================="));
}

void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  String c = cmd;
  c.toLowerCase();

  if (c == "service") {
    if (!serviceMode) {
      serviceMode = true;
      showSpeedInService = false;
      calibrating = false;
      canTxPending = false;
      Serial.println(F("Вход в сервисный режим"));
      delay(90);
      printServiceHints();
    } else {
      Serial.println(F("Уже в сервисном режиме"));
    }
    return;
  }

  if (!serviceMode) {
    Serial.println(F("Не в сервисном режиме. Отправь 'service' чтобы войти."));
    return;
  }

  if (c.startsWith("win")) {
    int spaceIdx = cmd.indexOf(' ');
    if (spaceIdx < 0) {
      Serial.println(F("Формат: win <odd_value>"));
      return;
    }

    int v = cmd.substring(spaceIdx + 1).toInt();

    if (v < 3 || v > PERIOD_BUF_MAX || (v % 2) == 0) {
      Serial.print(F("Ошибка: размер окна должен быть нечётным (3.."));
      Serial.print(PERIOD_BUF_MAX);
      Serial.println(F(")"));
      return;
    }

    EEPROM.update(EEPROM_ADDR_WIN, (uint8_t)v);

    Serial.print(F("Размер окна сохранён = "));
    Serial.println(v);
    Serial.println(F("Перезагрузите устройство для применения"));
    return;
  }


  if (c == "stop") {
    serviceMode = false;
    showSpeedInService = false;
    calibrating = false;
    Serial.println(F("Выход из сервисного режима. Возобновлена нормальная работа."));
    return;
  }

  if (c == "sp") {
    showSpeedInService = !showSpeedInService;
    Serial.print(F("Показывать скорость: "));
    Serial.println(showSpeedInService ? F("ВКЛ") : F("ВЫКЛ"));
    return;
  }

  // sl <value>  — изменить нижнюю отсечку скорости
  if (c.startsWith("sl ")) {
    int spaceIdx = cmd.indexOf(' ');
    if (spaceIdx < 0) {
      Serial.println(F("Формат: sl <value>"));
      return;
    }

    String num = cmd.substring(spaceIdx + 1);
    num.trim();
    long v = num.toInt();

    if (v < 2 || v > 250) {
      Serial.println(F("Ошибка: sl вне диапазона (2..250)"));
      return;
    }

    lowerSpeedCutoff = (uint8_t)v;
    EEPROM.update(EEPROM_ADDR_SL, lowerSpeedCutoff);


    Serial.print(F("SL сохранён в EEPROM = "));
    Serial.println(lowerSpeedCutoff);

    return;
  }

  // ocr <value>  — записать OCR1A в EEPROM (применится после перезагрузки)
  if (c.startsWith("ocr ")) {
    int spaceIdx = cmd.indexOf(' ');
    if (spaceIdx < 0) {
      Serial.println(F("Формат: ocr <value>"));
      return;
    }

    String num = cmd.substring(spaceIdx + 1);
    num.trim();
    long val = num.toInt();

    if (val < 50 || val > 60000) {
      Serial.println(F("Ошибка: OCR1A вне диапазона (50..60000)"));
      return;
    }

    eepromWriteUint32(EEPROM_ADDR_OCR1A, (uint32_t)val);

    Serial.print(F("OCR1A сохранён в EEPROM = "));
    Serial.println(val);
    Serial.println(F("Перезагрузите устройство для применения"));

    return;
  }

  // start <speed>
  if (c.startsWith("start")) {
    int spd = -1;
    int spaceIdx = cmd.indexOf(' ');
    if (spaceIdx > 0) {
      String num = cmd.substring(spaceIdx + 1);
      num.trim();
      if (num.length() > 0) {
        spd = num.toInt();
      }
    }
    if (spd <= 0) {
      Serial.println(F("Не верный ввод -> (пример ввода: start 50)"));
      return;
    }

    if (calibrating) {
      Serial.println(F("Калибровка уже идёт."));
      return;
    }
    calibTargetSpeed = (uint32_t)spd;
    calibrating = true;
    calib_samples_collected = 0;
    calibSpeedSum = 0;
    calib_last_ms = millis();
    Serial.print(F("Калибровка начата для скорости "));
    Serial.print(calibTargetSpeed);
    Serial.println(F(" km/h. Держать скорость 5 сек."));
    return;
  }

  Serial.print(F("Неизвестная команда: "));
  Serial.println(cmd);
  delay(90);
  printServiceHints();
}
