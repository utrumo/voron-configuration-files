# driver_fan_controller — Klipper extra module

PI контроллер вентилятора по температуре датчиков. Управляет `fan_generic` через reactor timer с плавным изменением скорости.

Создан для driver fan (TMC2240), но работает с любыми датчиками у которых есть `get_status()['temperature']`.

## Зачем

Стандартный `temperature_fan` (PID) не работает с TMC2240 — `temperature_combined` падает с ошибкой "Temperature monitor not supported" потому что TMC2240 возвращает `temperature: None` до включения степперов.

`controller_fan` привязан к активности степперов, а не к температуре — вентилятор молотит на 75% даже если драйверы холодные.

Этот модуль читает температуру напрямую через Python API Klipper, пропускает `None` значения, и управляет вентилятором через PI контроллер с EMA сглаживанием и rate limiting.

## Установка

### 1. Symlink в Klipper extras

```bash
ln -sf ~/printer_data/config/driver_fan_controller.py ~/klipper/klippy/extras/driver_fan_controller.py
```

Symlink переживает `git pull` в klipper (untracked файлы не удаляются).

### 2. Конфигурация в printer.cfg

```ini
[fan_generic driver_fan]         # Пин вентилятора (обязательно)
pin: PD14
cycle_time: 0.00005
max_power: 1.0
shutdown_speed: 0.0

[driver_fan_controller]          # PI контроллер
fan: driver_fan                  # имя fan_generic секции
sensors: tmc2240 stepper_x, tmc2240 stepper_y  # датчики (берётся max)
target_temp: 65.0                # целевая температура (C)
kp: 0.2                         # P-коэффициент
ki: 2.0                         # I-коэффициент (с делителем /100)
ema_alpha: 0.1                  # EMA сглаживание (0.1=агрессивное, 1.0=без)
off_below: 0.15                 # выключить если скорость < 15%
hysteresis: 0.01                # не обновлять если разница < 1%
max_speed_delta: 0.02           # макс изменение за цикл (0=без лимита)
poll_interval: 0.3              # интервал опроса (сек)
```

### 3. Перезапуск

```bash
# После изменения printer.cfg (параметры секции):
FIRMWARE_RESTART

# После изменения .py файла:
sudo systemctl restart klipper
```

FIRMWARE_RESTART не перезагружает Python модули — только конфиг.

## Алгоритм

```
Датчик(и) -> [max] -> [EMA сглаживание] -> [PI контроллер] -> [Rate limiter] -> [Hysteresis] -> Вентилятор
```

1. **Max** — из нескольких датчиков берётся самый горячий
2. **EMA** — `smooth = alpha * raw + (1-alpha) * smooth` — фильтрует шум ADC
3. **PI** — `speed = Kp * error + Ki * integral / 100` — пропорционально-интегральный регулятор
4. **Off-below** — скорость ниже порога = выключен (вентилятор гудит на низких оборотах)
5. **Rate limit** — максимальное изменение скорости за цикл (плавный разгон/торможение)
6. **Hysteresis** — не отправлять команду если изменение слишком мало

## Подбор параметров

**Формула для kp:** `kp = 1.0 / (temp_100pct - target_temp)`

Пример: 100% при 70C, target 65C -> kp = 1.0 / 5 = 0.2

**Время разгона:** `time_0_to_100 = 1.0 / max_speed_delta * poll_interval`

Пример: delta=0.02, poll=0.3s -> 0-100% за 15 секунд

| Симптом | Что крутить |
|---------|-------------|
| Не выходит на 100% при нужной температуре | Уменьшить target_temp или увеличить kp |
| Скорость скачет ступеньками | Уменьшить max_speed_delta |
| Дёргается вкл/выкл | Увеличить off_below или уменьшить kp |
| Медленная реакция на нагрев | Увеличить ema_alpha или kp |
| Температура не стабилизируется | Увеличить ki |

## Проверка работы

```bash
# Статус контроллера
curl -s "http://localhost:7125/printer/objects/query?driver_fan_controller" | python3 -m json.tool

# Лог загрузки
grep "driver_fan_controller" ~/printer_data/logs/klippy.log

# Мониторинг (60с)
for i in $(seq 1 60); do
  curl -s "http://localhost:7125/printer/objects/query?driver_fan_controller" | \
    python3 -c "import sys,json; c=json.load(sys.stdin)['result']['status']['driver_fan_controller']; print('temp=%.1f speed=%.0f%%' % (c['temperature'], c['speed']*100))"
  sleep 1
done
```

## Результаты (Voron 2.4 R2, TMC2240)

Конфиг: poll 0.3s, target 65C, Kp=0.2, Ki=2.0, alpha=0.1, max_delta=0.02

120 замеров за 60 секунд:
- Raw температура: range 0.39C (шум ADC)
- Smooth температура: range 0.27C (сглаживание 1.4x)
- Скорость: range 5.6% (85-91%), стабильна 90% времени
- Max delta между соседними замерами: 1.4%
