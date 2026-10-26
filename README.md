# ice-audio-lab

Процедурная real-time симуляция ДВС + синтез звука. **Никаких сэмплов, никаких ИИ-моделей** — весь звук порождается физической симуляцией (0D термодинамика цилиндров + 1D газодинамика выхлопа) и DSP-цепочкой.

Стадия: **S0 skeleton** (см. `../research/ROADMAP.md`) — рядные 4T двигатели, аудио-тракт, UI с телеметрией.

## Сборка (Windows, VS 2026)

```
scripts\build.bat
```

Или вручную любым CMake ≥ 3.21 + MSVC:
```
cmake -S . -B build -G "Visual Studio 18 2026" -A x64
cmake --build build --config Release
```

Зависимости (imgui, implot, nlohmann/json, miniaudio) подтягиваются через CMake FetchContent при конфигурации — нужен интернет один раз.

## Запуск

```
run.bat
```

Запускает `build\Release\ice-audio-lab.exe` с `assets\engines\baseline_i4.json`. Можно передать свой конфиг:
```
build\Release\ice-audio-lab.exe assets\engines\my_engine.json
```

## Управление

| Клавиша | Действие |
|---|---|
| **W** (зажать) | 100% throttle пока удерживается |
| Слайдер внизу | плавный throttle 0–100% |
| **S** | стартер (тумблер) |
| **A** | зажигание (тумблер) |

## UI

* RPM / момент (мгновенный и средний) / мощность / MAP
* Развёртка давления и объёма в цилиндре по углу 0–720°
* Живая waveform выходного сигнала
* FPS, frame time, загрузка симуляции (% бюджета кадра)
* Ручки громкости и jitter

## Структура

```
core/     физика: gas_system (0D), piston_engine (4T), combustion (flame event),
          exhaust1d (Lax-Wendroff 1D + радиация открытого конца), config (JSON)
audio/    synth: SPSC ring (sim->audio), jitter/DC/derivative/AGC, 48 kHz
app/      Win32+OpenGL3+Dear ImGui оболочка, ImPlot графики
assets/engines/  JSON-конфиги (baseline_i4 + примеры S2-совместимые)
```

## Формат двигателя (S0-подмножество v1)

См. `assets/engines/baseline_i4.json`: crankshafts[0].journals (углы шеек), banks[].cylinders (bore/stroke, rod, head: chamber_cc, cam profile, intake/exhaust port flow_k), ignition (firing_order, timing_curve, rev_limit), induction (plenum+throttle), exhaust.nodes (collector/tailpipe), audio (sim_freq_hz auto, jitter, air_noise, hf_gain).

`flow_k` принимает `"k_carb(220)"` — калибровка в SCFM (как в engine-sim).

## Лицензия

MIT. Вдохновлено engine-sim (Ange Yaghi, MIT) — идеи 0D-газа, flame event и k-калибровки; код написан с нуля.
