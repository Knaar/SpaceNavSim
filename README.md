# SpaceNavSim

Базовый C++ проект на Unreal Engine 5.7 для дальнейшей разработки SpaceNavSim. Сейчас в проекте нет собственных ассетов в `Content`; в качестве игровой карты указана стандартная карта движка `/Engine/Maps/Templates/OpenWorld`.

## Структура

- `SpaceNavSim.uproject` — файл проекта.
- `Source/` — игровые и редакторские цели C++.
- `Config/` — конфигурация проекта.
- `Content/` — будущие игровые ассеты.
- `Scripts/Bats/` — батники сборки Win64.
- `Logs/` — конспекты изменений; порядок работы описан в `AGENTS.md`.

## Сборка

Батники рассчитаны на Windows и Unreal Engine 5.7. По умолчанию они используют `E:\UE57`; другой путь к установленному движку можно задать переменной окружения `UE_ROOT`.

| Батник | Результат |
| --- | --- |
| `BuildEditorSpaceNavSim.bat` | Инкрементальная сборка `SpaceNavSimEditor` для Win64 Development. |
| `Scripts/Bats/Development/BuildClient.bat` | Упаковка одиночной игры `SpaceNavSim` для Win64 Development. |
| `Scripts/Bats/Shipping/BuildClient.bat` | Упаковка одиночной игры `SpaceNavSim` для Win64 Shipping. |

Два батника упаковки по умолчанию сохраняют результат в `E:\UE_Builded\SpaceNavSim\Development\Client` и `E:\UE_Builded\SpaceNavSim\Shipping\Client` соответственно. Путь архива можно передать первым аргументом. Они используют игровую цель `SpaceNavSim`; отдельная клиентская или серверная цель проекту не требуется.

## Git

В репозиторий входят исходники, конфигурация, скрипты, `Content` при появлении ассетов и проектные конспекты в `Logs`. Генерируемые папки Unreal, настройки IDE и файл решения исключены через `.gitignore`. Файлы ассетов Unreal помечены как бинарные в `.gitattributes`; Git LFS пока не настроен.
