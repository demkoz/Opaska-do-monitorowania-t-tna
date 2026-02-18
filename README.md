
# Opaska do monitorowania HR

Repozytorium zawiera kod źródłowy oraz pliki konfiguracyjne prototypu opaski do monitorowania tętna (HR) i saturacji krwi (SpO2) z komunikacją Bluetooth Low Energy, zrealizowane w Zephyr RTOS.


## Płytka drukowana

![Opaska-do-monitorowania-tetna](./Zrzut ekranu 2025-11-25 161712.png)
## Struktura projektu

- `src/` – źródła oprogramowania mikrokontrolera
- `inc/` – nagłówki projektowe
- `boards/nrf/opaska/` – konfiguracja płytki dla Zephyr (piny, urządzenia)
- `prj.conf` – konfiguracja projektu Zephyra
- `.gitignore` – ignorowane pliki/ścieżki
