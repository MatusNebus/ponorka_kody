# Ponorka Arduino kódy

Tento repozitár obsahuje Arduino kódy ktore som pouzil pri vyvoji mojej ponorky na dialkove ovladanie. 

## Priečinky

- `developmentKodu` – hlavný all-in-one vývojový kód: PPM vstupy, stepper, ESC, LED, regulácia, senzor a OSD výstup
- `kalibraciaStepperMotoruEEPROM` – kalibrácia rozsahu steppera a uloženie MIN/MAX do EEPROM cez Serial
- `kodPreStepper` – jednoduchý test steppera s pohybom dopredu a dozadu
- `kodPreTlakomer` – test tlakomera MS5837 a výpočet hĺbky z tlaku
- `kod_pre_teplomer` – test DS18B20 teplomera
- `LedkyCezPPMFunguju` – ovládanie LED cez PPM prepínač na CH6
- `OSD_fungujuci` – funkčný OSD kód, ktorý zobrazuje hĺbku aj teplotu
- `OSDdevelopmentKodu` – vývojová verzia OSD kódu
- `OSDzistovanieFormatu` – zistenie formátu dát, ktorý používa OSD
- `RegulaciaSimulaciaLogovanie` – simulácia regulácie a logovanie výsledkov
- `RiadenieRegulaciaSimulaciaTest` – test riadenia a regulácie v simulácii
- `realneRiadenieRegulaciaLogovanie` – DOLEZITE reálne riadenie s reguláciou, logovaním a odosielaním dát do OSD
- `skuskaOSD_nepodstatne` – starší a nepodstatný test OSD
- `testujemTelemetriu` – testovanie telemetrie cez iBUS
- `vynulovanieEEPROM` – úplné vymazanie EEPROM

## Poznámka

Každý Arduino sketch je v samostatnom priečinku, pretože Arduino IDE vyžaduje, aby sa `.ino` súbor volal rovnako ako priečinok.