# SmoothDriving-ESP32
Codigo implementado no chipset de coleta de dados do veiculo 


# Configuração de ambiente de desenvolvimento utilizando Arduino IDE

- Adicionar o link no gerenciador de placas:
    https://dl.espressif.com/dl/package_esp32_index.json

- Dentre as placas do pacote ESP32, selecionar a DOIT_ESP32_DEVKIT_V1 
- No arquivo "Boards,txt" adicionar as configurações abaixo dentro da sessão da placa DOIT_ESP32_DEVKIT_V1
    
        # additional settings by GuiiXav

        esp32doit-devkit-v1.menu.PartitionScheme.default=Default
        esp32doit-devkit-v1.menu.PartitionScheme.default.build.partitions=default
        esp32doit-devkit-v1.menu.PartitionScheme.minimal=Minimal (2MB FLASH)
        esp32doit-devkit-v1.menu.PartitionScheme.minimal.build.partitions=minimal
        esp32doit-devkit-v1.menu.PartitionScheme.no_ota=No OTA (Large APP)
        esp32doit-devkit-v1.menu.PartitionScheme.no_ota.build.partitions=no_ota
        esp32doit-devkit-v1.menu.PartitionScheme.no_ota.upload.maximum_size=2097152
        esp32doit-devkit-v1.menu.PartitionScheme.min_spiffs=Minimal SPIFFS (Large APPS with OTA)
        esp32doit-devkit-v1.menu.PartitionScheme.min_spiffs.build.partitions=min_spiffs
        esp32doit-devkit-v1.menu.PartitionScheme.min_spiffs.upload.maximum_size=1966080
        esp32doit-devkit-v1.menu.PartitionScheme.fatflash=16M Fat
        esp32doit-devkit-v1.menu.PartitionScheme.fatflash.build.partitions=ffat


- Utilizar o "Partion Scheme" configurado no passo anterior "No OTA (Large APP)"
