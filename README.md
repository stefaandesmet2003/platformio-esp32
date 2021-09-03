my esp32 projects on platformio

## trainer-bledongle
- antique, kept for reference
- was once used as interface between Elite Direto smarttrainer and an old version of my hometrainer app, before i switched it to web bluetooth. this dongle is not needed anymore
- required modification to BLE library, not sure if this is somehow merged
- made a note here https://github.com/nkolban/esp32-snippets/issues/423 : 

The BLERemoteDescriptor::readValue() function doesn't work, i.e. it never returns, because it waits forever on the semaphore m_semaphoreReadDescrEvt.
I use this function for reading the descriptor 2902 (checking if notifications/indications are on on a BLE server characteristic).

I solved the issue by adding a gattClientEventHandler in the BLERemoteDescriptor class that handles the ESP_GATTC_READ_DESCR_EVT event. The BLERemoteCharacteristic class also needs a small change in its gattClientEventHandler : passing on the received event to the underlying BLERemoteDescriptor's from the m_descriptorMap
The code is in attached zipfile. I hope you find it useful to add to your library.

## others
todo