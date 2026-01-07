### VS Code IntelliSense setup

VS Code users should treat .ino files as C++ (see settings.json).

Set the following environment variables:

- ARDUINO_LIB  
  → path to your Arduino sketchbook `libraries` folder  
  (e.g. `C:\Users\<you>\Arduino\libraries`)

- ARDUINO15  
  → path to your Arduino15 data directory  
  (e.g. `C:\Users\<you>\AppData\Local\Arduino15`)

- ARDUINO_BIN 
 → Arduino IDE CLI directory
 (e.g. `C:\Program Files\Arduino IDE\resources\app\lib\backend\resources`)

Once set, open the project in VS Code and reload the window.
