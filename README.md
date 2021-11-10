# Adjustable Power Supply

Linear power supply. Output voltage is selected through a keypad interface.

## How to build

`xc8-cc @build.xc8 psu_main.c instruments.c -mcpu=16F1933 -o build/adj_psu.hex`

Note: you need to copy the [FFPIC](https://github.com/kcjulio/ffpic) framework into the src/ directory in order to build this project.
