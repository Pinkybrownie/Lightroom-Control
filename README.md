# Lightroom-Control
Proyecto de programación de microcontroladores sobre el STM32F4 Discovery de ARM.
El sistema cuenta con un dormitorio, un baño, un pasillo y una terraza. 
En el dormitorio se apaga y se enciende la luz mediante el presionado de un interruptor. 
En el baño se apaga y se enciende la luz con un interruptor. Si la luz lleva encendida 100 segundos, se apagará automáticamente.
En el pasillo se enciende la luz al activarse un sensor de proximidad y ésta se mantiene encendida por 10 segundos antes de apagarse automáticamente. 
En la terraza el control automático del alumbrado viene mediante un sensor LDR, que cuando no detecta luz (de noche) activa la luz y cuando sí la detecta (de día) la apaga.
Requisitos
•	Utilizar entradas (pulsadores, botones, etc.) y salidas.
•	Utilizar interrupciones
•	Utilizar temporizadores
•	Utilizar convertidores DA o AD.
