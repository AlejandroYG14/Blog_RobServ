# Práctica 3 - Autoparking

El objetivo de esta práctica es diseñar e implementar el sistema de navegación de un vehículo autónomo capaz de localizar una plaza de aparcamiento y estacionar de forma segura. Para lograrlo, el sistema debe procesar la información del entorno mediante un sensor LiDAR 3D, interpretar los datos para detectar huecos válidos y ejecutar las maniobras necesarias sin colisionar con los vehículos adyacentes.

He utilizado la documentación de usuario de Unibotics para esta práctica: https://jderobot.github.io/RoboticsAcademy/exercises/AutonomousCars/autoparking/

## Descripción del funcionamiento

La solución desarrollada se basa en una arquitectura modular que separa el procesamiento sensorial de la toma de decisiones.

### Procesamiento del LiDAR y geometría de la calle

Para entender el entorno, utilizamos los datos de la función `HAL.getLidarData()`. Como el LiDAR devuelve mucha información que no necesitamos, en la función `fetch_local_obstacles()` hacemos un filtrado espacial en dos pasos. Primero, descartamos todo lo que caiga fuera de nuestra franja de interés vertical (usando `LIDAR_Z_MIN` y `LIDAR_Z_MAX`) para ignorar el suelo y objetos altos. Después, creamos una "caja ciega" alrededor del sensor con las medidas físicas del vehículo (`CAR_FRONT_Y_LIDAR`, `CAR_REAR_Y_LIDAR`, `CAR_EDGE_X_LIDAR`) para que el coche no vea su propio chasis y crea que está rodeado de obstáculos.

El reto aquí era que, si el coche se tuerce un poco, las lecturas laterales del sensor también se tuercen. Para solucionar esto, necesitábamos un sistema de coordenadas global. Lo hacemos al principio de la ejecución con `estimate_street_geometry()`. Esta función toma los puntos del lateral derecho, hace un barrido angular y agrupa los puntos en tramos de 25 cm. Mediante un sistema de "votación", encontramos el ángulo exacto en el que los coches aparcados forman la línea recta más larga. Guardamos este ángulo como nuestra orientación base (`street_heading` dentro de `street_ref`).

Con esta referencia clara, cualquier lectura nueva pasa por `transform_cloud_to_street()`. Usando trigonometría (`math.cos`, `math.sin`), rotamos la nube de puntos del sensor para proyectarla siempre sobre los ejes de la calle. Por último, usamos `WebGUI.showLidar()` para dibujar estos puntos ya procesados en la interfaz, lo que ha resultado muy útil para depurar lo que ve el coche realmente.

### Control de navegación

Durante la fase de búsqueda de plaza, se descartó un control de conducción puramente reactivo. Si el giro del volante dependiera del obstáculo más cercano en cada instante, el coche reaccionaría a cada hueco entre vehículos o esquivaría los retrovisores pegando volantazos.

Para que el movimiento sea perfecto, el coche navega anclado a un "raíl virtual". La función `command_lane_tracking()` se encarga de esto: calcula continuamente nuestro error lateral (`lat_error`), que es la diferencia entre dónde estamos (`d_left`) y la cota ideal por la que deberíamos ir (`target_lane_d`, calculada para mantener la separación `SEARCH_TARGET_DIST`).

Este error se compensa usando una ganancia `LATERAL_GAIN`, pero para evitar movimientos bruscos, saturamos la corrección máxima permitida con `MAX_STEER_CORRECTION_RAD`. El comando final se manda a los motores a través de `HAL.setW()` ajustado por `YAW_GAIN`. El resultado es un avance suave y paralelo a la fila de coches.

Al llevar el coche estabilizado sobre este raíl virtual, medir el hueco es mucho más sencillo. La función `evaluate_parking_window()` solo tiene que revisar la nube de puntos proyectada y comprobar si hay algún obstáculo dentro de nuestra zona de interés (ayudándose de `is_in_parking_strip()`). Cuando vemos una zona completamente libre con la longitud necesaria (`MIN_SPOT_LENGTH`), localizamos la posición del coche de atrás (`rear_ref`) y marcamos la coordenada exacta (`s_spot_start`) en la que el coche debe detenerse para empezar a maniobrar.

### Máquina de estados

La ejecución está gobernada por la variable `current_phase`, que define la máquina de estados. Las transiciones dependen de la odometría absoluta (`get_state_in_street_frame()`) y los márgenes de los parachoques (`measure_bumper_clearances()`):

- `CALIBRATING_STREET` y `ALIGNING_CAR`: Fija el sistema de coordenadas (`street_ref`) y estabiliza el coche en el carril.

- `SCANNING_SPOT` y `ADVANCING_TO_START`: Localiza el hueco libre y avanza la distancia de preparación preestablecida en la constante `PRE_TURN_ADVANCE`.

- `REVERSE_RIGHT` y `REVERSE_LEFT`: Secuencia de marcha atrás a velocidad `PARK_SPEED`, inyectando el giro `MAX_STEER_ANGLE`. El contravolante se activa al superar el ángulo crítico `S_TURN_CUTOFF_ANGLE`.

- `CHECK_DEPTH` y ajuste en S: Una vez enderezado, se comprueba si el error lateral supera `TARGET_PARK_DEPTH`. De ser así, se activan los estados `REVERSE_STRAIGHT`, `S_ADJUST_RIGHT` y `S_ADJUST_LEFT` (limitados por `MAX_S_TRIES`) para ganar profundidad en la plaza.

- `CENTERING` y `COMPLETED`: El vehículo avanza o retrocede hasta que la diferencia entre la distancia frontal y trasera (`front_margin` y `rear_margin`) es inferior a 0.20 metros, deteniendo finalmente los motores.

## Problemas y soluciones

Durante el desarrollo de la práctica, aparecieron diversas complicaciones derivadas de la cinemática del coche y de cómo este interpreta la información del sensor LiDAR. A continuación, se detallan los principales problemas y las soluciones implementadas:

### El coche buscaba aparcamiento a la izquierda
En las primeras fases de pruebas, el vehículo tendía a ignorar los huecos del carril derecho y se dirigía hacia el lado opuesto de la carretera, intentando alinearse y aparcar a la izquierda. Este comportamiento se debía a una interpretación ambigua del signo del error lateral y de la orientación de los ejes del LiDAR, lo que provocaba que el regulador proporcional invirtiera la corrección del volante.

**Solución**: Se unificó el criterio geométrico para todo el algoritmo. En el módulo de estimación inicial (`estimate_street_geometry()`), se forzó al sistema a buscar y puntuar únicamente las líneas de obstáculos situadas en coordenadas laterales positivas (`right_proj > 0.0`). Posteriormente, en la función `command_lane_tracking()`, se ajustaron los signos del cálculo del `lat_error` y del comando de giro resultante (`desired_yaw`) para garantizar que el volante siempre aplicara la corrección hacia el carril derecho.

### Desorientación del sensor al girar (problema con la perspectiva local)
Inicialmente, la distancia a los obstáculos se medía usando de forma directa las coordenadas X e Y relativas al coche. El problema era que en cuanto el vehículo modificaba un poco su rumbo (`yaw_error`), su "derecha" y su "frente" cambiaban por completo. Esto provocaba variaciones bruscas en las distancias leídas, causando que el algoritmo descartara huecos grandes o abortara el aparcamiento por un falso peligro.

**Solución**: Se implementó la proyección geométrica explicada anteriormente (`transform_cloud_to_street()`). Al independizar los cálculos de la orientación momentánea del chasis y proyectarlos sobre los ejes de la calle, se crea un "rectángulo virtual" que no gira. El sistema dejó de evaluar el entorno de forma puramente local para entenderlo de manera global.

### Distancia de aparcamiento irregular
Una vez lograda la inserción en la plaza, el resultado solía variar según el punto exacto donde se hubiera iniciado la maniobra. Tras realizar el contravolante, el vehículo quedaba muchas veces perfectamente paralelo al resto, pero demasiado alejado del bordillo e invadiendo parte del carril.

**Solución**: Se añadió el estado `CHECK_DEPTH` a la máquina de decisiones. En lugar de dar por finalizado el aparcamiento al quedarse recto, el algoritmo evalúa su posición lateral. Si el vehículo está muy salido, activa una secuencia de ajuste dinámico (`REVERSE_STRAIGHT`, `S_ADJUST_RIGHT`, `S_ADJUST_LEFT`). Esto permite al coche realizar pequeñas maniobras hacia adelante en forma de "S" para meterse más en la plaza, garantizando la seguridad mediante el cálculo matemático de sus esquinas con `get_front_right_corner_d()`.

###  Falsos positivos de colisión en los parachoques
Durante la maniobra de marcha atrás o ajuste, el coche a menudo se detenía de golpe o cambiaba de estado antes de tiempo al creer que iba a chocar. Esto sucedía porque se le pedía al LiDAR el obstáculo más cercano, que habitualmente resultaba ser el lateral del coche aparcado justo al lado, y no un obstáculo real situado frente al morro o tras el maletero.

**Solución**: Se desarrolló la función `measure_bumper_clearances()`. En lugar de buscar la distancia mínima absoluta a su alrededor, el sistema "recorta" un corredor imaginario que coincide con el ancho físico del vehículo (`HALF_CAR_WIDTH + 0.10`). De este modo, se ignoran los objetos a los lados y solo se frena cuando un obstáculo invade físicamente la trayectoria directa, respetando en todo momento el margen de parada de emergencia (`BUMPER_SAFETY_DIST`).

## Funcionamiento


