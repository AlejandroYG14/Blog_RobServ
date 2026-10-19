# Práctica 2 - Rescue People

El objetivo  de esta práctica consiste en implementar la lógica de navegación y visión artificial de un dron para localizar a un grupo de 6 personas perdidas en un entorno marítimo.

Partiendo desde una lancha ubicada en el origen de coordenadas, el sistema debe dirigir el dron hasta la zona de búsqueda aproximada, realizar un barrido del área, procesar las imágenes de la cámara ventral para detectar a los supervivientes y estimar sus coordenadas exactas en el mapa. Además, el sistema debe gestionar de manera autónoma la batería del dron, interrumpiendo la misión para regresar a la lancha a recargarla cuando sea necesario, para posteriormente retomar la búsqueda en el punto donde se detuvo.

He utilizado la documentación de usuario de Unibotics para esta práctica: https://jderobot.github.io/RoboticsAcademy/exercises/Drones/rescue_people

## Descripción del funcionamiento

El código ha sido estructurado en torno a una máquina de estados finita (FSM) reactiva, que determina los comandos de vuelo correspondientes. El funcionamiento se divide en los siguientes bloques:

### Transformación de coordenadas

El primer paso de la misión consiste en traducir las coordenadas GPS proporcionadas en el enunciado (**40º16’47.23” N**, **3º49’01.78” W**) a un sistema de referencia que el dron pueda interpretar. Para ello, se realizó una conversión de estas coordenadas de latitud y longitud a formato UTM (Universal Transverse Mercator), obteniendo así valores en metros. Al establecer la posición inicial de la lancha como el origen en el simulador (0, 0), se calculó la diferencia métrica en ambos ejes respecto a las coordenadas UTM de la zona de los supervivientes. Este proceso arrojó las coordenadas locales de destino (**X=40.3**, **Y=-30.3**) hacia las que el dron debía dirigirse.

### Navegación y patrón de búsqueda

El desplazamiento hacia los objetivos se realiza mediante la función `calculate_navigation`, que implementa un control proporcional. El dron calcula el error angular respecto a su destino y orienta su morro hacia él mientras avanza. Para garantizar que el dron no pierda cota durante los desplazamientos largos, se emplea `HAL.set_cmd_mix` fijando la altitud en 5.0 metros.
Una vez en la zona objetivo (cuyo centro ha sido ajustado a `CENTER_X` y `CENTER_Y` para optimizar el área), el dron inicia un patrón en espiral cuadrada generado por `get_square_spiral_wp`. La separación entre las líneas de la espiral (`STEP_SIZE`) se calcula matemáticamente utilizando el campo de visión de la cámara (FOV) y la altitud, aplicando un solape del 20% para asegurar que no queden zonas ciegas.

### Visión artificial y proyección
Para el reconocimiento de las víctimas se utiliza el clasificador Haar Cascade de OpenCV (`haarcascade_frontalface_default.xml`). Dado que este modelo solo detecta rostros en posición vertical, se diseñó la función `scan_faces_multidirectional`, la cual rota el fotograma en incrementos de 40 grados.
Cuando se detecta un rostro en los píxeles de la imagen, la función `map_pixel_to_world` utiliza trigonometría pura para proyectar esa coordenada 2D al plano 3D del mundo, basándose en la altura, la orientación del dron (`yaw`) y el FOV.

### Gestión de datos y batería

Para evitar registrar a la misma persona varias veces, el sistema almacena los avistamientos en una lista. Si una nueva detección ocurre a menos de 2.5 metros (`MERGE_DISTANCE`) de una anterior, se asume que es la misma persona y se recalcula su posición mediante una media ponderada. Solo cuando una víctima alcanza las 3 confirmaciones visuales (`CONFIRM_THRESHOLD`), se da por válida.
Paralelamente, una variable simula la descarga de la batería. Al bajar del 35%, el dron guarda el índice actual de la espiral (`spiral_idx`), regresa al barco, aterriza y recarga, para luego despegar y continuar la misión de forma transparente. Al confirmar a las 6 personas, la misión concluye y el dron regresa definitivamente a la base.

### Máquina de estados
La lógica del dron se rige por los siguientes estados:
*   `TAKING_OFF`: Inicia el despegue y verifica que se alcance la altitud antes de permitir desplazamientos. Si es el inicio de la misión transiciona a `NAVIGATING_ZONE`, si es una reanudación transiciona a `RETURNING_SEARCH`.
*   `NAVIGATING_ZONE`: Dirige el dron desde la base hasta el centro exacto de la zona de rescate.
*   `SEARCHING`: Ejecuta el barrido en espiral cuadrada waypoint a waypoint. Es el único estado donde se procesa la visión artificial.
*   `RETURNING_BOAT`: Interrumpe la misión actual (por batería baja o misión completada) y dirige el dron de vuelta a las coordenadas `(0, 0)`.
*   `LANDING`: Detiene el movimiento horizontal y desciende. Evalúa si debe recargar o dar la misión por finalizada basándose en el recuento de víctimas.
*   `RECHARGING`: Mantiene motores apagados mientras simula la recarga al 100%. Tras esto, reinicia el ciclo yendo a `TAKING_OFF`.
*   `RETURNING_SEARCH`: Vuelo de regreso desde la base hasta el último punto de la espiral visitado antes de recargar.
*   `MISSION_COMPLETED`: Estado terminal. Las 6 víctimas han sido localizadas y el dron se encuentra en el barco.

## Problemas y soluciones

- **Orientación de los rostros no detectada**: El clasificador Haar Cascade utilizado requiere que los rostros se encuentren en posición vertical. Como el dron emplea una cámara ventral y los supervivientes flotaban en diferentes orientaciones en el agua, inicialmente el algoritmo no detectaba las caras al no estar bien orientadas. La solución consistió en rotar cada fotograma varias veces (en incrementos de 40 grados) y pasar el detector sobre cada rotación para asegurar el reconocimiento sin importar la postura de la persona.

- **Recortes de imagen en la rotación**: Derivado de la solución anterior, al girar las imágenes de la cámara ventral, las esquinas de los fotogramas se perdían fuera de los márgenes, ignorando posibles víctimas. Se solventó creando por software un lienzo cuadrado negro del tamaño de la hipotenusa de la imagen original sobre el cual realizar las rotaciones de forma segura.

- **Detecciones duplicadas de una misma persona**: Al procesar las imágenes a alta frecuencia, el sistema detectaba repetidamente a una misma persona en posiciones o fotogramas ligeramente distintos, lo que alteraba el recuento final. Para solucionarlo, se introdujo una lógica de agrupación por distancias. Si se detecta un rostro en un radio menor a 2.5 metros de una marca previa, se unifican las coordenadas. Exigir 3 confirmaciones por persona también ayudó a filtrar por completo los falsos positivos producidos por elementos del mar.

- **Origen de la espiral descentrado**: Al utilizar la coordenada GPS calculada como centro absoluto de la espiral, el patrón de búsqueda comenzaba en la esquina superior izquierda del grupo de supervivientes. Esto provocaba que la espiral tuviera que expandirse de manera desproporcionada hacia zonas de agua vacía para poder cubrir a las víctimas situadas en el extremo opuesto, agotando la batería prematuramente. Se solucionó restando un desplazamiento fijo a las coordenadas iniciales (`CENTER_X = TARGET_X - 7.0` y `CENTER_Y = TARGET_Y - 5.0`), situando así el inicio geométrico de la espiral en el centro real del grupo de personas.

- **Inercia al aterrizar en la lancha**: Inicialmente, el dron se precipitaba al agua al intentar aterrizar, debido a la velocidad que traía. Esto se corrigió añadiendo un frenado proporcional en distancias menores a 2.5 metros, y exigiendo una tolerancia más estricta (`0.3m`) antes de ejecutar el comando de aterrizaje. Además, como respaldo al sensor de estado del simulador, que no siempre detectaba correctamente el aterrizaje, se añadió una lectura combinada (`if HAL.get_landed_state() == 1 or current_z < 1.5:`). De este modo, si el dron descendía de 1.5m (la cubierta está a 1.4m), se asumía el contacto para iniciar la recarga.

## Funcionamiento
