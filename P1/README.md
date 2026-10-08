# Práctica 1 - Localized Vacuum Cleaner

El objetivo de esta práctica consiste en implementar un algoritmo robusto de planificación de rutas y navegación para una aspiradora autónoma operando en un entorno simulado. Se busca conseguir la máxima cobertura de la superficie disponible utilizando una aproximación estructurada basada en celdillas (grid), evitando el movimiento puramente aleatorio.

Para alcanzar este objetivo, el desarrollo se ha estructurado en el cumplimiento de tres tareas fundamentales:

1. **Registro del mapa de la casa** para situar al robot en su entorno espacial.
2. **Planificación del movimiento** utilizando una variante del algoritmo BSA (Backtracking Spiral Algorithm).
3. **Ejecución del camino con un controlador reactivo** que va corrigiendo las posibles desviaciones en el movimiento debidas al ruido en los actuadores.

He utilizado la documentación de usuario de Unibotics para este ejercicio: https://jderobot.github.io/RoboticsAcademy/exercises/MobileRobots/vacuum_cleaner_loc

## Funcionamiento del algoritmo

El sistema ejecuta estas tres tareas de manera secuencial y coordinada mediante la siguiente arquitectura:

### 1. Registro del mapa de la casa y generación del gridmap

Para establecer una correspondencia exacta entre las coordenadas métricas de Gazebo (X, Y) y los píxeles de la imagen del mapa de la casa (U, V), implementé un sistema de calibración basado en mínimos cuadrados. Tomando 12 puntos de referencia conocidos, el código calcula dinámicamente dos matrices de transformación afín (directa e inversa) utilizando la función `np.linalg.lstsq` de la librería NumPy, lo que permite traducciones precisas bidireccionales.

<img width="410" height="132" alt="imagen" src="https://github.com/user-attachments/assets/ddbeb4e6-74ef-43a3-9ab6-42713e1d409b" />

- Matriz izquierda (A): Son las coordenadas de Gazebo (`A_gazebo`), donde a cada punto se le añade un 1 en la tercera columna para poder calcular la traslación.
- Matriz central (T): Es la matriz de transformación desconocida de 3x2. Esto es lo que el algoritmo de mínimos cuadrados está intentando descubrir (`T_DIRECT`).
- Matriz derecha (B): Son los puntos en píxeles medidos en el mapa (`pixel_points`).


Posteriormente, el mapa original en formato imagen se transforma en una matriz lógica (gridmap). El espacio se divide en celdas de 30 centímetros. Para procesar la imagen de entrada, me apoyé en la librería OpenCV (`cv2`). Se neutraliza el canal alfa (transparencias) pasándolo a blanco, se convierte a escala de grises con `cv2.cvtColor` y se binariza mediante `cv2.threshold`. Con un umbral de ocupación muy sensible (0.1%), cualquier celda que contenga una mínima parte de un obstáculo se marca como ocupada, generando un espacio de configuración seguro para el diámetro del robot.

### 2. Planificación del movimiento usando el algoritmo BSA

Antes de iniciar el movimiento, el robot calcula toda la ruta necesaria para limpiar la casa mediante el algoritmo BSA. Las características principales de esta fase de planificación son:

* **Comportamiento en espiral:** El algoritmo avanza marcando las celdas como visitadas y priorizando los movimientos en el siguiente orden: Recto > Giro a la derecha > Giro a la izquierda (configurado en la variable `TURN_PRIORITY`). Esto genera patrones de limpieza en espiral.
* **Puntos de retorno:** Mientras explora, el sistema guarda las celdas vecinas libres como posibles puntos de retorno.
* **Resolución de puntos críticos:** Cuando el algoritmo alcanza un callejón sin salida (punto crítico), detiene el barrido secuencial. En este momento, se ejecuta un algoritmo de Búsqueda en Anchura (BFS). Para optimizar el rendimiento de esta búsqueda, utilicé una estructura de cola doble (`deque` del módulo `collections`) sobre el mapa original, encontrando el camino libre más corto hacia el punto de retorno más cercano.
* **Generación de waypoints:** Durante toda la planificación, se almacenan los vértices de la ruta en la lista `waypoints`, que luego se traduce a coordenadas espaciales continuas (`waypoints_gazebo`).

### 3. Ejecución del camino con un controlador reactivo

Una vez calculada la lista de objetivos, el programa entra en un bucle de control continuo. Para el seguimiento de la ruta y corregir de forma continua las posibles desviaciones en el movimiento debidas al ruido en los actuadores del simulador, implementé un controlador proporcional (P).

El sistema calcula en cada iteración el ángulo hacia el objetivo usando `math.atan2` y determina el error angular respecto a la orientación actual (`yaw`) del robot. Este error se normaliza estrictamente entre -pi y +pi para evitar giros redundantes. La velocidad angular de salida se ajusta multiplicando este error por la ganancia `K_P`, y finalmente se satura utilizando la constante `W_MAX` mediante las funciones `max()` y `min()` para evitar comandos inestables. Simultáneamente, el visualizador se actualiza en tiempo real mostrando la posición del robot mediante `cv2.circle` y dibujando el próximo objetivo en el grid.

## Problemas enfrentados y solucionados

Durante el desarrollo de la práctica surgieron diversas dificultades geométricas y de control que requirieron adaptaciones específicas en el código:

* **Desfases en la discretización del mapa:** Al principio, la división matemática de los píxeles generaba un error de truncamiento que desplazaba la cuadrícula a medida que avanzaba por el mapa, provocando que los muros lógicos no coincidieran con los muros físicos. Solucioné esto sustituyendo las conversiones a enteros simples por la función matemática de redondeo (`round()`), y utilizando `math.ceil()` junto a `min()` en los bucles de iteración para asegurar que los bordes inferiores y derechos del mapa se procesaran correctamente.
* **Desfase en el mapa original**: Durante las primeras pruebas de navegación, el robot colisionaba repetidamente en cierta pared de la casa a pesar de que la planificación geométrica de la ruta indicaba que el camino era seguro y correcto. Tras depurar el proceso, detecté que existía un desfase importante: el mapa 2D proporcionado inicialmente tenía la puerta de una habitación modelada de manera distinta en Gazebo. La solución consistió en sustituir el mapa base por una versión más fidedigna, lo que eliminó las colisiones y permitió que el gridmap coincidiera con la realidad.
* **Efecto de órbita:** Durante la fase de navegación reactiva, el ruido en los actuadores y las limitaciones cinemáticas hacían que el robot a veces entrara en un bucle circular infinito alrededor de un waypoint sin llegar a validarlo por la distancia de tolerancia (`DIST_TOLERANCE`). El problema fue solucionado introduciendo una saturación dinámica en la velocidad lineal: si el valor absoluto del error angular supera los 0.4 radianes, el robot frena su avance (`V_CONST = 0.0`) y pivota sobre su propio eje. Una vez alineado, recupera su velocidad de crucero para continuar la ruta trazada.

## Resultado

Este es funcionamiento final de la práctica, donde se aprecia la planificación inicial de la ruta y la posterior ejecución de la misma:


https://github.com/user-attachments/assets/c1505e38-1201-4a07-a970-56c9003288bf


Y este es el resultado final:

<img width="765" height="558" alt="resultado" src="https://github.com/user-attachments/assets/e111ebc1-06f5-478c-9d23-a5c1791042d2" />

