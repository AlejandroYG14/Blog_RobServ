# Práctica 4 - Amazon Warehouse

El objetivo de esta práctica es programar un robot logístico capaz de recoger y reubicar de forma autónoma las 6 estanterías de un almacén. La navegación debe resolverse combinando planificación global de trayectorias mediante la biblioteca OMPL (Open Motion Planning Library) con un pilotaje reactivo basado en la posición del robot (`HAL.getPose3d()`).

He utilizado la documentación de usuario de Unibotics para esta práctica: https://jderobot.github.io/RoboticsAcademy/exercises/MobileRobots/amazon_warehouse/

## Funcionamiento

La solución se ha dividido en tres scripts independientes que abordan el problema de forma progresiva:

- **Apartado A** (`p4_a.py`): planificación puntual en espacio euclídeo *R²* para el robot cuadrado holonómico, engordando los obstáculos del mapa en función del tamaño del robot y de la estantería.
- **Apartado B** (`p4_b.py`): planificación para el mismo robot holonómico sin engordar los obstáculos, utilizando su geometría real en el trayecto de ida y la geometría de la estantería cargada en el trayecto de vuelta.
- **Apartado C** (`p4_c.py`): navegación con el robot rectangular de cinemática Ackermann, incorporando sus restricciones de giro mínimo y maniobras marcha atrás.

En los tres casos el mapa se actualiza dinámicamente a medida que se mueven las estanterías, borrando las patas de su posición original al levantarlas y añadiéndolas como nuevos obstáculos al depositarlas en su destino.

### 1. Conversión de coordenadas y actualización del mapa

Al arrancar cada script se carga el mapa del almacén (`MAP_PATH`) con `WebGUI.getMap()` y se binariza en una matriz de ocupación `mask` (`1` para obstáculos y `0` para espacio libre). La correspondencia entre las coordenadas del simulador en metros y los píxeles de la imagen se realiza mediante `pix(x, y)`, usando una resolución `M_PER_PX = 0.05` m/píxel y el origen calibrado en la esquina superior izquierda (`ORIGIN_X = 7.02`, `ORIGIN_Y = 10.35`).

Para reflejar los cambios en el escenario sin necesidad de eliminar manualmente las estanterías en Gazebo, se han implementado dos funciones sobre `mask`:

- `remove_shelf(x, y)`: limpia con un círculo libre la zona de las patas de la estantería en cuanto la plataforma la eleva, ya que pasa a formar parte del robot.
- `add_shelf(x, y, yaw)`: dibuja las cuatro patas en la pose de entrega utilizando el desplazamiento `LEG_OFFSET = (0.99, 0.40)` rotado según `yaw`.

Cada actualización del mapa se envía al visor mediante `show_map()` (`WebGUI.showNumpy()`), y cada ruta calculada se representa con `show_path(pts)` (`WebGUI.showPath()`).

### 2. Planificación de trayectorias con OMPL

La función `plan_path()` construye el problema en OMPL sobre los límites del almacén (`X_RANGE`, `Y_RANGE`) y fija como criterio la longitud mínima mediante `ob.PathLengthOptimizationObjective`. Para buscar la ruta se realizan hasta `MAX_ATTEMPTS = 6` intentos de `OMPL_TIMEOUT = 3.0` segundos cada uno: en los primeros se utiliza `og.RRTstar` para obtener caminos optimizados y, si no obtiene una solución exacta, los dos últimos recurren a `og.RRTConnect` seguido de una simplificación con `og.PathSimplifier`.

La diferencia entre los tres apartados reside en el espacio de estados y en la validación de colisiones:

| Script | Espacio de estados | Mapa de colisión | Validación de estados |
| :--- | :--- | :--- | :--- |
| `p4_a.py` | `ob.RealVectorStateSpace(2)` | Obstáculos engordados con `cv2.dilate` (`get_inflated_mask`) | Comprobación puntual en `is_valid_point` usando `RAD_EMPTY = 0.49` m (ida) y `RAD_LOADED = 1.20` m (vuelta). En el visor el margen engordado se muestra en gris (`70`). |
| `p4_b.py` | `ob.ReedsSheppStateSpace(TURN_RAD)` con `TURN_RAD = 0.15` m | Mapa real sin engordar | Huella rectangular generada en `build_footprint` y rotada en `is_valid_pose`: `ROBOT_DIM = (0.55, 0.55)` m en ida y `CARGO_DIM = (2.06, 0.88)` m en vuelta, con `FOOTPRINT_PAD = 0.08` m. |
| `p4_c.py` | `ob.ReedsSheppStateSpace(PLAN_TURN_RAD)` con `PLAN_TURN_RAD = 1.3` m | Mapa real sin engordar | Huella rectangular desplazada al eje trasero (`AXLE_OFFSET = 0.244` m): `ROBOT_DIM = (1.15, 0.53)` m en ida y `CARGO_DIM = (2.06, 0.88)` m en vuelta, con `FOOTPRINT_PAD = 0.15` m. |

En `p4_b.py` se ha optado por un espacio de Reeds-Shepp con radio muy reducido (`0.15` m) en lugar de un `SE2StateSpace` libre para evitar desplazamientos laterales que el controlador diferencial no podría reproducir, manteniendo la posibilidad de girar prácticamente en el sitio.

### 3. Estrategia de traslado y pilotaje reactivo

Todos los traslados definidos en `DESTINATIONS` (con distintas orientaciones) siguen el mismo esquema en `main()` y `travel()`:

1. **Aproximación y entrada recta**: en lugar de planificar directamente hasta el centro de la estantería, se calcula una pose de espera (`wait_pose`) situada `ENTRY_DIST` metros por detrás del objetivo (`2.1` m en holonómico y `3.0` m en Ackermann) y alineada con él. OMPL planifica hasta ese punto y después se concatena una recta generada por `straight()` para entrar bajo la estantería sin rozar las patas.
2. **Seguimiento por tramos**: `split_by_sense()` divide la trayectoria en segmentos de marcha adelante (`+1`) y marcha atrás (`-1`). Entre un segmento y otro el robot se detiene completamente mediante `settle()`.
3. **Controlador reactivo (`follow_piece`)**:
   - **Holonómico (`p4_a.py` y `p4_b.py`)**: implementa persecución pura con distancia `PURSUIT_DIST = 0.4` m. Si el error angular respecto al punto objetivo supera `0.5` rad, gira sobre sí mismo antes de avanzar. Cuando lleva la estantería a bordo reduce sus límites de velocidad (`VEL_LIN_CARGO = 0.3`, `VEL_ANG_CARGO = 0.5`) y ajusta su orientación final con `rotate_to()` antes de accionar la plataforma.
   - **Ackermann (`p4_c.py`)**: el seguimiento se referencia al eje trasero del vehículo (restando `AXLE_OFFSET` a la pose del centro) con `PURSUIT_DIST = 0.7` m. La curvatura se calcula como *C = 2 · lateral/d²* y se acota con `CTRL_MIN_RAD = 0.9` m, dejando margen de corrección frente al radio de planificación (`1.3` m). Cada tramo arranca con una rampa cuadrática de velocidad desde `VEL_INIT = 0.02` hasta `VEL_CRUISE = 0.25` en `ACCEL_TIME = 3.0` s para absorber el retardo de las ruedas directrices.
4. **Carga, descarga y salida**: tras esperar `LIFT_DELAY = 2.0` s en tiempo simulado (`wait_sim`) al elevar o bajar la plataforma (`HAL.lift()` / `HAL.putdown()`), el robot sale en línea recta marcha atrás `EXIT_DIST = 2.2` m con `back_out()` para quedar en una zona despejada antes del siguiente traslado.


## Problemas y soluciones

- **Diferencia en los atributos de `HAL.getSimTime()`**: la documentación indicaba los campos `sec` y `nanosec`, pero al ejecutar el código en el simulador se producía un `AttributeError` porque el objeto `SimTimeData` exponía `seconds` y `nanoseconds`. Para evitar dependencias con la versión del entorno, encapsulé la consulta en `sim_time()` comprobando dinámicamente qué atributos existen.
  
- **Desfase entre las coordenadas de Gazebo y el mapa**: con una conversión aproximada las patas de las estanterías no coincidían con las posiciones reales y la ruta pintada en el visor aparecía desplazada. Calibré `ORIGIN_X`, `ORIGIN_Y` y `M_PER_PX` tomando como referencia las seis estanterías de `SHELVES` y añadí en `show_path()` los factores de escala propios del visor de `WebGUI`.

- **Visualización del mapa actualizado y del engordado**: en las primeras pruebas la máscara `mask` se actualizaba en memoria pero el visor no reflejaba los cambios ni distinguía el engordado del apartado (a). Lo solucioné llamando a `show_map()` desde `remove_shelf()`, `add_shelf()` y `plan_path()`, pintando además en `p4_a.py` la zona dilatada en gris (`70`) sobre los obstáculos reales en negro (`0`).

- **Estados inicial o final inválidos junto a las patas de las estanterías**: al pedir a OMPL que planificase directamente hasta el centro de una estantería, el estado meta quedaba bloqueado por las patas (o por su engordado en el modo puntual). Introducir el punto de espera alineado (`wait_pose`) seguido del tramo recto `straight()` y la maniobra de salida `back_out()` eliminó esos rechazos y evitó roces al cargar y descargar.

- **Imprecisión y derrapes en el robot Ackermann**: al controlar el Ackermann usando el centro devuelto por `HAL.getPose3d()`, el robot recortaba las curvas y llegaba desalineado a las estanterías, sobre todo en cambios de sentido rápidos. Desplazar tanto la huella de OMPL como el punto de control al eje trasero (`AXLE_OFFSET`), planificar con un radio más amplio que el límite del controlador (`1.3` m frente a `0.9` m), esperar a que el vehículo quede detenido en `settle()` y aplicar una rampa progresiva al arrancar resolvió el seguimiento.

- **Convergencia de `RRTstar` en maniobras con carga**: en algunos traslados con la estantería cargada, `RRTstar` agotaba el tiempo sin encontrar una ruta exacta. Añadir un bucle de reintentos que cambia a `RRTConnect` con simplificación en los últimos intentos garantizó encontrar solución en todos los traslados.

- **Gestión del espacio para las 6 estanterías**: dado que las estanterías depositadas se conservan en el escenario y se añaden al mapa, se diseñó la secuencia `DESTINATIONS` de modo que las tres primeras ocupan huecos libres del almacén y las tres últimas reutilizan los espacios que dejaron libres las anteriores.


## Vídeos del funcionamiento

En los siguientes vídeos se puede ver la ejecución de la práctica para cada uno de los apartados:

- Apartado A - Planificación puntual con obstáculos engordados:


- Apartado B - Geometrías diferenciadas ida/vuelta sin engordar:


- Apartado C - Robot rectangular Ackermann:

