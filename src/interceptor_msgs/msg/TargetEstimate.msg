# Estimacion del movimiento del objetivo que da target_estimator a partir de las
# observaciones visuales (TargetSighting) y la posicion de la camara.
#
# El tamano del objetivo es DESCONOCIDO y se estima junto con el movimiento. La escala
# (position, velocity, size, range) solo es observable si el interceptor acelera; hasta
# entonces se queda cerca del prior y lo dice su covarianza. angular_range no depende
# de la escala y es fiable desde la primera observacion.

# stamp = instante de la ultima observacion usada. frame_id = frame del mundo (map).
std_msgs/Header header

# Centro del objetivo [m] y su velocidad [m/s], en header.frame_id.
geometry_msgs/Point position
geometry_msgs/Vector3 velocity

# Tamano (diametro) estimado del objetivo [m].
float64 size

# Covarianza 7x7 de [position (3), velocity (3), size], por filas.
float64[49] covariance

# Distancia camara-objetivo [m] (depende de la escala).
float64 range

# Distancia camara-objetivo en tamanos de objetivo (range / size); sin escala.
float64 angular_range

# Numero de observaciones incorporadas desde la ultima (re)inicializacion.
uint32 update_count

# Numero de reinicios del estimador desde que arranco el nodo (a peticion, por perdida o por
# divergencia). Sirve para descartar estimaciones anteriores a un reinicio pedido: pueden
# seguir llegando despues de pedirlo.
uint32 reset_count
