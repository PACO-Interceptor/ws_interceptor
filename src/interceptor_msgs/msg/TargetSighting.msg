# Una observacion del objetivo en un instante, tal y como sale del detector visual.
#
# Lleva juntos el bearing y el tamano angular porque ambos salen de la MISMA caja
# en el MISMO instante: separarlos en dos topics obligaria al consumidor a
# resincronizarlos.

# stamp = instante de captura de la imagen (no de la inferencia).
# frame_id = frame de la camara, en convenio FLU.
std_msgs/Header header

# Vector unitario que apunta de la camara al objetivo, en el frame de la cabecera.
geometry_msgs/Vector3 bearing

# Angulo subtendido por el objetivo [rad], calculado del ANCHO de la caja por la
# ley del coseno. Es invariante a la rotacion de la camara, al contrario que el
# ancho en pixeles. Vale 0.0 si no se pudo calcular (sin CameraInfo).
#
# No da distancia por si solo: cumple theta ~= l/r con l el tamano fisico del
# objetivo, desconocido. Su derivada temporal si da tiempo hasta el impacto,
# que es invariante a la escala.
float64 subtended_angle

# Confianza que dio el detector, en [0, 1].
float64 confidence

# Clase que dio el detector, tal cual.
string class_id
