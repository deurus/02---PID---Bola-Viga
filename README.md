# Bola Viga

La maqueta está realizada con piezas impresas en 3D para la ocasión y un par de varillas metálicas por donde se desliza la bola.

El funcionamiento es muy sencillo, disponemos de un micro interruptor para cambiar entre modo manual y automático, y de un potenciómetro que en modo manual mueve el ángulo del servo y por ende la inclinación de la viga (OP), y en modo automático cambia el punto de consigna (SP) de posición de la bola en la viga. El objetivo es situar una bola en una posición determinada de la viga mediante un controlador PID.

<p align="center">
  <img src="https://garikoitz.info/blog/wp-content/uploads/2022/06/Boceto03_bolaviga_control.png" width="350" alt="esquema">
</p>

Investigando un poco que tipo de sensores de distancia asequibles existen para Arduino, encontré unos cuantos y muchos ejemplos de uso, pero es diferente cuando se hace una prueba de un sensor a cuando le das un uso en el que se requiere cierta precisión. De todo lo que encontré me quedo con cuatro sensores de los que os dejo una tabla comparativa.

<p align="center">
  <img src="https://garikoitz.info/blog/wp-content/uploads/2021/12/Tabla_sensores.png" width="350" alt="esquema">
</p>

Tenéis más información en la entrada del blog: https://garikoitz.info/blog/2021/09/sintonizar-pid-con-arduino-sistema-bola-viga/

