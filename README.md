# IMPLEMENTACIÓN DE UN PROTOTIPO DE UN SISTEMA PARA AUTOMATIZACIÓN DE SUBESTACIONES ELÉCTRICAS SIGUIENDO LA NORMA IEC 61850 SOBRE UN SISTEMA DE COMUNICACIONES BASADO EN TIME-SENSITIVE NETWORKING ETHERNET

Prototipo para evaluar el comportamiento temporal de los mensajes IEC 61850 SV y GOOSE en una red TSN.
Prototipo realista de una bahía, dentro del sistema de automatización de una subestación eléctrica basada en la norma IEC 61850, con el fin de demostrar que es factible el uso de TSN para este propósito. Se utiliza este prototipo para verificar experimentalmente la eficacia del mapeo de prioridades sobre mensajes GOOSE y SV.

## Descripción del proyecto

Este proyecto implementa un prototipo realista de una bahía, dentro del sistema de automatización de una subestación eléctrica basada en la norma IEC 61850, con el fin de demostrar que es factible el uso de TSN para este propósito. Se utiliza este prototipo para verificar experimentalmente la eficacia del mapeo de prioridades sobre mensajes GOOSE y SV bajo diferentes condiciones de tráfico.

El prototipo incluye:
- Implementaciones de nodos lógicos para medición, protección y control
- Generación de mensajes SV y GOOSE
- Priorización y programación del tráfico TSN mediante TAS
- Una pequeña interfaz SCADA para visualización

## Arquitectura del sistema

El prototipo representa una subestación simplificada compuesta por tres nodos físicos conectados a través de un puente TSN.

Nodos lógicos implementados:
- TCTR / TVTR: medición
- PIOC: protección contra sobrecorriente
- PIOV: protección contra sobretensión
- CSWI: controlador de interruptores
- XCBR: disyuntor

//////////////![Network topology](docs/topology.png)

## Estructura del repositorio
src/
node_measurement/ # TCTR / TVTR implementation
node_protection/ # PIOC / PIOV
node_control/ # CSWI / XCBR

tsn_config/
tas_schedule/ # TAS configuration
gptp_config/ # gPTP synchronization

scada/
mini_scada/ # Visualization interface

docs/
figures/
project_report.pdf

## Requisitos

- Linux (probado en Ubuntu)
- GCC
- Make
- Librería [libiec6180-1.5](https://github.com/mz-automation/libiec61850/blob/v1.5)
- Switch compatible con TSN

## Compilation

Example:

"```bash
gcc node1.c -o node1 -lpthread
gcc node2.c -o node2 -lpthread"

## Execution

Run each node in a different terminal:

"```bash
./node1
./node2
./node3"

## Configuración TSN

El prototipo utiliza los siguientes mecanismos TSN:

- gPTP y PHC para la sincronización del reloj
- Prioridades IEEE 802.1Q
- Time-Aware Shaper (TAS)

Asignación de prioridades:

| Protocol | Priority |
|--------|--------|
| SV | 7 |
| GOOSE HP | 6 |
| GOOSE LP | 5 |

## Experiments

El prototipo se utilizó para evaluar:

/////////- Latencia de extremo a extremo.
/////////- Retraso de la red.
/////////- Fluctuación.
/////////- Impacto del tráfico de fondo.

Los experimentos se centran en el enlace más cargado de la topología para reproducir el peor escenario posible de la red.

## Referencias

- IEC 61850
- IEE 802.3
## Autor/es

Joan Pau Rodríguez Sureda

Trabajo final de grado:
IMPLEMENTACIÓN DE UN PROTOTIPO DE UN SISTEMA PARA AUTOMATIZACIÓN DE SUBESTACIONES ELÉCTRICAS SIGUIENDO LA NORMA IEC 61850 SOBRE UN SISTEMA DE COMUNICACIONES BASADO EN TIME-SENSITIVE NETWORKING ETHERNET
Universitat de los Illes Balears

## Descripción del proyecto

## Disclaimer

Este prototipo simplifica las PDU SV y GOOSE de acuerdo con el perfil IEC 61850-9-2LE con el fin de centrarse en el análisis de la sincronización de la red en lugar de en el modelado completo de datos.
