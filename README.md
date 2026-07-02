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

## Autor/es

Joan Pau Rodríguez Sureda

Trabajo final de grado:
IMPLEMENTACIÓN DE UN PROTOTIPO DE UN SISTEMA PARA AUTOMATIZACIÓN DE SUBESTACIONES ELÉCTRICAS SIGUIENDO LA NORMA IEC 61850 SOBRE UN SISTEMA DE COMUNICACIONES BASADO EN TIME-SENSITIVE NETWORKING ETHERNET
Universitat de los Illes Balears
