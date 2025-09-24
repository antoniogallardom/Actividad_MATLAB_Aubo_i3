[Formulario_Actividad_Antonio_Gallardo.md](https://github.com/user-attachments/files/21950709/Formulario_Actividad_Antonio_Gallardo.md)


# **Exploración cinemática del cobot AUBO i3 con un visor interactivo**<img width="1365" height="719" alt="Captura de pantalla 2025-08-23 140420" src="https://github.com/user-attachments/assets/7916b42b-3296-44a4-88cf-63952355f5d8" />


**Autor: Antonio Gallardo Montti, Departmento de Ingeniería Eléctrica, Universidad de Santiago de Chile.**

[![View Actividad MATLAB Aubo i3 on File Exchange](https://www.mathworks.com/matlabcentral/images/matlab-file-exchange.svg)](https://la.mathworks.com/matlabcentral/fileexchange/181858-actividad-matlab-aubo-i3)

[![Open in MATLAB Online](https://www.mathworks.com/images/responsive/global/open-in-matlab-online.svg)](https://matlab.mathworks.com/open/github/v1?repo=Antonio Gallardo Montti/https://matlab.mathworks.com/open/fileexchange/v1?id=181858)

<!-- Begin Toc -->
![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.16934110.svg)
## Tabla de Contenidos
[**Exploración cinemática del cobot AUBO i3 con un visor interactivo**](#TMP_2d11)
 
[Tipo de actividad](#TMP_0a87)
 
[Área del conocimiento](#TMP_530d)
 
[Pre\-requisitos](#TMP_1b05)
 
[Resumen](#TMP_34ae)
 
[Objetivos de Aprendizaje](#TMP_45ad)
 
[Contexto](#TMP_3892)
 
[Materiales de Soporte y Referencia](#TMP_3529)
 
[Notas para los Educadores](#TMP_66c7)
 
[Evaluación Sugerida](#TMP_4d60)
 
<!-- End Toc -->
<a id="TMP_0a87"></a>

# Tipo de actividad

Laboratorio práctico guiado (demostración + ejercicios cortos). Se trabaja con una app compilada: el alumno manipula perillas y espinners de **6 juntas** y observa el movimiento del robot y una **esfera de trabajo** aproximada.

<a id="TMP_530d"></a>

# Área del conocimiento

Robótica, Automatización y Control (vínculos con CAD/Simulación y Manufactura).

<a id="TMP_1b05"></a>

# Pre\-requisitos

Cálculo lineal básico (vectores/matrices, rotaciones) y una primera exposición a cinemática de manipuladores en serie (juntas **revolutas**, configuración articular, pose del efector). Conocer nociones de URDF ayuda, pero **no** es necesario programar: la app está compilada.

<a id="TMP_34ae"></a>

# Resumen

Actividad de 90–120 minutos donde los estudiantes **mueven** las juntas de un AUBO i3 virtual (URDF+STL), comparan posturas, reconocen **límites articulares**, usan **Home** y **Reset Camera**, y muestran/ocultan una **esfera de alcance** para discutir el volumen de trabajo. La meta es construir intuición de **cinemática directa** sin derivaciones largas.

# Objetivos de Aprendizaje

Al terminar, el estudiante podrá :

-   **Mapear** configuraciones a la pose cualitativa del efector (cinemática directa a nivel conceptual). 
-  **Identificar** límites articulares y su efecto en posturas típicas (p. ej., “codo arriba/abajo”). 
-  **Describir** el espacio de trabajo como una **aproximación** (esfera de alcance) y sus restricciones. 
<a id="TMP_3892"></a>

# Contexto

Esta práctica ancla un bloque de cinemática con un brazo industrial realista. Se usa una app sencilla (sin fondo de IK/optimización) para **visualizar** la relación junta\-postura y discutir **alcance vs. diseño**. La esfera de trabajo incluida **no** modela autocolisión ni orientación: es un apoyo visual para conversar sobre escalas y límites.


**Actividad**

1.  **Inicio (10–15 min)**Presentación breve del modelo URDF del AUBO i3 y de la interfaz (perillas/espinners por junta, botones *Home*, *Reset Camera*, *Show/Hide Workspace*).
2. **Exploración de juntas (25–35 min)**En parejas, barrer 1–2 juntas dentro de sus límites; capturar 3–4 posturas representativas (captura + vector qqq); comentar cómo cambian **posición** y **orientación** del efector.
3. **Volumen de trabajo (20–25 min)**Activar la **esfera** y relacionar radio y centro con el primer eje; discutir zonas más densas/menos accesibles y el papel de los límites.
4. **Retos breves (20–30 min)**Intentar “tocar” puntos dados dentro de la esfera (solo con juntas); registrar qqq, distancia aproximada y cercanía a límites.
5. **Cierre (10 min)**Poner en común observaciones y ligarlas con conceptos de cinemática directa y diseño geométrico.

**Notas de uso de la app**

-  Movimiento “suave”: los cambios por perilla/espinner se limitan en velocidad para evitar saltos bruscos. 
-  *Home* ejecuta una animación hasta la configuración de referencia. 
-  *Reset Camera* recentra la vista y reposiciona la única luz (*headlight*). 
-  *Show/Hide Workspace* alterna una **esfera** (radio ≈ 0.66 m) centrada en la primera junta, con pequeño ajuste en Z. 
<a id="TMP_3529"></a>

# Materiales de Soporte y Referencia
-  **Paquete de recursos (GitHub):** carpeta con `aubo_i3.urdf`, `link0.stl … link6.stl` y el archivo P\-code/script principal. 
-  **Guía de instalación (offline) – MATLAB Runtime** 

1.  Descargar el [**MATLAB Runtime**](https://la.mathworks.com/products/compiler/matlab-runtime.html) de la versión con la que se compiló la app (R2025a (25.1)).
2. Instalar el Runtime (no requiere licencia).
3. Clonar o descargar el repositorio **GitHub** con URDF, STL y el ejecutable/instalador.
4. Si usa ejecutable: ejecutar el **installer** y seguir el asistente. Si prefiere MATLAB: abrir el script/P\-code en una carpeta que contenga el **URDF y las mallas STL**, y ejecutar.
5. Al iniciar la app, verificar que el robot se renderiza y que *Home*, *Reset Camera* y *Show Workspace* funcionan.

-  **Plantilla breve de reporte** (1–2 páginas) para registrar capturas y vectores qqq. 
-  **Referencias útiles:** 
-  Documentación de *Robotics System Toolbox*: `importrobot`, `show`. 
-  Hoja de datos/dibujo dimensional del AUBO i3 (para discutir escalas). 
<a id="TMP_66c7"></a>

# Notas para los Educadores
-  La app **funciona offline** con MATLAB Runtime; ideal para laboratorios con dificultad para mantener una conexión con la licencia. 
-  Si la escena “se aclara” tras varios *Reset Camera*, es la luz reposicionada; no hay acumulación de luces. 
-  En equipos antiguos, reduzca el tamaño de la ventana para mantener fluidez. 
-  Aclare que la esfera es **orientativa** (no verifica colisiones ni orientación del efector). 
-  Puede pedirse a los equipos que **sincronicen** sus capturas en un documento compartido para discusión. 
<a id="TMP_4d60"></a>

# Evaluación Sugerida
-  **Formativa:** observación del uso correcto de controles y respeto de límites; calidad de las anotaciones. 
-  **Sumativa:** mini\-reporte con 3 posturas documentadas (captura + vector q), comentarios sobre el alcance observado y reflexiones al intentar llegar a puntos dados. Rubrica sugerida: (1) Registro claro (30 %), (2) Análisis razonado (40 %), (3) Claridad y presentación (30 %)

