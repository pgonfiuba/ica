# Ejemplo de Código para Curso de Identificación y Control Adaptativo  
Repositorio del curso de **Identificación y Control Adaptativo** (ICA)  

## Descripción  
Este repositorio contiene los códigos de ejemplo, los notebooks y los ejercicios desarrollados para el curso de Identificación y Control Adaptativo.  
Los distintos temas del curso están organizados en carpetas por unidad (introducción, sistemas discretos, modelo de perturbaciones, no paramétrica, identificación paramétrica, predictores, reguladores clásicos, gain scheduling, STR, etc.).  
En cada carpeta encontrarás notebooks Jupyter (`.ipynb`) que ilustran los conceptos teóricos con simulaciones, gráficos y experimentos prácticos.  

## Estructura del repositorio  
- `0. Introducción/` – Presentación del curso, fundamentos de identificación y control adaptativo.  
- `1. Sistemas discretos/` – Modelado de sistemas discretos, transformaciones, etc.  
- `2. Modelo de perturbaciones/` – Cómo incorporar modelos de perturbaciones y ruido, ejemplos prácticos.  
- `3. No paramétrica/` – Métodos no paramétricos de identificación, estimación de funciones de transferencia, etc.  
- `4. Identificación paramétrica/` – Ajuste de modelos paramétricos (ARX, ARMAX, OE…), validación y selección de modelos.  
- `5. Modelos y Predictores/` – Diseño de predictores, estimación de estados, predicción futura y uso en control.  
- `6. Reguladores Clásicos/` – Diseño de controladores clásicos (PID, ajuste por IMC, Ajuste IFT) como punto de partida.  
- `7. Gain Scheduling/` – Técnicas de programación de ganancia para sistemas con variación de parámetros.  
- `8. STR/` – Algoritmo de Auto-Regresión con Retroalimentación (Self‐Tuning Regulator) y ejemplos de código.  
- `README.md` – Este archivo con la información general del repositorio.  

## Requisitos  
Para ejecutar los notebooks se recomienda:  
- Python 3.11 o superior  
- Instalar dependencias 
  `pip install -r requirements.txt`
- Tener Jupyter Notebook o JupyterLab instalado  

## Cómo usarlo  
1. Clonar el repositorio:  
   ```bash
   git clone https://github.com/pgonfiuba/ica.git
   ```  
2. Navegar a la carpeta del tema correspondiente:  
   ```bash
   cd ica/"2. Modelo de perturbaciones"/
   ```  
3. Abrir el notebook de ejemplo:  
   ```bash
   jupyter notebook nombre_del_notebook.ipynb
   ```  
4. Ejecutar las celdas, modificar parámetros, experimentar con los ejemplos y seguir la explicación del curso.  

## ¿Para quién es?  
Este repositorio está pensado para estudiantes de ingeniería de control y automatización, y para cualquier profesional interesado en profundizar en la identificación de sistemas y el diseño de control adaptativo.  

## Aporte y colaboración  
Si encontrás errores (tipográficos, de código, de documentación) o querés proponer mejoras (nuevos ejemplos, mejores visualizaciones, más ejercicios), abrí un *Issue* o un *Pull Request*. Toda contribución es bienvenida.  

## Licencia  
Este material se distribuye bajo la licencia Creative Commons Atribución 4.0 Internacional (CC BY 4.0)

