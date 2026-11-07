# Bikebrake

Luz de freno para bici usando los sensores del móvil.

- `index.html`: la app (PWA instalable, funciona sin conexión).
- `luz_freno.ino`: firmware ESP32 para una luz Bluetooth (opcional).
- `detector.js` y `test.js`: el algoritmo de detección y su simulación (`node test.js`).

En el móvil viejo: abre la página, pulsa «Modo luz trasera» y fíjalo al sillín con la pantalla hacia atrás.

Detección de frenada: mediana corta + acumulador con fuga (el ruido del pedaleo no reinicia la cuenta).
Pendiente: se estima con el acelerómetro y la aceleración medida por GPS y se muestra en pantalla.
«Poner a 0» (calle llana) corrige el desfase de calibrado.
