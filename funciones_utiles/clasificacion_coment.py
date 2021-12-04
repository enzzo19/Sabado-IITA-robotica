# Importamos la libreria a utilizar 
import cv2 as cv

# leemos una imagen alojada en nuestra PC
img = cv.imread(r"C:/Users/enzzo/OneDrive/Escritorio/H.png")

# redimensionamos la imagen
img = cv.resize(img, (100, 100))

# la pasamos a escala de grises
gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

# creamos una imagen binaria a partir del valor 100
thresh1 = cv.threshold(gray, 100, 255, cv.THRESH_BINARY_INV)[1]

# contamos la cantidad de contornos
conts, h = cv.findContours(thresh1, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
# print(f"conts: {conts}, h: {h}")


x, y, w, h = cv.boundingRect(conts[0])
# print(x, y, w, h)

# marcamos exactamente la letra en la imagen
letter = thresh1[y:y + h, x:x + w]

# redimensionamos la letra
letter = cv.resize(letter, (100, 100), interpolation=cv.INTER_AREA)


# valores para definir los espacios de interes
areaWidth = 20
areaHeight = 30
areas = {
    "top": ((0, areaHeight),
            (50 - areaWidth // 2, 50 + areaWidth // 2)),
    "middle": ((50 - areaHeight // 2, 50 + areaHeight // 2),
               (50 - areaWidth // 2, 50 + areaWidth // 2)),
    "bottom": ((100 - areaHeight, 100),
               (50 - areaWidth // 2, 50 + areaWidth // 2))
    }
# print(areas)

# vemos lo que hay dentro de las areas de interes pero en nuestra imagen
images = {
    "top": letter[areas["top"][0][0]:areas["top"][0][1],
                  areas["top"][1][0]:areas["top"][1][1]],
    "middle": letter[areas["middle"][0][0]:areas["middle"][0][1],
                     areas["middle"][1][0]:areas["middle"][1][1]],
    "bottom": letter[areas["bottom"][0][0]:areas["bottom"][0][1],
                     areas["bottom"][1][0]:areas["bottom"][1][1]]
    }
counts = {}
acceptanceThreshold = 50
for key in images.keys():
    count = 0
    for row in images[key]:
        for pixel in row:
            if pixel == 255:
                count += 1
    counts[key] = count > acceptanceThreshold
letters = {
    "H": {'top': False,
          'middle': True,
          'bottom': False},
    "S": {'top': True,
          'middle': True,
          'bottom': True},
    "U": {'top': False,
          'middle': False,
          'bottom': True}
    }
for letterKey in letters.keys():
    if counts == letters[letterKey]:
        finalLetter = letterKey
        break
print(finalLetter)
cv.waitKey(0)
