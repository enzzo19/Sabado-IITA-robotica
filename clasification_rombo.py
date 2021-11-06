import cv2 as cv
import pytesseract

def clasif_rombos(img):

    if numero == "2":
        valor = "F"
    elif numero == "6":
        valor = "P"
    elif numero == "8" or numero == "3":
        valor == "C"
    else:
        valor = "O"
    return valor

img = cv.imread(r"erebus-v21.2.2\game\worlds\textures\placard-6-poison.png")
cv.imshow("rombo_rojo", img)

redimensionada = cv.resize(img, (100, 100), interpolation=cv.INTER_AREA)
cv.imshow("Resize", redimensionada)

cropped = redimensionada[70:90, 40:60]
cv.imshow("Cropped", cropped)

valor = funcionalejo(cropped)
print(clasif_rombos(valor))
# img_rgb = cv.cvtColor(cropped, cv.COLOR_BGR2RGB)
# print(pytesseract.image_to_string(img_rgb))
cv.waitKey(0)
