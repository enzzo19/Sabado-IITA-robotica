import cv2 as cv
from ReadDigits import Read_Digit
def clasif_rombos(numero):
    if numero == "2":
        valor = "F"
    elif numero == "6":
        valor = "P"
    elif numero == "8" :
        valor = "C"
    else:
        valor = "O"
    return valor
def Reconocimiento_Carteles_Rombos(imagen):
    img = cv.imread(imagen) #path a la img
    # cv.imshow("rombo_rojo", img)

    redimensionada = cv.resize(img, (100, 100), interpolation=cv.INTER_AREA)
    #cv.imshow("Resize", redimensionada)

    cropped = redimensionada[70:90, 42:55]
    #cv.imshow("Cropped", cropped)

    valor = Read_Digit(cropped)
    #print(clasif_rombos(valor.strip()))
    return clasif_rombos(valor.strip())
    # img_rgb = cv.cvtColor(cropped, cv.COLOR_BGR2RGB)
    # print(pytesseract.image_to_string(img_rgb))
    # cv.waitKey(0)

print(Reconocimiento_Carteles_Rombos("erebus-v21.2.2/game/worlds/textures/placard-6-poison.png"))