import cv2 as cv
from funciones_utiles.ReadDigits import Read_Digit
def clasif_rombos(numero):
    if numero == "2":
        valor = "F"
    elif numero == "6":
        valor = "P"
    elif numero == "8" :
        valor = "C"
    else:
        valor = "O"
    print(valor)
    return valor
def Reconocimiento_Carteles_Rombos(imagen):
    img = cv.imread(imagen) #path a la img
    #cv.imshow("rombo_rojo", img)

    redimensionada = cv.resize(img, (100, 100), interpolation=cv.INTER_AREA)
    #cv.imshow("Resize", redimensionada)

    cropped = redimensionada[70:90, 42:55]
    cv.imshow("Cropped", cropped)

    valor = Read_Digit(cropped)
    #print(clasif_rombos(valor.strip()))
    print(valor)
    return clasif_rombos(valor.strip())
    # img_rgb = cv.cvtColor(cropped, cv.COLOR_BGR2RGB)
    # print(pytesseract.image_to_string(img_rgb))
   
    
Reconocimiento_Carteles_Rombos(r"Imagenes\placard-2-flammable-gas.png")