import cv2
import pytesseract
def Read_Digit(image):
    #Importante, debe apuntar a donde tengan instalado tesseract
    pytesseract.pytesseract.tesseract_cmd = r"C:\Program Files\Tesseract-OCR\tesseract.exe"
    #image = cv2.imread('crop-corrosive.png')
    # Grayscale, Gaussian blur, Otsu's threshold
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (1,1), 0)
    thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)[1]

    # inviere imagen, remueve ruido mediante Morph
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1,1))
    opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=1)
    invert = 255 - opening

    # Extrae numero de imagen
    data = pytesseract.image_to_string(invert, lang='eng', config='--psm 10 --oem 3 -c tessedit_char_whitelist=0123456789')
    print(data)
    #cv2.imshow('thresh', thresh)
    #cv2.imshow('opening', opening)
    #cv2.imshow('invert', invert)
    #cv2.waitKey()
    return data