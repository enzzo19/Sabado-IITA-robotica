import numpy

grilla = numpy.zeros((10,14))


def gridConfig(gridX,gridY):
    grilla[gridY,gridX] = 1
    print("Cuadricula:")
    print(grilla)
    print("Fin de la cuadricula")


gridConfig(1,1)
