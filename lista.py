lista = [(12,32),(6,48),(40,6)]

x = 12
y = 33

if (range(x-2,x+2),range(y-2,y+2)) in lista:
    print("aveces")
if (range(x-2,x+2),range(y-2,y+2)) in lista:
    print("nunca")

if (x,y) in lista or (x+1,y+1) in lista or (x-1,x-1) in lista or (x-1,y) in lista or (x,y-1) in lista or (x+1,y) in lista or (x,y+1) in lista or (x-1,y+1) in lista or (x+1,y-1) in lista:
    print("siempre no hay nunca")