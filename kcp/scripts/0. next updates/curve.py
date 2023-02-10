# ecuacion de una curva
import numpy as np
import matplotlib.pyplot as plt
plt.show()

def curve(point1, point2, point3):
    # valores de los puntos
    x = [point1[0], point2[0], point3[0]]
    y = [point1[1], point2[1], point3[1]]

    # encontrar coeficientes del polinomio
    coefs = np.polyfit(x, y, 2)

    # generar puntos equidistantes
    num_points = 10
    x_points = np.linspace(min(x), max(x), num_points)
    y_points = coefs[0] * x_points**2 + coefs[1] * x_points + coefs[2]
      

    # imprimir puntos
    x_list = x_points.tolist()
    y_list = y_points.tolist()

    # Redondear
    x_list = [round(x,3) for x in x_list]    
    y_list = [round(y,3) for y in y_list]

    positions = zip(x_list, y_list)

    return list(positions)

def shape(centro, radio, esquinas):

    #Generar Puntos
    theta = np.linspace(0, 2*np.pi, esquinas+1)
    x = centro[0] + radio * np.cos(theta)
    y = centro[1] + radio * np.sin(theta)
    x_list = x.tolist()
    y_list = y.tolist()

    # Redondear
    x_list = [round(x,3) for x in x_list]    
    y_list = [round(y,3) for y in y_list]

    positions = zip(x_list, y_list)

    return list(positions)

test2=curve([1,1],[2,2],[3,1])
print(test2[8])

x0, y0, r = 2.5, 2.5, 2 #centro y radio
n_esquinas = 10 #numero de esquinas
test = shape((x0,y0),r,n_esquinas)
print(test[3], len(test))