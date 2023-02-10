import numpy as np
import matplotlib.pyplot as plt

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

    plt.scatter(x_points, y_points)
    plt.xlabel('x-axis')
    plt.ylabel('y-axis')
    
    plt.title('Mi curva')
    plt.show()

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

#test2=curve([0.4919, -0.0936], [0.4920, 0.0313], [0.5232, -0.0311])

""" 


x0, y0, r = 2.5, 2.5, 2 #centro y radio
n_esquinas = 10 #numero de esquinas
test = shape((x0,y0),r,n_esquinas)
print(test[3], len(test)) 
"""

""" 
Example Output:
- curve:
[(1.0, 1.0), (1.222, 1.395), (1.444, 1.691), (1.667, 1.889), (1.889, 1.988), (2.111, 1.988), (2.333, 1.889), (2.556, 1.691), (2.778, 1.395), (3.0, 1.0)]
- shape:
[(4.5, 2.5), (4.118, 3.676), (3.118, 4.402), (1.882, 4.402), (0.882, 3.676), (0.5, 2.5), (0.882, 1.324), (1.882, 0.598), (3.118, 0.598), (4.118, 1.324), (4.5, 2.5)]
"""