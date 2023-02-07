# ecuacion de una curva
import numpy as np
import matplotlib.pyplot as plt



# valores de los puntos
x = [1, 4, 2]
y = [1, 4, 3]

# encontrar coeficientes del polinomio
coefs = np.polyfit(x, y, 2)

# generar puntos equidistantes
num_points = 10
x_points = np.linspace(min(x), max(x), num_points)
y_points = coefs[0] * x_points**2 + coefs[1] * x_points + coefs[2]

# imprimir puntos
x_list = x_points.tolist()
x_list = [round(x,2) for x in x_list]
y_list = y_points.tolist()
y_list = [round(y,2) for y in y_list]

print("X LIST", x_list,"\nY LIST",y_list)

plt.plot(x_points, y_points)
plt.xlabel('x-axis')
plt.ylabel('y-axis')
plt.xlim(0, 5)
plt.ylim(0, 5)
plt.title('Mi curva')
plt.show()
