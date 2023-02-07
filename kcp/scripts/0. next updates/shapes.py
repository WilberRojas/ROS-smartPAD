import numpy as np
import matplotlib.pyplot as plt
import pprint

x0, y0, r = 2.5, 2.5, 2
theta = np.linspace(0, 2*np.pi, 8)
x = x0 + r * np.cos(theta)
y = y0 + r * np.sin(theta)

x_list = x.tolist()
y_list = y.tolist()
print("X LIST", x_list,"Y LIST",y_list)
plt.plot(x, y)
plt.xlabel('x-axis')
plt.ylabel('y-axis')
plt.xlim(0, 5)
plt.ylim(0, 5)
plt.title('Mi curva')
plt.show()
plt.show()