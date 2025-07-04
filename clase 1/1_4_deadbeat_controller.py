import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
import control as ctrl

# Parámetros
Ts = 0.1  # Período de muestreo
N = 10  # Número de muestras

# Doble integrador continuo: G(s) = 1/s^2
num = [1]
den = [1, 0, 0]
G = ctrl.TransferFunction([1], [1,0,0])
print("Planta (tiempo continuo):", G)

# Discretización por ZOH
G_d = ctrl.c2d(G, Ts, method='zoh')
print("Planta (tiempo discreto):", G_d)

# Deadbeat controller para doble integrador: C(z) = (z-1)^2 / (z^2)
# (lleva la salida al valor deseado en 2 pasos)
C = ctrl.TransferFunction([1, -2, 1], [1, 0, 0],dt=Ts) / Ts**2
print("Controlador (tiempo discreto):", C)

# Sistema en lazo cerrado (deadbeat)
Gl_d = ctrl.minreal(C*G_d)
Gcl_d = ctrl.feedback(Gl_d, 1,)
Gcl_d = Gl_d 
print("Sistema en lazo cerrado (tiempo discreto):", Gcl_d)


plt.figure(1)
ctrl.pzmap(Gcl_d, plot=True)

# Simulación de la respuesta al escalón
n = np.arange(N)
t_k, y_k = ctrl.step_response(Gcl_d, T=Ts*n)
print("Respuesta al escalón:", y_k)

plt.figure(2)
plt.step(t_k, y_k, where='post')
plt.xlabel('Tiempo [s]')
plt.ylabel('Salida')
plt.title('Doble integrador con controlador deadbeat')
plt.grid()
#plt.show()


# Implementación
# Con la ley de control, ahora calculamos la entrada del sistema de tiempo
# continuo considerando que tenemos un ZOH (Zero-Order Hold) 

# Se calcula la ley de control
u_k = np.zeros_like(n)  # salida del controlador (entrada a la planta)
_, u_k = ctrl.step_response(C, T=Ts*n)

# Preparamos la entrada considerando que pasa por el ZOH
t_total = Ts * len(u_k)
t_step = 0.0001      # resolución del tiempo de simulación

# Tiempo de simulación de alta resolución
t = np.arange(0, t_total, t_step)

# Construir entrada u(t) como escalón por tramos (ZOH)
u = np.zeros_like(t)
for k, uu in enumerate(u_k):
    idx = (t >= k*Ts) & (t < (k+1)*Ts)
    u[idx] = uu 

t_out, y = ctrl.forced_response(G, T=t, U=u)
plt.figure(3)
plt.subplot(2,1,1); plt.grid()
plt.plot(t_out, y, label="y(t)")
plt.plot(t_k, y_k,'*', label="y[k]")
plt.legend()
plt.ylabel("y(t)")
plt.subplot(2,1,2); plt.grid()
plt.plot(t, u, label="Entrada u(t)")
plt.xlabel("Tiempo [s]")
plt.ylabel("u(t)")
plt.legend()
plt.show()
