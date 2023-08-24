import sympy

# State variables
#  Velocity
vx = sympy.symbols("v_x")
vy = sympy.symbols("v_y")
vz = sympy.symbols("v_z")

# Observation variables
#  Acceleration
ax, ay, az = sympy.symbols("a_x a_y a_z")
#  Velocity measured by optical flow
vofx, vofy, vofz = sympy.symbols("v_ofx v_ofy v_ofz")

# Other variables
#  Time step
dt = sympy.symbols("dt")
#  Angular velocity
wx, wy, wz = sympy.symbols("w_x w_y w_z")
#  Position of the center of the image plane
rx, ry, rz = sympy.symbols("r_x r_y r_z")
#  Camera Z axis
zcx, zcy, zcz = sympy.symbols("z_cx z_cy z_cz")

# vector of state variables
x = sympy.Matrix([vx, vy, vz])  # .T

# vector of observation variables
z = sympy.Matrix([vofx, vofy, vofz])  # .T

# vector of other variables
#  angular velocity
w = sympy.Matrix([wx, wy, wz])  # .T

#  center of the image plane
r = sympy.Matrix([rx, ry, rz])  # .T

#  camera Z axis
zc = sympy.Matrix([zcx, zcy, zcz])  # .T

# acceleration
a = sympy.Matrix([ax, ay, az])  # .T

# state transition function
f = x + dt * a

# observation function
h = (w.cross(r) + x) - ((w.cross(r) + x).dot(zc)) * zc

# Jacobian of state transition function
F = f.jacobian(x)

# Jacobian of observation function
H = h.jacobian(x)

print("state transition function")
sympy.pprint(f)

print("observation function")
sympy.pprint(h)

print("Jacobian of state transition function")
sympy.pprint(F)

print("Jacobian of observation function")
sympy.pprint(H)
