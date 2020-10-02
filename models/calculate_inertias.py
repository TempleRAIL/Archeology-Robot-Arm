#!/usr/bin/env python

s = 100
rho = 5000 # kg/m^3

V = [32.420815, 53.188896, 4.930576]

scaled_inertials = [ [111.292740, 9.156473, -16.609148, 152.018372, 5.356035, 59.590786],
[81.241760, -15.198512, -2.260159, 240.552139, -6.028688, 260.175842],
[2.572505, -1.579282, 0.045291, 3.972176, -0.385105, 4.561591] ]

true_inertials = []
m = [(volume/s**3)*rho for volume in V]


for tensor in scaled_inertials:
	true_inertials.append([(moment/s**5)*rho for moment in tensor])

print("Sherd_a inertials and mass: {}, {}kg".format(true_inertials[0], m[0]) )
print("Sherd_b inertials and mass: {}, {}kg".format(true_inertials[1], m[1]) )
print("Sherd_c inertials and mass: {}, {}kg".format(true_inertials[2], m[2]) )


