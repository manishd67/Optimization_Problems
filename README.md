# Optimization_Problems
Implementation of optimization problems taught in my institute course IME639 using CPLEX C++ API

- cutstock_CPLEX.cpp  - One dimenstional cutting stock problem


The cutting-stock problem is the problem of cutting standard-sized pieces of stock material,
such as paper rolls or sheet metal, into pieces of specified sizes while minimizing material wasted. 
It is an optimizationproblem in mathematics that arises from applications in industry.
In terms of computational complexity, the problem isan NP-hard problem reducible to the knapsack problem. 
The problem can be formulated as an integer linear programming problem.

- UFLP_CPLEX.cpp  - Uncapacitated Facility Location Problem

The goal of this problem is to choose a subset of facilities(plants, warehouses. etc) to open from a
given set of facilities to minimize the sum of transportation costs while meeting the demands of a 
client and the fixed costs of setting up the facilities. The key point to note is that
each facility has enough capacity to meet the demands of all clients.
