import math

vals = [1,2,3] #actions
N = len(vals)
M = 2 #num agents

for i in range(int(math.pow(N,M))):
    x = []
    # convert i to a base N number
    i2 = i
    for j in range(M):
        digit = i2%N
        x.append(vals[digit])
        i2 = int(i2/N)
    print x
        
