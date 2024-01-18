import numpy as np
import matplotlib.pyplot as plt
import numpy.matlib as mat


def BasisFuncGauss(N, h, dt):
    T = int(round(1/dt+1))
    Phi = np.zeros((T-1,N))
    for z in range(0,T-1):
        t = z*dt
        phi = np.zeros((1, N))
        for k in range(1,N+1):
            c = (k-1)/(N-1)
            phi[0,k-1] = np.exp(-(t - c)*(t - c)/(2*h))
        Phi[z,:N] = phi[0, :N]
    Phi = Phi/np.transpose(mat.repmat(np.sum(Phi,axis=1),N,1)); #[TxN]   Normalizes the rows of self.Phi such that each row sums to 1.
    return Phi, T #[TxN]


result, T = BasisFuncGauss(N=4, h=0.015, dt=0.1)
print(T)

# Plot the matrix
time_values = np.arange(0, 1, 0.1)

# Plot each basis function over time
for i in range(result.shape[1]):
    plt.plot(time_values, result[:, i], label=f'Basis {i+1}')

plt.xlabel('Time')
plt.ylabel('Basis Function Value')
plt.title('Basis Functions Over Time')
plt.legend()
plt.show()