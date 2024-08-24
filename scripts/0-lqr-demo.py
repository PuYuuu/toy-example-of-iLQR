import numpy as np
import matplotlib.pyplot as plt


def solve_dare(A: np.matrix, B: np.matrix,
               Q: np.matrix, R: np.matrix, max_iter: int = 150) -> np.matrix:
    """
    solve a Discrete-time Algebraic Riccati Equation (DARE)
    """
    p = Q
    p_next = Q
    eps = 0.01

    for i in range(max_iter):
        p_next = A.T @ p @ A - A.T @ p @ B @ np.linalg.inv(R + B.T @ p @ B) @ B.T @ p @ A + Q
        if (abs(p_next - p)).max() < eps:
            break
        p = p_next

    return p_next


def main():
    A =np.mat('1.9500,  -0.0250,    -1.6000; \
              1.6000,   1.1000,     -3.2000; \
              0.4250,   0.185,      0.3000')
    B =np.mat('0 1 0; 1 1 1').T
    nu = 2
    nx = 3
    # simulate original system without control (open loop)
    N = 50  # number of discrete time steps
    x = np.mat(np.zeros((3, N)))
    x[:, 0] = np.array([[1], [2], [3]])
    for k in np.arange(N - 1):
        x[:, k + 1] = A @ x[:, k]
    time = np.arange(N)

    fig1, ax1 = plt.subplots(2, 2)
    ax1[0, 0].plot(time, x[0, :].T)
    ax1[0, 0].set_xlabel('t')
    ax1[0, 0].set_ylabel('x_1')
    ax1[0, 1].plot(time, x[1, :].T)
    ax1[0, 1].set_xlabel('t')
    ax1[0, 1].set_ylabel('x_2')
    ax1[1, 0].plot(time, x[2, :].T)
    ax1[1, 0].set_xlabel('t')
    ax1[1, 0].set_ylabel('x_3')
    ax1[1, 1].plot(time, np.zeros(N))
    ax1[1, 1].set_xlabel('t')
    ax1[1, 1].set_ylabel('||u||')
    ax1[1, 1].yaxis.set_major_formatter(plt.FuncFormatter(lambda x, _: '{:.2f}'.format(x)))
    fig1.suptitle('Open Loop')
    fig1.subplots_adjust(wspace=0.4, hspace=0.4)

    Q = np.eye(nx) 
    R = np.eye(nu)

    Pstar = solve_dare(A, B, Q, R)
    Kstar = np.linalg.inv(R + B.T @ Pstar @ B) @ B.T @ Pstar @ A
    print(f"Pstar = \n{Pstar}")
    print(f"Kstar = \n{Kstar}")

    x = np.mat(np.zeros((nx, N)))
    u = np.mat(np.zeros((nu, N)))
    norm_u = np.zeros(N)
    x[:, 0] = np.array([[1], [2], [3]])
    for k in np.arange(N - 1):
        u[:, k] = -Kstar @ x[:, k]
        norm_u[k] = np.linalg.norm(u[:, k])
        x[:, k+1] = A @ x[:, k] + B @ u[:, k]

    time = np.arange(N)
    fig2, ax2 = plt.subplots(2, 2)
    ax2[0, 0].plot(time,x[0, :].T)
    ax2[0, 0].set_xlabel('t')
    ax2[0, 0].set_ylabel('x_1')
    ax2[0, 1].plot(time, x[1,:].T)
    ax2[0, 1].set_xlabel('t')
    ax2[0, 1].set_ylabel('x_2')
    ax2[1, 0].plot(time, x[2,:].T)
    ax2[1, 0].set_xlabel('t')
    ax2[1, 0].set_ylabel('x_3')
    ax2[1, 1].plot(time, norm_u)
    ax2[1, 1].set_xlabel('t')
    ax2[1, 1].set_ylabel('||u||')
    fig2.suptitle('Closed Loop')
    fig2.subplots_adjust(wspace=0.4, hspace=0.4)

    plt.show()


if __name__ == '__main__':
    main()
