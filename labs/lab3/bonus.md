根据FVM有

$$
\begin{aligned}
\mathbf{P} &= \mathbf{F} \frac{\partial W}{\partial \mathbf{G}} \\
&= \mathbf{F} \mathbf{S} \\ 
&= \mathbf{U} \mathbf{D} \mathbf{V}^T \mathbf{S} \\
&= \mathbf{U} \mathbf{D} \mathbf{V}^T (2 \mu \mathbf{G} + \lambda \text{tr}(\mathbf{G}) \mathbf{I})
\end{aligned}
$$

其中

$$
\begin{aligned}
\mathbf{G} &= \frac{1}{2} (\mathbf{V} \mathbf{D}^2 \mathbf{V}^T - \mathbf{I}) \\
&= \frac{1}{2} \mathbf{V} (\mathbf{D}^2 - \mathbf{I}) \mathbf{V}^T
\end{aligned}
$$

把$\mathbf{G}$带入$\mathbf{P}$有

$$
\begin{aligned}
\mathbf{P} &= \mathbf{U} \mathbf{D} \mathbf{V}^T (2 \mu \mathbf{G} + \lambda \text{tr}(\mathbf{G}) \mathbf{I}) \\
&= 2 \mu \mathbf{U} \mathbf{D} \mathbf{V}^T \mathbf{G} + \lambda \text{tr}(\mathbf{G}) \mathbf{U} \mathbf{D} \mathbf{V}^T \\
&= \mu \mathbf{U} \mathbf{D} (\mathbf{D}^2 - \mathbf{I}) \mathbf{V}^T + \frac{\lambda}{2} \text{tr}(\mathbf{D}^2 - \mathbf{I}) \mathbf{U} \mathbf{D} \mathbf{V}^T \\
&= \mathbf{U} \bigg( \mu \mathbf{D} (\mathbf{D}^2 - \mathbf{I}) + \frac{\lambda}{2} \text{tr}(\mathbf{D}^2 - \mathbf{I}) \mathbf{D} \bigg) \mathbf{V}^T
\end{aligned}
$$

另一方面，根据SVD有

$$
\mathbf{P} = \mathbf{U} \ \text{diag}\bigg( \frac{\partial W}{\partial \lambda_i} \bigg) \ \mathbf{V}^T
$$

因此只需证明

$$
\mu \mathbf{D} (\mathbf{D}^2 - \mathbf{I}) + \frac{\lambda}{2} \text{tr}(\mathbf{D}^2 - \mathbf{I}) \mathbf{D} = \text{diag}\bigg( \frac{\partial W}{\partial \lambda_i} \bigg)
$$

以对角矩阵第一项$\lambda_0$为例，等式左边有

$$
\mu \lambda_0 (\lambda_0^2 - 1) + \frac{\lambda}{2} \lambda_0 (\lambda_0^2 + \lambda_1^2 + \lambda_2^2 - 3)
$$

而右边有

$$
\begin{aligned}
\frac{\partial W}{\partial \lambda_0}
=
\begin{bmatrix}
\frac{\partial I}{\partial \lambda_0} & \frac{\partial II}{\partial \lambda_0} & \frac{\partial III}{\partial \lambda_0}
\end{bmatrix}
\begin{bmatrix}
\frac{\partial W}{\partial I} \\ 
\frac{\partial W}{\partial II} \\ 
\frac{\partial W}{\partial III}
\end{bmatrix}
\end{aligned}
$$

对于StVK模型有

$$
\begin{aligned}
\begin{bmatrix}
\frac{\partial I}{\partial \lambda_0} & \frac{\partial II}{\partial \lambda_0} & \frac{\partial III}{\partial \lambda_0}
\end{bmatrix}
=
\begin{bmatrix}
2 \lambda_0 & 4 \lambda_0^3 & *
\end{bmatrix}
\end{aligned}
$$

$$
\begin{aligned}
\begin{bmatrix}
\frac{\partial W}{\partial I} \\ 
\frac{\partial W}{\partial II} \\ 
\frac{\partial W}{\partial III}
\end{bmatrix}
=
\begin{bmatrix}
s_0 (I-3) - \frac{s_1}{2} \\ 
\frac{s_1}{4} \\ 
0
\end{bmatrix}
\end{aligned}
$$

因此

$$
\begin{aligned}
\frac{\partial W}{\partial \lambda_0} &= 2 \lambda_0 [s_0 (I-3) - \frac{s_1}{2}] + s_1 \lambda_0^3 \\
&= 2 s_0 \lambda_0 (I - 3) + s_1 \lambda_0 (\lambda_0^2 - 1) \\
&= 2 s_0 \lambda_0 (\lambda_0^2 + \lambda_1^2 + \lambda_2^2 - 3) + s_1 \lambda_0 (\lambda_0^2 - 1)
\end{aligned}
$$

因此只需令拉梅参数$\lambda = 4 s_0$, $\mu = s_1$即可使左右两边相等。对于对角矩阵的另外两项可以类似地证明。