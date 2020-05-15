"""
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
"""

import numpy as np


class KalmanFilter:
    def __init__(self, F, B, H, x, P, Q, R):
        self.F_k = F
        self.B_k = B
        self.H_k = H
        self.x_k = x
        self.P_k = P
        self.Q_k = Q
        self.R_k = R

    def predict(self, u_k):
        self.x_k = np.dot(self.F_k, self.x_k) + np.dot(self.B_k, u_k)
        self.P_k = np.dot(np.dot(self.F_k, self.P_k), np.transpose(self.F_k)) + self.Q_k

    def update(self, z_k):
        y_k = z_k - np.dot(self.H_k, self.x_k)
        S_k = np.dot(self.H_k, np.dot(self.P_k, np.transpose(self.H_k))) + self.R_k
        K_k = np.dot(np.dot(self.P_k, np.transpose(self.H_k)), np.linalg.inv(S_k))
        self.x_k = self.x_k + np.dot(K_k, y_k)
        I = np.eye(self.H_k.shape[1])
        self.P_k = np.dot(I - np.dot(K_k, self.H_k), self.P_k)
