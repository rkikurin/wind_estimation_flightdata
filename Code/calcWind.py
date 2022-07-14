import numpy as np


def calcWind(alpha, beta, attitude, vel, tas):
    """Method for calculating three-dimensional velocity vector from aircraft flight data

    Args:
        alpha (float): angle of attack, radian
        beta (float): angle of sideslip, radian
        attitude (float): the body-axis Euler angles(phi, theta, psi), radian
        vel (float): INS velocity(INS_U, INS_V, INS_W), m/s
        tas (float): true airspeed, m/s

    Returns:
        float: wind vector (U, V, W), m/s
    """
    tmpC = np.sin(alpha) * np.cos(beta) * np.cos(attitude[0]) + np.sin(beta) * np.sin(attitude[0])  # C
    tmpD = np.cos(alpha) * np.cos(beta) * np.cos(attitude[1]) + tmpC * np.sin(attitude[1])  # D

    sin_gammma_a = np.cos(alpha) * np.cos(beta) * np.sin(attitude[1]) - tmpC * np.cos(attitude[1])
    cos_gammma_a = np.sqrt(1 - sin_gammma_a**2)

    if (tmpD == 0):
        tan_psi_a_tmp = 0
    else:
        tan_psi_a_tmp = (np.sin(beta) * np.cos(attitude[0]) - np.sin(alpha) * np.cos(beta) * np.sin(attitude[0])) / tmpD
    psi_a = np.arctan(tan_psi_a_tmp) + attitude[2]

    wind = np.zeros(3)
    wind[0] = vel[0] - tas * np.cos(psi_a) * cos_gammma_a
    wind[1] = vel[1] - tas * np.sin(psi_a) * cos_gammma_a
    wind[2] = vel[2] - tas * sin_gammma_a

    return wind


if __name__ == "__main__":
    calcWind()
