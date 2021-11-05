import numpy as np
import math
import skfuzzy as fuzz


def generateR1(X):
    R1 = np.zeros((len(X), len(X)))
    m = len(X[0])

    for i in range(len(X)):
        for j in range(len(X)):
            a = 0
            b = 0
            c = 0
            for k in range(m):
                a += X[i][k] * X[j][k]
                b += X[i][k] * X[i][k]
                c += X[j][k] * X[j][k]
            R1[i][j] = abs(a) / math.sqrt(b * c)
    return R1


def findR(R1):
    R = R1
    Last_R = np.zeros_like(R1)
    while not (Last_R == R).all():
        Last_R = R
        R = fuzz.maxmin_composition(R1, R)
    return R


def alpha_cut_R(R, alpha):
    return (R > alpha).astype(int)


def find_class(R_alpha):
    k = len(R_alpha)
    flag = np.zeros(k)
    classes = []

    for i in range(k):
        if flag[i] == 0:
            new = R_alpha[i][i:k]
            index = np.where(new == 1)[0].tolist()
            offset_index = [idx + i for idx in index]
            for j in offset_index:
                flag[j] = 1
            classes.append(offset_index)
    return classes


if __name__ == "__main__":
    X = np.array([[0.1, 0.7, 0.2, 0.0],
                  [0.0, 0.5, 0.5, 0.0],
                  [0.2, 0.2, 0.2, 0.4],
                  [0.8, 0.1, 0.0, 0.1],
                  [0.3, 0.0, 0.4, 0.3],
                  [0.0, 0.4, 0.0, 0.6],
                  [0.5, 0.0, 0.4, 0.1],
                  [0.6, 0.3, 0.0, 0.1],
                  [0.0, 0.5, 0.1, 0.4],
                  [0.1, 0.6, 0.0, 0.3],
                  [0.3, 0.2, 0.1, 0.4],
                  [0.1, 0.5, 0.4, 0.0],
                  [0.2, 0.0, 0.2, 0.6],
                  [0.2, 0.6, 0.1, 0.1],
                  [0.1, 0.7, 0.1, 0.1],
                  [0.2, 0.4, 0.2, 0.2]])

    R1 = generateR1(X)
    R = findR(R1)

    R_alpha_0_4 = alpha_cut_R(R, 0.4)
    R_alpha_0_8 = alpha_cut_R(R, 0.8)

    class_0_4 = find_class(R_alpha_0_4)
    class_0_8 = find_class(R_alpha_0_8)

    print(f"R1: {R1}")
    print("\n-------------\n")
    print(f"R: {R}")
    print("\n-------------\n")
    print(f"R_alpha 0.4:\n{R_alpha_0_4}")
    print("\n-------------\n")
    print(f"R_alpha 0.8:\n{R_alpha_0_8}")
    print("\n-------------\n")
    print(f"classes 0.4:\n{class_0_4}")
    print("\n-------------\n")
    print(f"classes 0.8:\n{class_0_8}")

    first = False
    end = False
    start_alpha = -1
    end_alpha = -1
    for i in np.arange(0.8, 0.9, 0.00000001):
        R_alpha = alpha_cut_R(R, i)
        class_alpha = find_class(R_alpha)
        if len(class_alpha) == 3 and not first:
            start_alpha = i
            first = True

        if len(class_alpha) != 3 and not end and first:
            end_alpha = i
            break
    print(start_alpha)  # 0.8426648502143806
    print(end_alpha)  # 0.8571428602871292
