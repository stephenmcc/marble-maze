def min(x1, x2):
    if (x1 < x2):
        return x1
    else:
        return x2

def max(x1, x2):
    if (x1 > x2):
        return x1
    else:
        return x2

def alpha_filter(alpha, old_value, new_value):
    return alpha * new_value + (1.0 - alpha) * old_value

def signum(val):
    if (val > 0.0):
        return 1.0
    else:
        return -1.0

def clamp(val, min_max):
    if (val > min_max):
        return min_max
    elif (val < -min_max):
        return -min_max
    else:
        return val

def test_alpha_filter():
    x0 = 1.0
    xt = (1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.5, 1.5, 1.5)
    x = x0
    for xi in xt:
        x = alpha_filter(0.25, x, xi)
        print(str(x))

if __name__ == "__main__":
    test_alpha_filter()
