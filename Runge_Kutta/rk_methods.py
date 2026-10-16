import numpy as np

# ODE dy/dx = f(x, y), y(x1) = Y0
# Identified from the photographed true error table: dy/dx = sin(pi*x/2), y(0) = 0
OMEGA = np.pi/2
Y0 = 0.


def f(x, y):
    return np.sin(OMEGA*x)


def analytical(x, x1=0., y1=Y0):
    # y(x) = y1 + (cos(w*x1) - cos(w*x))/w
    return y1 + (np.cos(OMEGA*x1) - np.cos(OMEGA*x))/OMEGA


# One step of each method: y_{i+1} = y_i + h*phi(x_i, y_i, h)

def euler(x, y, h):
    return y + h*f(x, y)


def midpoint(x, y, h):
    k1 = f(x, y)
    k2 = f(x + h/2, y + h/2*k1)
    return y + h*k2


def heun(x, y, h):
    k1 = f(x, y)
    k2 = f(x + h, y + h*k1)
    return y + h/2*(k1 + k2)


def ralston(x, y, h):
    k1 = f(x, y)
    k2 = f(x + 3*h/4, y + 3*h/4*k1)
    return y + h*(k1/3 + 2*k2/3)


def rk3(x, y, h):
    k1 = f(x, y)
    k2 = f(x + h/2, y + h/2*k1)
    k3 = f(x + h, y - h*k1 + 2*h*k2)
    return y + h/6*(k1 + 4*k2 + k3)


def rk4(x, y, h):
    k1 = f(x, y)
    k2 = f(x + h/2, y + h/2*k1)
    k3 = f(x + h/2, y + h/2*k2)
    k4 = f(x + h, y + h*k3)
    return y + h/6*(k1 + 2*k2 + 2*k3 + k4)


def rk5(x, y, h):
    # Butcher's fifth order RK
    k1 = f(x, y)
    k2 = f(x + h/4, y + h/4*k1)
    k3 = f(x + h/4, y + h/8*k1 + h/8*k2)
    k4 = f(x + h/2, y - h/2*k2 + h*k3)
    k5 = f(x + 3*h/4, y + 3*h/16*k1 + 9*h/16*k4)
    k6 = f(x + h, y - 3*h/7*k1 + 2*h/7*k2 + 12*h/7*k3 - 12*h/7*k4 + 8*h/7*k5)
    return y + h/90*(7*k1 + 32*k3 + 12*k4 + 32*k5 + 7*k6)


def rk6(x, y, h):
    # Butcher's sixth order RK (7 stages)
    k1 = f(x, y)
    k2 = f(x + h/3, y + h/3*k1)
    k3 = f(x + 2*h/3, y + 2*h/3*k2)
    k4 = f(x + h/3, y + h*(k1/12 + k2/3 - k3/12))
    k5 = f(x + h/2, y + h*(-k1/16 + 9*k2/8 - 3*k3/16 - 3*k4/8))
    k6 = f(x + h/2, y + h*(9*k2/8 - 3*k3/8 - 3*k4/4 + k5/2))
    k7 = f(x + h, y + h*(9*k1/44 - 9*k2/11 + 63*k3/44 + 18*k4/11 - 16*k6/11))
    return y + h*(11*k1/120 + 27*k3/40 + 27*k4/40 - 4*k5/15 - 4*k6/15 + 11*k7/120)


# key: (label in legend, column name in table, step function, plot color)
METHODS = {
    '1st':          ('1st',          'First',   euler,    '#e6c229'),
    '2nd_Ralston':  ('2nd_Ralston',  'SecondR', ralston,  '#3fb950'),
    '2nd_Midpoint': ('2nd_Midpoint', 'SecondM', midpoint, '#2ea8a8'),
    '2nd_Heun':     ('2nd_Heun',     'SecondH', heun,     '#4c8de0'),
    '3rd':          ('3rd',          'Third',   rk3,      '#d4b106'),
    '4th':          ('4th',          'Fourth',  rk4,      '#1f4fa0'),
    '5th':          ('5th',          'Fifth',   rk5,      '#7a3fb0'),
    '6th':          ('6th',          'Sixth',   rk6,      '#b03f7a'),
}
# Table column order as in the original app
TABLE_ORDER = ['1st', '2nd_Midpoint', '2nd_Heun', '2nd_Ralston', '3rd', '4th', '5th', '6th']


def solve(step, x1, x2, h):
    n = int(round((x2 - x1)/h))
    xs = x1 + h*np.arange(n + 1)
    ys = np.zeros(n + 1)
    ys[0] = analytical(x1, x1)
    for i in range(n):
        ys[i + 1] = step(xs[i], ys[i], h)
    return xs, ys


def true_error(step, x1, x2, h, mode='Global'):
    """Global: |y_exact(x_i) - y_i| accumulated over all steps.
    Local: error of one step started from the exact value y_exact(x_{i-1})."""
    xs, ys = solve(step, x1, x2, h)
    exact = analytical(xs, x1)
    if mode == 'Global':
        err = np.abs(exact - ys)
    else:
        err = np.zeros_like(xs)
        for i in range(1, len(xs)):
            err[i] = abs(exact[i] - step(xs[i-1], exact[i-1], h))
    return xs, ys, err


if __name__ == '__main__':
    # Print the true error table (same layout as the original app)
    x1, x2, h = 0., 5., 0.1
    cols = {}
    for key in TABLE_ORDER:
        xs, _, err = true_error(METHODS[key][2], x1, x2, h)
        cols[key] = err
    print('x     ' + ''.join(f'{METHODS[k][1]:>11s}' for k in TABLE_ORDER))
    for i in range(12):
        print(f'{xs[i]:<6.1f}' + ''.join(f'{cols[k][i]:>11.3G}' for k in TABLE_ORDER))
