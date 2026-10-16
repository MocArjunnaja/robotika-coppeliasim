import tkinter as tk
from decimal import Decimal, ROUND_DOWN
from tkinter import ttk, messagebox

import numpy as np
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

import rk_methods as rk

ANALYTICAL_COLOR = '#555555'


def fmtError(e):
    # Same style as the original table: 3 significant digits,
    # scientific notation below 1e-5, otherwise truncated to 5 decimals
    v = float(f'{e:.3g}')
    if v == 0:
        return '0'
    if abs(v) < 1e-5:
        mantissa, exponent = f'{v:.3G}'.split('E')
        return f'{mantissa}E{int(exponent)}'
    d = Decimal(repr(v)).quantize(Decimal('0.00001'), rounding=ROUND_DOWN)
    return f'{d:f}'.rstrip('0').rstrip('.')


class RungeKuttaApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title('Runge Kutta')
        self.geometry('1250x820')
        self.minsize(1000, 700)
        self.results = {}
        self.buildInputs()
        self.buildMainPlot()
        self.buildTable()
        self.buildErrorPlot()
        self.run()

    # ---------- layout ----------
    def buildInputs(self):
        frame = ttk.Frame(self, padding=(10, 8))
        frame.grid(row=0, column=0, columnspan=2, sticky='w')
        self.inputs = {}
        for col, (name, default) in enumerate([('x1', '0'), ('x2', '5'), ('h', '0.1'), ('fs', '100')]):
            ttk.Label(frame, text=name).grid(row=0, column=col, padx=12, sticky='w')
            var = tk.StringVar(value=default)
            ttk.Entry(frame, textvariable=var, width=10).grid(row=1, column=col, padx=12)
            self.inputs[name] = var
        ttk.Button(frame, text='Run', command=self.run).grid(row=1, column=4, padx=12)
        ttk.Label(frame, text='dy/dx = sin(πx/2),  y(x1) = 0', foreground='#666666').grid(
            row=1, column=5, padx=20)

    def buildMainPlot(self):
        self.fig = Figure(figsize=(9, 3.6), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        self.canvas.get_tk_widget().grid(row=1, column=0, sticky='nsew', padx=(10, 0))

        select = ttk.Frame(self, padding=(10, 20))
        select.grid(row=1, column=1, sticky='n')
        ttk.Label(select, text='Select').grid(row=0, column=0, columnspan=2, sticky='w', pady=(0, 6))
        self.selected = {'Analytical': tk.BooleanVar(value=True)}
        rows = [('Analytical', ANALYTICAL_COLOR)] + [(k, rk.METHODS[k][3]) for k in rk.METHODS]
        for i, (key, color) in enumerate(rows, start=1):
            ttk.Checkbutton(select, text=key, command=self.redraw,
                            variable=self.selected.setdefault(key, tk.BooleanVar(value=(key == '6th')))
                            ).grid(row=i, column=0, sticky='w')
            swatch = tk.Canvas(select, width=40, height=10, highlightthickness=0)
            swatch.create_line(2, 5, 38, 5, fill=color, width=2)
            swatch.grid(row=i, column=1, padx=6)

    def buildTable(self):
        frame = ttk.Frame(self, padding=(10, 4))
        frame.grid(row=2, column=0, sticky='nsew')
        ttk.Label(frame, text='True Error Value').pack(anchor='w')
        columns = ['Iterasi'] + [rk.METHODS[k][1] for k in rk.TABLE_ORDER]
        self.table = ttk.Treeview(frame, columns=columns, show='headings', height=11)
        for c in columns:
            self.table.heading(c, text=c)
            self.table.column(c, width=90, anchor='w')
        scroll = ttk.Scrollbar(frame, orient='vertical', command=self.table.yview)
        self.table.configure(yscrollcommand=scroll.set)
        self.table.pack(side='left', fill='both', expand=True)
        scroll.pack(side='right', fill='y')

    def buildErrorPlot(self):
        frame = ttk.Frame(self, padding=(4, 4))
        frame.grid(row=2, column=1, sticky='nsew')
        top = ttk.Frame(frame)
        top.pack(anchor='w')
        ttk.Label(top, text='Error').pack(side='left')
        self.errorMode = tk.StringVar(value='Global')
        mode = ttk.Combobox(top, textvariable=self.errorMode, values=['Global', 'Local'],
                            width=8, state='readonly')
        mode.pack(side='left', padx=6)
        mode.bind('<<ComboboxSelected>>', lambda e: self.run())
        self.logScale = tk.BooleanVar(value=False)
        ttk.Checkbutton(top, text='log', variable=self.logScale, command=self.redraw).pack(side='left')

        self.errFig = Figure(figsize=(3.4, 2.8), dpi=100)
        self.errAx = self.errFig.add_subplot(111)
        self.errCanvas = FigureCanvasTkAgg(self.errFig, master=frame)
        self.errCanvas.get_tk_widget().pack(fill='both', expand=True)

        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)
        self.rowconfigure(2, weight=1)

    # ---------- computation ----------
    def readInputs(self):
        try:
            x1 = float(self.inputs['x1'].get())
            x2 = float(self.inputs['x2'].get())
            h = float(self.inputs['h'].get())
            fs = float(self.inputs['fs'].get())
        except ValueError:
            raise ValueError('x1, x2, h and fs must be numbers.')
        if h <= 0 or x2 <= x1 or fs <= 0:
            raise ValueError('Need h > 0, x2 > x1 and fs > 0.')
        return x1, x2, h, fs

    def run(self):
        try:
            x1, x2, h, fs = self.readInputs()
        except ValueError as e:
            messagebox.showerror('Input error', str(e))
            return
        self.x1, self.x2, self.fs = x1, x2, fs
        mode = self.errorMode.get()
        self.results = {k: rk.true_error(m[2], x1, x2, h, mode) for k, m in rk.METHODS.items()}
        self.fillTable()
        self.redraw()

    def fillTable(self):
        self.table.delete(*self.table.get_children())
        xs = self.results['1st'][0]
        for i, x in enumerate(xs):
            row = [f'{x:.10g}'] + [fmtError(self.results[k][2][i]) for k in rk.TABLE_ORDER]
            self.table.insert('', 'end', values=row)

    def redraw(self):
        if not self.results:
            return
        self.ax.clear()
        if self.selected['Analytical'].get():
            # fs = samples per unit x for the analytical curve
            n = max(int((self.x2 - self.x1)*self.fs), 2) + 1
            xa = np.linspace(self.x1, self.x2, n)
            self.ax.plot(xa, rk.analytical(xa, self.x1), color=ANALYTICAL_COLOR, lw=1.6, label='Analytical')
        for key, (label, _, _, color) in rk.METHODS.items():
            if self.selected[key].get():
                xs, ys, _ = self.results[key]
                self.ax.plot(xs, ys, color=color, lw=1.2, label=label)
        self.ax.grid(True, ls=':')
        self.ax.set_xlim(self.x1, self.x2)
        self.fig.tight_layout()
        self.canvas.draw_idle()

        self.errAx.clear()
        for key, (label, _, _, color) in rk.METHODS.items():
            if self.selected[key].get():
                xs, _, err = self.results[key]
                if self.logScale.get():
                    self.errAx.semilogy(xs[1:], err[1:], color=color, lw=1.2)
                else:
                    self.errAx.plot(xs, err, color=color, lw=1.2)
        if not self.logScale.get():
            self.errAx.set_ylim(-0.1, 0.1)
        self.errAx.set_xlim(self.x1, self.x2)
        self.errAx.grid(True, ls=':')
        self.errAx.tick_params(labelsize=7)
        self.errFig.tight_layout()
        self.errCanvas.draw_idle()


if __name__ == '__main__':
    RungeKuttaApp().mainloop()
