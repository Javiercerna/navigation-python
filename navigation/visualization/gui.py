from typing import Callable
from tkinter import *
from tkinter import ttk

from navigation.run_simulation import create_simulation
from navigation.visualization.plot import CanvasPlot


class GUI:
    def __init__(self) -> None:
        self.main_window = Tk()
        self.main_window.rowconfigure(0, weight=1)
        self.main_window.columnconfigure(0, weight=1)

        self.simulation = None
        self.simulation_active = False
        self.restart_simulation = False
        self.simulation_options = {}

        self.canvas = None
        self.buttons = {}
        self.option_menus = {}

    def _load_content_frame(self) -> None:
        content_frame = ttk.Frame(self.main_window, padding=10)
        content_frame.grid(row=0, column=0, sticky=(N, W, E, S))

        self._load_option_menu(
            frame=content_frame,
            options=['FixedReferencePlanner', 'SplinePlanner'],
            label_text='Planner',
            row=2,
            column=1,
        )

        self._load_option_menu(
            frame=content_frame,
            options=['PurePursuit'],
            label_text='Lateral Controller',
            row=2,
            column=2,
        )

        self._load_option_menu(
            frame=content_frame,
            options=['FixedLinearVelocityController'],
            label_text='Longitudinal Controller',
            row=2,
            column=3,
        )

        self._load_button(
            frame=content_frame,
            text='Start',
            command=self._start_simulation,
            row=2,
            column=4,
        )

        self._load_button(
            frame=content_frame,
            text='Stop',
            command=self._stop_simulation,
            row=2,
            column=5,
        )

        self._load_button(
            frame=content_frame,
            text='Restart',
            command=self._restart_simulation,
            row=2,
            column=6,
        )

        self.canvas = CanvasPlot(frame=self.main_window, row=3, column=0)

        for widget in content_frame.winfo_children():
            if isinstance(widget, ttk.Label):
                continue
            widget.grid_configure(padx=15)

    def _start_simulation(self) -> None:
        self.simulation_active = True

    def _stop_simulation(self) -> None:
        self.simulation_active = False

    def _restart_simulation(self) -> None:
        self.restart_simulation = True

    def _handle_simulation(self, force_restart: bool = False) -> None:
        if force_restart or self.restart_simulation:
            self.simulation = create_simulation(self.simulation_options)
            self.canvas.attach_simulation(self.simulation)
            self.restart_simulation = False

        if self.simulation_active:
            self.simulation.update()
            self.canvas.show()

        self.main_window.after(50, self._handle_simulation)

    def _load_option_menu(
            self,
            frame: ttk.Frame,
            options: list[str],
            label_text: str,
            row: int,
            column: int,
        ) -> None:
        tkvar = StringVar(self.main_window)
        option_menu = ttk.OptionMenu(
            frame,
            tkvar,
            *[None, *options],
            command=self._option_menu_changed(label_text),
        )
        option_menu.grid(row=row, column=column)
        self.option_menus[label_text] = option_menu
        ttk.Label(frame, text=label_text).grid(row=row - 1, column=column)

    def _option_menu_changed(self, text):
        def option_changed(value):
            self.simulation_options[text] = value

            for option_menu_text in self.option_menus:
                if option_menu_text not in self.simulation_options:
                    return

            for button in self.buttons.values():
                if str(button['state']) == DISABLED:
                    button['state'] = NORMAL

            self._handle_simulation(force_restart=True)

        return option_changed

    def _load_button(
            self,
            frame: ttk.Frame,
            text: str,
            command: Callable,
            row: int,
            column: int,
        ) -> None:
        button = ttk.Button(frame, text=text, command=command, state=DISABLED)
        button.grid(row=row, column=column)
        self.buttons[text] = button

    def run(self) -> None:
        self._load_content_frame()
        self.main_window.mainloop()


if __name__ == '__main__':
    gui = GUI()
    gui.run()
