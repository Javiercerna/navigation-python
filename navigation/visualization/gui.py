from tkinter import *
from tkinter import ttk
from typing import Callable

from navigation.run_simulation import create_simulation
from navigation.simulation.options import REQUIRED_OPTIONS, get_simulation_options
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
        self.selected_options = {
            option_key: False
            for option_key in get_simulation_options()
        }

    def _load_content_frame(self) -> None:
        content_frame = ttk.Frame(self.main_window, padding=10)
        content_frame.grid(row=0, column=0, sticky=(N, W, E, S))

        self._load_canvas_plot_frame(frame=content_frame)
        self._load_options_frame(frame=content_frame)

    def _load_canvas_plot_frame(self, frame):
        canvas_plot_frame = ttk.Frame(frame, padding=10)
        canvas_plot_frame.grid(row=0, column=0, sticky=(N, W, E, S))

        self.canvas = CanvasPlot(frame=canvas_plot_frame, row=0, column=0)

    def _load_options_frame(self, frame):
        options_frame = ttk.Frame(frame, padding=10)
        options_frame.grid(row=0, column=1, sticky=(N, W, E, S))

        row = 1
        for key, options in get_simulation_options().items():
            self._load_option_menu(
                frame=options_frame,
                options=options,
                simulation_options_key=key,
                row=row,
                column=0,
            )
            row += 2

        button_commands = {
            'Start': self._start_simulation,
            'Pause': self._pause_simulation,
            'Restart': self._restart_simulation,
        }

        for text, command in button_commands.items():
            row += 1
            self._load_button(
                frame=options_frame, text=text, command=command, row=row, column=0
            )

    def _start_simulation(self) -> None:
        self.simulation_active = True

    def _pause_simulation(self) -> None:
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
            simulation_options_key: str,
            row: int,
            column: int,
        ) -> None:
        tkvar = StringVar(self.main_window)
        option_menu = ttk.OptionMenu(
            frame,
            tkvar,
            *[None, *options],
            command=self._option_menu_changed(simulation_options_key),
        )
        option_menu.grid(row=row, column=column, pady=(0, 15))
        label_text = f'Choose {simulation_options_key.lower()}'
        ttk.Label(frame, text=label_text).grid(row=row - 1, column=column)

    def _option_menu_changed(self, simulation_options_key):
        def option_changed(value):
            self.simulation_options[simulation_options_key] = value
            self.selected_options[simulation_options_key] = value is not None

            for option_key, option_selected in self.selected_options.items():
                if option_key in REQUIRED_OPTIONS and not option_selected:
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
