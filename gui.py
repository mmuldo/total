from typing import Optional
import tkinter as tk

import serialio
import sensor

PADDING=5

def frequency_row(frame: tk.Widget, row: int) -> tk.IntVar:
    '''
    creates a row where user can input frequency inside specified frame

    Parameters
    ----------
    frame : tk.Widget
        the frame where this row will be created
    row : int
        the row number inside the frame

    Returns
    -------
    tk.IntVar
        the frequency variable, in case it needs to be used
    '''
    frequency = tk.IntVar()
    frequency_label = tk.Label(frame, text='Frequency: ')
    frequency_label.grid(
        row=row,
        column=0,
        sticky='W',
        padx=PADDING,
        pady=PADDING,
    )
    frequency_entry = tk.Entry(frame, textvariable=frequency)
    frequency_entry.grid(
        row=row,
        column=1,
        sticky='W',
        padx=PADDING,
        pady=PADDING,
    )
    frequency_unit = tk.Label(frame, text='Hz')
    frequency_unit.grid(
        row=row,
        column=2,
        sticky='W',
        padx=PADDING,
        pady=PADDING,
    )
    run_button = tk.Button(frame, text='Run', command=run)
    run_button.grid(
        row=row,
        column=3,
        sticky='E',
        padx=PADDING,
        pady=PADDING,
    )
    return frequency

def log_row(frame: tk.Widget, row: int) -> tk.StringVar:
    '''
    creates a row in which program can log messages to user

    Parameters
    ----------
    frame : tk.Widget
        the frame where this row will be created
    row : int
        the row number inside the frame

    Returns
    -------
    tk.StingVar
        the log variable, in case it needs to be used
    '''
    log = tk.StringVar()
    log_label = tk.Label(frame, textvariable=log)
    log_label.grid(
        row=row,
        column=0,
        columnspan=4,
        sticky='W',
        padx=PADDING,
        pady=PADDING,
    )
    return log

def output_row(frame: tk.Widget, row: int, name: str, units: str) -> tk.StringVar:
    '''
    creates a row where user can view an output

    Parameters
    ----------
    frame : tk.Widget
        the frame where this row will be created
    row : int
        the row number inside the frame
    name : str
        the name of the output
    units : str
        the units of the output

    Returns
    -------
    tk.StringVar
        the output variable, in case it needs to be used
    '''
    output = tk.StringVar()
    output_label = tk.Label(frame, text=f'{name}: ')
    output_label.grid(
        row=row,
        column=0,
        sticky='W',
        padx=PADDING,
        pady=PADDING,
    )
    output_label = tk.Label(frame, textvariable=output)
    output_label.grid(
        row=row,
        column=1,
        sticky='W',
        padx=PADDING,
        pady=PADDING,
    )
    output_unit = tk.Label(frame, text=f'{units}')
    output_unit.grid(
        row=row,
        column=2,
        sticky='W',
        padx=PADDING,
        pady=PADDING,
    )
    return output

def settings_row(
    frame: tk.Widget,
    row: int,
    name: str,
    filetype: str = ''
) -> tuple[tk.BooleanVar, Optional[tk.StringVar]]:
    '''
    creates a settings row in the specified fram

    Parameters
    ----------
    frame : tk.Widget
        the frame where this row will be created
    row : int
        the row number inside the frame
    name : str
        the name of this setting
    filetype : str
        the filetype of files where data related to this setting can be saved;
        default is '', in which case no save file entry will be created

    Returns
    -------
    tk.BooleanVar
        the setting variable, in case it needs to be used
    Optional[tk.StringVar]
        the save file variable corresponding to the setting variable;
        if filetype is not specified, this will be None
    '''
    setting_file = None
    setting = tk.BooleanVar(value=True)
    setting_checkbutton = tk.Checkbutton(
        frame,
        text=name,
        variable=setting
    )
    setting_checkbutton.grid(
        row=row,
        column=0,
        sticky='W',
        padx=PADDING,
        pady=PADDING,
    )
    if filetype:
        # if filetype is specified, then create entry for user to enter save file
        setting_checkbutton.config(command=lambda: setting_file_entry.config(state='normal' if setting.get() else 'disabled'))
        setting_file = tk.StringVar()
        setting_file_label = tk.Label(
            frame,
            text=f'File ({filetype}): ',
        )
        setting_file_label.grid(
            row=row,
            column=2,
            sticky='E',
            padx=PADDING,
            pady=PADDING,
        )
        setting_file_entry = tk.Entry(
            frame,
            textvariable=setting_file,
        )
        setting_file_entry.grid(
            row=row,
            column=3,
            sticky='E',
            padx=PADDING,
            pady=PADDING,
        )
    return setting, setting_file

def input_frame(root: tk.Tk, row: int) -> tuple[tk.IntVar, tk.StringVar]:
    '''
    creates a label frame where user can input data

    Parameters
    ----------
    root : tk.Tk
        top level widget
    row : int
        row in root where this frame should be placed

    Returns
    -------
    tk.IntVar
        the frequency variable
    tk.StringVar
        the log variable
    '''
    frame = tk.LabelFrame(root, text="Input")
    frame.grid(
        row=row,
        sticky='WE',
        padx=PADDING,
        pady=PADDING,
        ipadx=PADDING,
        ipady=PADDING
    )
    frequency = frequency_row(frame, row=0)
    log = log_row(frame, row=1)
    return frequency, log

def output_frame(root: tk.Tk, row: int) -> tuple[tk.StringVar, tk.StringVar, tk.StringVar]:
    '''
    creates a label frame where user can view output data

    Parameters
    ----------
    root : tk.Tk
        top level widget
    row : int
        row in root where this frame should be placed

    Returns
    -------
    tk.StringVar
        the impedence variable
    tk.StringVar
        the temperature variable
    tk.StringVar
        the pressure variable
    '''
    frame = tk.LabelFrame(root, text="Output")
    frame.grid(
        row=row,
        sticky='WE',
        padx=PADDING,
        pady=PADDING,
        ipadx=PADDING,
        ipady=PADDING
    )
    impedence = output_row(frame, row=0, name='Impedence', units='Ohms')
    temperature = output_row(frame, row=1, name='Temperature', units='C')
    pressure = output_row(frame, row=2, name='Pressure', units='mBars')
    return impedence, temperature, pressure

def settings_frame(
    root: tk.Tk,
    row: int
) -> tuple[
    tk.BooleanVar,
    tk.StringVar,
    tk.BooleanVar,
    tk.BooleanVar,
    tk.StringVar,
]:
    '''
    creates a label frame where user can change settings

    Parameters
    ----------
    root : tk.Tk
        top level widget
    row : int
        row in root where this frame should be placed

    Returns
    -------
    tk.BooleanVar
        the save_output variable
    tk.StringVar
        the output_file variable
    tk.BooleanVar
        the show_figure variable
    tk.BooleanVar
        the save_figure variable
    tk.StringVar
        the figure_file variable
    '''
    frame = tk.LabelFrame(root, text="Settings")
    frame.grid(
        row=row,
        sticky='WE',
        padx=PADDING,
        pady=PADDING,
        ipadx=PADDING,
        ipady=PADDING
    )
    save_output, output_file = settings_row(frame, row=0, name='Save Output', filetype='CSV')
    show_figure, _ = settings_row(frame, row=1, name='Show Figure')
    save_figure, figure_file = settings_row(frame, row=2, name='Save Figure', filetype='PNG')
    return save_output, output_file, show_figure, save_figure, figure_file

def run():
    try:
        freq = frequency.get()
    except tk.TclError:
        log.set('Bad frequency input: must be integer.')
        return

    if freq > sensor.MAX_FREQ_HZ or freq < sensor.MIN_FREQ_HZ:
        # if frequency invalid, exit
        log.set(f'Bad frequency input: please specify integer frequency between {sensor.MIN_FREQ_HZ} to {sensor.MAX_FREQ_HZ} Hz')
        return

    # if frequency looks good, proceed and reset log message
    log.set('')

    save_out = save_output.get()
    out_file = output_file.get()
    show_fig = show_figure.get()
    save_fig = save_figure.get()
    fig_file = figure_file.get()

    # initialize serial i/o
    ser = serialio.init_serial()

    # get measurements
    Z, T, P = sensor.get_measurements(
        freq,
        ser,
        out_file if save_out else '',
        show_fig,
        fig_file if save_fig else ''
    )

    # set output variables
    impedence.set(sensor.format_impedence(Z))
    temperature.set(str(T))
    pressure.set(str(P))

root = tk.Tk()

root.wm_title('Total Water Quality Sensor')

frequency, log = input_frame(root, row=0)
impedence, temperature, pressure = output_frame(root, row=1)
save_output, output_file, show_figure, save_figure, figure_file = settings_frame(root, row=2)

root.mainloop()
